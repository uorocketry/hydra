#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod types;

use atsamd_hal as hal;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::clock::v2::Source;
use atsamd_hal::dmac;
use atsamd_hal::dmac::DmaController;
use common_arm::mcan;
use common_arm::SdManager;
use common_arm::*;
use communication::Capacities;
use communication::{RadioDevice, RadioManager};
use data_manager::DataManager;
// use communication::radio_dma;

use defmt::{flush, info};
use hal::gpio::Pins;
use hal::gpio::{Alternate, Pin, PushPull, PushPullOutput, C, PA05, PB12, PB13, PB14, PB15};
use hal::prelude::*;
use hal::sercom::{spi, spi::Config, spi::Duplex, spi::Pads, spi::Spi, IoSet1, Sercom4};
use heapless::Vec;
use mcan::messageram::SharedMemory;
use messages::command::RadioRate;
use messages::sensor::Sensor;
use messages::state::State;
use messages::*;
use panic_halt as _;
use systick_monotonic::*;
use types::*;

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {
    use hal::gpio::Output;

    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        data_manager: DataManager,
        can0: communication::CanDevice0,
    }

    #[local]
    struct Local {
        led: Pin<PA05, PushPullOutput>,
        radio: RadioDevice,
        sd_manager: SdManager<
            Spi<
                Config<
                    Pads<
                        hal::pac::SERCOM4,
                        IoSet1,
                        Pin<PB15, Alternate<C>>,
                        Pin<PB12, Alternate<C>>,
                        Pin<PB13, Alternate<C>>,
                    >,
                >,
                Duplex,
            >,
            Pin<PB14, Output<PushPull>>,
        >,
        radio_manager: RadioManager,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init(local = [
        #[link_section = ".can"]
        can_memory: SharedMemory<Capacities> = SharedMemory::new()
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut peripherals = cx.device;
        let core = cx.core;
        let pins = Pins::new(peripherals.PORT);

        let mut dmac = DmaController::init(peripherals.DMAC, &mut peripherals.PM);
        let dmaChannels = dmac.split();

        /* Logging Setup */
        HydraLogging::set_ground_station_callback(queue_gs_message);

        /* Clock setup */
        let (_, clocks, tokens) = atsamd_hal::clock::v2::clock_system_at_reset(
            peripherals.OSCCTRL,
            peripherals.OSC32KCTRL,
            peripherals.GCLK,
            peripherals.MCLK,
            &mut peripherals.NVMCTRL,
        );
        let gclk0 = clocks.gclk0;

        // SAFETY: Misusing the PAC API can break the system.
        // This is safe because we only steal the MCLK.
        let (_, _, _, mclk) = unsafe { clocks.pac.steal() };

        /* CAN config */
        let (pclk_can, gclk0) = Pclk::enable(tokens.pclks.can0, gclk0);
        let (can0, gclk0) = communication::CanDevice0::new(
            pins.pa23.into_mode(),
            pins.pa22.into_mode(),
            pclk_can,
            clocks.ahbs.can0,
            peripherals.CAN0,
            gclk0,
            cx.local.can_memory,
            false,
        );

        /* Radio config */
        let dmaCh0 = dmaChannels.0.init(dmac::PriorityLevel::LVL3);
        let (radio, gclk0, xfer) = RadioDevice::new(
            tokens.pclks.sercom5,
            &mclk,
            peripherals.SERCOM5,
            pins.pb17,
            pins.pb16,
            gclk0,
            dmaCh0,
        );

        let radio_manager = RadioManager::new(xfer);

        /* SD config */
        let (pclk_sd, gclk0) = Pclk::enable(tokens.pclks.sercom4, gclk0);
        let pads_spi = spi::Pads::<Sercom4, IoSet1>::default()
            .sclk(pins.pb13)
            .data_in(pins.pb15)
            .data_out(pins.pb12);
        let sdmmc_spi = spi::Config::new(&mclk, peripherals.SERCOM4, pads_spi, pclk_sd.freq())
            .length::<spi::lengths::U1>()
            .bit_order(spi::BitOrder::MsbFirst)
            .spi_mode(spi::MODE_0)
            .enable();
        let sd_manager = SdManager::new(sdmmc_spi, pins.pb14.into_push_pull_output());

        /* Status LED */
        let led = pins.pa05.into_push_pull_output();

        /* Spawn tasks */
        sensor_send::spawn().ok();
        state_send::spawn().ok();
        blink::spawn().ok();

        /* Monotonic clock */
        let mono = Systick::new(core.SYST, gclk0.freq().to_Hz());

        (
            Shared {
                em: ErrorManager::new(),
                data_manager: DataManager::new(),
                can0,
            },
            Local {
                led,
                radio,
                sd_manager,
                radio_manager,
            },
            init::Monotonics(mono),
        )
    }

    /// Idle task for when no other tasks are running.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    /// Handles the CAN0 interrupt.
    #[task(priority = 3, binds = CAN0, shared = [can0, data_manager])]
    fn can0(mut cx: can0::Context) {
        cx.shared.can0.lock(|can| {
            cx.shared
                .data_manager
                .lock(|data_manager| can.process_data(data_manager));
        });
    }
    /**
     * Sends a message over CAN.
     */
    #[task(capacity = 10, local = [counter: u16 = 0], shared = [can0, &em])]
    fn send_internal(mut cx: send_internal::Context, m: Message) {
        cx.shared.em.run(|| {
            cx.shared.can0.lock(|can| can.send_message(m))?;
            Ok(())
        });
    }

    /// Probably should use this ( ﾉ^ω^)ﾉ
    pub fn queue_gs_message(d: impl Into<Data>) {
        let message = Message::new(&monotonics::now(), COM_ID, d.into());

        send_gs::spawn(message).ok();
    }

    /**
     * Sends a message to the radio over UART.
     */
    #[task(capacity = 10, local = [radio], shared = [&em])]
    fn send_gs(cx: send_gs::Context, m: Message) {
        cx.shared.em.run(|| {
            cx.local.radio.send_message(m)?;
            Ok(())
        });
    }

    #[task(capacity = 10, local = [sd_manager], shared = [&em])]
    fn sd_dump(cx: sd_dump::Context, m: Message) {
        let manager = cx.local.sd_manager;
        cx.shared.em.run(|| {
            let mut buf: [u8; 255] = [0; 255];
            let msg_ser = postcard::to_slice_cobs(&m, &mut buf)?;
            if let Some(mut file) = manager.file.take() {
                manager.write(&mut file, &msg_ser)?;
                manager.file = Some(file);
            } else if let Ok(mut file) = manager.open_file("log.txt") {
                manager.write(&mut file, &msg_ser)?;
                manager.file = Some(file);
            }
            Ok(())
        });
    }

    /**
     * Sends information about the sensors.
     */
    #[task(shared = [data_manager, &em])]
    fn sensor_send(mut cx: sensor_send::Context) {
        let (sensor_data, logging_rate) = cx.shared.data_manager.lock(|data_manager| {
            (
                data_manager.clone_sensors(),
                data_manager.get_logging_rate(),
            )
        });

        let messages = sensor_data
            .into_iter()
            .flatten()
            .map(|x| Message::new(&monotonics::now(), COM_ID, Sensor::new(x)));

        cx.shared.em.run(|| {
            for msg in messages {
                spawn!(send_gs, msg.clone())?;
                spawn!(sd_dump, msg)?;
            }
            Ok(())
        });
        match logging_rate {
            RadioRate::Fast => {
                spawn_after!(sensor_send, ExtU64::millis(250)).ok();
            }
            RadioRate::Slow => {
                spawn_after!(sensor_send, ExtU64::millis(2000)).ok();
            }
        }
    }

    #[task(shared = [data_manager, &em])]
    fn state_send(mut cx: state_send::Context) {
        let state_data = cx
            .shared
            .data_manager
            .lock(|data_manager| data_manager.state.clone());
        cx.shared.em.run(|| {
            if let Some(x) = state_data {
                let message = Message::new(&monotonics::now(), COM_ID, State::new(x));
                spawn!(send_gs, message)?;
            } // if there is none we still return since we simply don't have data yet.
            Ok(())
        });
        spawn_after!(state_send, ExtU64::secs(5)).ok();
    }

    /**
     * Simple blink task to test the system.
     * Acts as a heartbeat for the system.
     */
    #[task(local = [led], shared = [&em])]
    fn blink(cx: blink::Context) {
        cx.shared.em.run(|| {
            cx.local.led.toggle()?;
            if cx.shared.em.has_error() {
                spawn_after!(blink, ExtU64::millis(200))?;
            } else {
                spawn_after!(blink, ExtU64::secs(1))?;
            }
            Ok(())
        });
    }

    // extern "Rust" {
    //     #[task(binds = DMAC_0, shared=[&em], local=[radio_manager])]
    //     fn radio_dma(context: radio_dma::Context);
    // }
}
