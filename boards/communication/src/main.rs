#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod health;
mod types;

use atsamd_hal as hal;
use common_arm::mcan;
use common_arm::HealthManager;
use common_arm::HealthMonitor;
use common_arm::SdManager;
use common_arm::*;
use communication::Capacities;
use communication::{RadioDevice, RadioManager};
use data_manager::DataManager;
use hal::adc::Adc;
use hal::clock::v2::pclk::Pclk;
use hal::clock::v2::Source;
use hal::gpio::Pins;
use hal::gpio::{
    Alternate, Output, Pin, PushPull, PushPullOutput, C, PA05, PB12, PB13, PB14, PB15,
};
use hal::prelude::*;
use hal::sercom::{spi, spi::Config, spi::Duplex, spi::Pads, spi::Spi, IoSet1, Sercom4};
use health::HealthMonitorChannelsCommunication;
use heapless::Vec;
use mcan::messageram::SharedMemory;
use messages::command::RadioRate;
use messages::health::Health;
use messages::state::State;
use messages::*;
use systick_monotonic::*;
use types::*;

/// Custom panic handler.
/// Reset the system if a panic occurs.
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    atsamd_hal::pac::SCB::sys_reset();
}

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        data_manager: DataManager,
        health_manager: HealthManager<HealthMonitorChannelsCommunication>,
        radio_manager: RadioManager,
        can0: communication::CanDevice0,
    }

    #[local]
    struct Local {
        led: Pin<PA05, PushPullOutput>,
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

        // let mut dmac = DmaController::init(peripherals.DMAC, &mut peripherals.PM);
        // let dmaChannels = dmac.split();

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
        let (_, _, _, mut mclk) = unsafe { clocks.pac.steal() };

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
        let (radio, gclk0) = RadioDevice::new(
            tokens.pclks.sercom5,
            &mclk,
            peripherals.SERCOM5,
            pins.pb17,
            pins.pb16,
            gclk0,
        );

        let radio_manager = RadioManager::new(radio);

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

        /* Setup ADC clocks */
        let (_pclk_adc0, gclk0) = Pclk::enable(tokens.pclks.adc0, gclk0);
        let (_pclk_adc1, gclk0) = Pclk::enable(tokens.pclks.adc1, gclk0);
        /* Setup ADC */
        let adc0 = Adc::adc0(peripherals.ADC0, &mut mclk);
        let adc1 = Adc::adc1(peripherals.ADC1, &mut mclk);

        /* Setup Health Monitor */
        let health_monitor_channels = HealthMonitorChannelsCommunication::new(
            adc0,
            adc1,
            pins.pb01.into(),
            pins.pb02.into(),
            pins.pb03.into(),
            pins.pb00.into(),
            pins.pb06.into(),
            pins.pb07.into(),
            pins.pb08.into(),
            pins.pb09.into(),
            pins.pb05.into(),
        );

        let health_monitor = HealthMonitor::new(health_monitor_channels, 10000, 5000, 1023);
        let health_manager = HealthManager::new(health_monitor);

        /* Status LED */
        let led = pins.pa05.into_push_pull_output();

        /* Spawn tasks */
        sensor_send::spawn().ok();
        state_send::spawn().ok();
        blink::spawn().ok();
        report_health::spawn().ok();

        /* Monotonic clock */
        let mono = Systick::new(core.SYST, gclk0.freq().to_Hz());

        (
            Shared {
                em: ErrorManager::new(),
                data_manager: DataManager::new(),
                health_manager,
                radio_manager,
                can0,
            },
            Local { led, sd_manager },
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

    /// Receives a log message from the custom logger so that it can be sent over the radio.
    pub fn queue_gs_message(d: impl Into<Data>) {
        let message = Message::new(&monotonics::now(), COM_ID, d.into());

        send_gs::spawn(message).ok();
    }

    /**
     * Sends a message to the radio over UART.
     */
    #[task(capacity = 10, shared = [&em, radio_manager])]
    fn send_gs(mut cx: send_gs::Context, m: Vec<u8, 255>) {
        cx.shared.radio_manager.lock(|radio_manager| {
            cx.shared.em.run(|| {
                radio_manager.send_message(m)?;
                Ok(())
            })
        });
    }

    #[task(capacity = 10, local = [sd_manager], shared = [&em])]
    fn sd_dump(cx: sd_dump::Context, m: Vec<u8, 255>) {
        let manager = cx.local.sd_manager;
        cx.shared.em.run(|| {
            let mut buf: [u8; 255] = m.into_array()?;
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
        let (sensors, logging_rate) = cx.shared.data_manager.lock(|data_manager| {
            (
                data_manager.stuff_messages(),
                data_manager.get_logging_rate(),
            )
        });

        cx.shared.em.run(|| {
            spawn!(send_gs, sensors.clone())?;
            spawn!(sd_dump, sensors)?;
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
     * Simple health report
     */
    #[task(shared = [&em, health_manager])]
    fn report_health(mut cx: report_health::Context) {
        cx.shared.em.run(|| {
            let msg = cx.shared.health_manager.lock(|health_manager| {
                let state = health_manager.evaluate();
                Message::new(
                    &monotonics::now(),
                    COM_ID,
                    Health::new(health_manager.monitor.data.clone(), state),
                )
            });
            spawn!(send_gs, msg)?;
            spawn_after!(report_health, ExtU64::secs(5))?;
            Ok(())
        });
    }

    #[task(binds = SERCOM5_2, shared = [&em, radio_manager])]
    fn radio_rx(mut cx: radio_rx::Context) {
        cx.shared.radio_manager.lock(|radio_manager| {
            cx.shared.em.run(|| {
                let msg = radio_manager.receive_message()?;
                spawn!(send_internal, msg)?; // just broadcast the message throughout the system for now.
                Ok(())
            });
        });
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
