#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod types;

use atsamd_hal as hal;
use common_arm::HealthManager;
use common_arm::HealthMonitor;
use common_arm::SdManager;
use common_arm::*;
use common_arm_atsame::mcan;
use communication::Capacities;
use communication::{RadioDevice, RadioManager};
use data_manager::DataManager;
use defmt::info;
use hal::adc::Adc;
use hal::clock::v2::pclk::Pclk;
use hal::clock::v2::Source;
use hal::gpio::Pins;
use hal::gpio::{
    Alternate, Output, Pin, PushPull, PushPullOutput, C, D, PA02, PA04, PA05, PA06, PA07, PB12,
    PB13, PB14, PB15,
};
use hal::prelude::*;
use hal::sercom::uart;
use hal::sercom::{
    spi, spi::Config, spi::Duplex, spi::Pads, spi::Spi, IoSet1, IoSet3, Sercom0, Sercom2, Sercom4,
};
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
        radio_manager: RadioManager,
        can0: communication::CanDevice0,
    }

    #[local]
    struct Local {
        led: Pin<PA02, PushPullOutput>,
        gps_uart: GpsUart,
        //sd_manager: SdManager<
        //   Spi<
        //       Config<
        //            Pads<
        //                hal::pac::SERCOM0,
        //                IoSet3,
        //               Pin<PA07, Alternate<D>>,
        //              Pin<PA04, Alternate<D>>,
        //             Pin<PA05, Alternate<D>>,
        //         >,
        //     >,
        //     Duplex,
        // >,
        // Pin<PA06, Output<PushPull>>,
        //>,
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
            tokens.pclks.sercom2,
            &mclk,
            peripherals.SERCOM2,
            pins.pa08,
            pins.pa09,
            gclk0,
        );

        let radio_manager = RadioManager::new(radio);

        /* SD config */
        //let (pclk_sd, gclk0) = Pclk::enable(tokens.pclks.sercom0, gclk0);
        //let pads_spi = spi::Pads::<Sercom0, IoSet3>::default()
        //   .sclk(pins.pa05)
        //  .data_in(pins.pa07)
        // .data_out(pins.pa04);
        //let sdmmc_spi = spi::Config::new(&mclk, peripherals.SERCOM0, pads_spi, pclk_sd.freq())
        //   .length::<spi::lengths::U1>()
        //  .bit_order(spi::BitOrder::MsbFirst)
        //  .spi_mode(spi::MODE_0)
        //  .enable();
        //let sd_manager = SdManager::new(sdmmc_spi, pins.pa06.into_push_pull_output());

        let (pclk_gps, gclk0) = Pclk::enable(tokens.pclks.sercom4, gclk0);
        //info!("here");
        let pads = hal::sercom::uart::Pads::<hal::sercom::Sercom4, IoSet3>::default()
            .rx(pins.pa13)
            .tx(pins.pa12);

        let mut gps_uart = GpsUartConfig::new(&mclk, peripherals.SERCOM4, pads, pclk_gps.freq())
            .baud(
                9600.Hz(),
                uart::BaudMode::Fractional(uart::Oversampling::Bits16),
            )
            .enable();
        gps_uart.enable_interrupts(hal::sercom::uart::Flags::RXC);

        /* Status LED */
        let led = pins.pa02.into_push_pull_output();
        let mut gps_enable = pins.pb09.into_push_pull_output();
        //gps_enable.set_high().ok();
        gps_enable.set_low().ok();
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
                radio_manager,
                can0,
            },
            Local { led, gps_uart },
            init::Monotonics(mono),
        )
    }

    /// Idle task for when no other tasks are running.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(binds = SERCOM2_2, local = [gps_uart], shared = [&em])]
    fn gps_rx(mut cx: gps_rx::Context) {
        cx.shared.em.run(|| {
            let byte = cx.local.gps_uart.read().unwrap();
            info!("GPS: {}", byte);
            Ok(())
        });
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
    fn send_gs(mut cx: send_gs::Context, m: Message) {
        cx.shared.radio_manager.lock(|radio_manager| {
            cx.shared.em.run(|| {
                radio_manager
                    .send_message(postcard::to_slice(&m, &mut [0; 255])?.try_into().unwrap())?;
                Ok(())
            })
        });
    }

    //   #[task(capacity = 10, local = [sd_manager], shared = [&em])]
    //  fn sd_dump(cx: sd_dump::Context, m: Message) {
    //     let manager = cx.local.sd_manager;
    //    cx.shared.em.run(|| {
    //        let mut buf: [u8; 255] = [0; 255];
    //       let msg_ser = postcard::to_slice_cobs(&m, &mut buf)?;
    //      if let Some(mut file) = manager.file.take() {
    //         manager.write(&mut file, &msg_ser)?;
    //         manager.file = Some(file);
    //    } else if let Ok(mut file) = manager.open_file("log.txt") {
    //       manager.write(&mut file, &msg_ser)?;
    //       manager.file = Some(file);
    //   }
    //   Ok(())
    // });
    //}

    /**
     * Sends information about the sensors.
     */
    #[task(shared = [data_manager, &em])]
    fn sensor_send(mut cx: sensor_send::Context) {
        let (sensors, logging_rate) = cx
            .shared
            .data_manager
            .lock(|data_manager| (data_manager.take_sensors(), data_manager.get_logging_rate()));

        cx.shared.em.run(|| {
            for msg in sensors {
                match msg {
                    Some(x) => {
                        spawn!(send_gs, x.clone())?;
                        //                     spawn!(sd_dump, x)?;
                    }
                    None => {
                        continue;
                    }
                }
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
