#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod gps_manager;
mod types;

use atsamd_hal as hal;
use atsamd_hal::dmac;
use atsamd_hal::rtc::Count32Mode;
use common_arm::*;
use common_arm_atsame::mcan;
use gps_manager::{gps_dma, GpsManager};
// use panic_halt as _;
use atsamd_hal::gpio::{Alternate, Output, PushPull, D, PA04, PA05, PA06, PA07};
use atsamd_hal::sercom::{
    spi,
    spi::{Config, Duplex, Pads, Spi},
    IoSet1, Sercom0,
};
use communication::Capacities;
use communication::{CanCommandManager, RadioManager};
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use data_manager::DataManager;
use defmt::info;
use defmt_rtt as _; // global logger
use fugit::ExtU64;
use fugit::RateExtU32;
use hal::clock::v2::pclk::Pclk;
use hal::clock::v2::Source;
use hal::gpio::Pins;
use hal::gpio::{Pin, PushPullOutput, C, PA02, PA03, PA08, PA09};
use hal::prelude::*;
use hal::sercom::uart;
use hal::sercom::IoSet3;
use mcan::messageram::SharedMemory;
use messages::command::RadioRate;
use messages::state::State;
use messages::*;
use panic_probe as _;
use systick_monotonic::*;
use types::*;
use ublox::{
    CfgPrtUartBuilder, DataBits, InProtoMask, OutProtoMask, Parity, StopBits, UartMode, UartPortId,
    UbxPacketRequest,
};

#[inline(never)]
#[defmt::panic_handler]
fn panic() -> ! {
    atsamd_hal::pac::SCB::sys_reset()
}

/// Hardfault handler.
///
/// Terminates the application and makes a semihosting-capable debug tool exit
/// with an error. This seems better than the default, which is to spin in a
/// loop.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    loop {}
}

#[global_allocator]
static HEAP: embedded_alloc::Heap = embedded_alloc::Heap::empty();

static RTC: Mutex<RefCell<Option<hal::rtc::Rtc<Count32Mode>>>> = Mutex::new(RefCell::new(None));

static mut MCLK: Mutex<RefCell<Option<hal::pac::MCLK>>> = Mutex::new(RefCell::new(None));

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {

    use core::borrow::BorrowMut;

    use atsamd_hal::{clock::v2::pclk, sercom::dma};
    use command::{Command, CommandData};

    use sender::Sender;

    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        data_manager: DataManager,
        radio_manager: RadioManager,
        gps_manager: GpsManager,
        can0: communication::CanDevice0,
        can_command_manager: CanCommandManager,
    }

    #[local]
    struct Local {
        led_green: Pin<PA03, PushPullOutput>,
        led_red: Pin<PA02, PushPullOutput>,
        // gps_uart: GpsUart,
        // sd_manager: SdManager<
        //     Spi<
        //         Config<
        //             Pads<
        //                 hal::pac::SERCOM0,
        //                 IoSet3,
        //                 Pin<PA07, Alternate<D>>,
        //                 Pin<PA04, Alternate<D>>,
        //                 Pin<PA05, Alternate<D>>,
        //             >,
        //         >,
        //         Duplex,
        //     >,
        //     Pin<PA06, Output<PushPull>>,
        // >,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init(local = [
        #[link_section = ".can"]
        can_memory: SharedMemory<Capacities> = SharedMemory::new(),
        #[link_section = ".can_command"]
        can_command_memory: SharedMemory<Capacities> = SharedMemory::new()
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 1024;
            static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
        }
        let mut peripherals = cx.device;
        let core = cx.core;
        let pins = Pins::new(peripherals.PORT);

        let mut dmac = dmac::DmaController::init(peripherals.DMAC, &mut peripherals.PM);
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

        let (pclk_can_command, gclk0) = Pclk::enable(tokens.pclks.can1, gclk0);
        let (can_command_manager, gclk0) = CanCommandManager::new(
            pins.pb15.into_mode(),
            pins.pb14.into_mode(),
            pclk_can_command,
            clocks.ahbs.can1,
            peripherals.CAN1,
            gclk0,
            cx.local.can_command_memory,
            false,
        );

        // setup external osc
        // let xosc0 = atsamd_hal::clock::v2::xosc::Xosc::from_crystal(
        //     tokens.xosc0,
        //     pins.pa14,
        //     pins.pa15,
        //     48_000_000.Hz(),
        // ).current(CrystalCurrent::Medium)
        // .loop_control(true)
        // .low_buf_gain(true)
        // .start_up_delay(StartUpDelay::Delay488us).enable();
        // while !xosc0.is_ready() {
        //     info!("Waiting for XOSC0 to stabilize");
        // }

        // let (mut gclk0, dfll) =
        // hal::clock::v2::gclk::Gclk::from_source(tokens.gclks.gclk2, xosc0);
        // let gclk2 = gclk2.div(gclk::GclkDiv8::Div(1)).enable();
        let (pclk_sercom2, gclk0) = Pclk::enable(tokens.pclks.sercom2, gclk0);
        /* Radio config */

        let rx: Pin<PA08, Alternate<_>> = pins.pa08.into_alternate();
        let tx: Pin<PA09, Alternate<_>> = pins.pa09.into_alternate();
        // let pads = uart::Pads::<hal::sercom::Sercom2, _>::default()
        //     .rx(rx)
        //     .tx(tx);
        // let uart =
        //     GroundStationUartConfig::new(&mclk, peripherals.SERCOM2, pads, pclk_sercom2.freq())
        //         .baud(
        //             RateExtU32::Hz(57600),
        //             uart::BaudMode::Fractional(uart::Oversampling::Bits16),
        //         )
        //         .enable();

        // let radio = RadioDevice::new(uart);

        unsafe {
            MCLK = Mutex::new(RefCell::new(Some(mclk)));
        }
        // let (pclk_gps, gclk0) = Pclk::enable(tokens.pclks.sercom2, gclk0);
        let gps_rx: Pin<atsamd_hal::gpio::PA13, Alternate<C>> = pins.pa13.into_alternate();
        let gps_tx: Pin<atsamd_hal::gpio::PA12, Alternate<C>> = pins.pa12.into_alternate();
        let gps_pads = hal::sercom::uart::Pads::<hal::sercom::Sercom2, IoSet1>::default()
            .rx(gps_rx)
            .tx(gps_tx);

        let mut gps_uart_config = cortex_m::interrupt::free(|cs| {
            let mut x = unsafe { MCLK.borrow(cs).borrow_mut() };
            let mclk = x.as_mut().unwrap();
            GpsUartConfig::new(&mclk, peripherals.SERCOM2, gps_pads, pclk_sercom2.freq()).baud(
                RateExtU32::Hz(9600),
                uart::BaudMode::Fractional(uart::Oversampling::Bits16),
            )
        });
        gps_uart_config = gps_uart_config.immediate_overflow_notification(false);

        let mut gps_uart = gps_uart_config.enable();
        let mut dmaCh0 = dmaChannels.0.init(dmac::PriorityLevel::LVL3);
        let gps_manager = cortex_m::interrupt::free(|cs| {
            let mut x = unsafe { MCLK.borrow(cs).borrow_mut() };
            let mclk = x.as_mut().unwrap();
            GpsManager::new(
                gps_uart,
                None, // pads are already consumed by the uart.
                None,
                pins.pb07.into_push_pull_output(),
                pins.pb09.into_push_pull_output(),
                dmaCh0,
                pclk_sercom2.freq(),
            )
        });
        let message = Message::new(
            cortex_m::interrupt::free(|cs| {
                let mut rc = RTC.borrow(cs).borrow_mut();
                let rtc = rc.as_mut().unwrap();
                rtc.count32()
            }),
            COM_ID,
            State::new(messages::state::StateData::Initializing),
        );
    
        let radio_manager = cortex_m::interrupt::free(|cs| {
            info!("Setting up radio");
            let mut x = unsafe { MCLK.borrow(cs).borrow_mut() };
            let mclk = x.as_mut().unwrap();
            RadioManager::new(rx, tx, pclk_sercom2.freq())
        });
        let mut gps_tx = gps_manager.gps_uart_tx.take().unwrap();
        let mut gps_uart = atsamd_hal::sercom::uart::Uart::join(gps_rx, gps_tx);
        let mut config = gps_uart.disable();
        let (mut sercom, mut pads) = config.free();
        let (rx, tx, _, _) = pads.free();
        let radio_uart = cortex_m::interrupt::free(|cs| {
            let mut x = unsafe { crate::MCLK.borrow(cs).borrow_mut() };
            let mclk = x.as_mut().unwrap();
            let pads = atsamd_hal::sercom::uart::Pads::<atsamd_hal::sercom::Sercom2, atsamd_hal::sercom::IoSet3>::default()
            .rx(radio_manager.rx.take().unwrap())
            .tx(radio_manager.tx.take().unwrap());
            GroundStationUartConfig::new(
                &mclk,
                sercom,
                pads,
                radio_manager.uart_clk_freq,
            )
            .baud(
                RateExtU32::Hz(57600),
                uart::BaudMode::Fractional(uart::Oversampling::Bits16),
            )
            .enable()
        });
        radio_manager.radio = Some(radio_uart);

        loop {

            let mut buf = [0; 255];
            let data = postcard::to_slice(&m, &mut buf).unwrap();
            radio_manager.send_message(data);
        }

        // /* SD config */
        // let (mut gclk1, dfll) =
        //     hal::clock::v2::gclk::Gclk::from_source(tokens.gclks.gclk1, clocks.dfll);
        // let gclk1 = gclk1.div(gclk::GclkDiv16::Div(3)).enable(); // 48 / 3 = 16 MHzs
        // let (pclk_sd, gclk0) = Pclk::enable(tokens.pclks.sercom0, gclk0);

        // let pads_spi = spi::Pads::<Sercom0, IoSet3>::default()
        //     .sclk(pins.pa05)
        //     .data_in(pins.pa07)
        //     .data_out(pins.pa04);
        // let sdmmc_spi = spi::Config::new(&mclk, peripherals.SERCOM0, pads_spi, pclk_sd.freq())
        //     .length::<spi::lengths::U1>()
        //     .bit_order(spi::BitOrder::MsbFirst)
        //     .spi_mode(spi::MODE_0)
        //     .enable();
        // let sd_manager = SdManager::new(sdmmc_spi, pins.pa06.into_push_pull_output());

        // No need to reclock the peripheral sercom since it's shared with the radio.

        // gps_uart.enable_interrupts(hal::sercom::uart::Flags::RXC);

        /* Status LED */
        // info!("Setting up LED");
        // let led = pins.pa02.into_push_pull_output();
        // let mut gps_enable = pins.pb09.into_push_pull_output();
        // //gps_enable.set_high().ok();
        // gps_enable.set_low().ok();
        /* Spawn tasks */
        // sensor_send::spawn().ok();
        // state_send::spawn().ok();
        let rtc = cortex_m::interrupt::free(|cs| {
            let mut x = unsafe { MCLK.borrow(cs).borrow_mut() };
            let mut mclk = x.as_mut().unwrap();
            hal::rtc::Rtc::clock_mode(
                peripherals.RTC,
                atsamd_hal::fugit::RateExtU32::Hz(1024),
                &mut mclk,
            )
        });

        let mut rtc = rtc.into_count32_mode(); // hal bug this must be done
        rtc.set_count32(0);
        cortex_m::interrupt::free(|cs| {
            RTC.borrow(cs).replace(Some(rtc));
        });
        let mut led_green = pins.pa03.into_push_pull_output();
        let mut led_red = pins.pa02.into_push_pull_output();
        led_green.set_high();
        led_red.set_high();
        blink::spawn().ok();
        sensor_send::spawn().ok();
        // generate_random_command::spawn().ok();
        let mono = Systick::new(core.SYST, gclk0.freq().to_Hz());

        // info!("Init done");
        (
            Shared {
                em: ErrorManager::new(),
                data_manager: DataManager::new(),
                radio_manager: radio_manager,
                gps_manager: gps_manager,
                can0,
                can_command_manager,
            },
            Local {
                led_green,
                led_red,
                // sd_manager,
            },
            init::Monotonics(mono),
        )
    }

    /// Idle task for when no other tasks are running.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    // #[task(binds = SERCOM2_2, local = [gps_uart], shared = [&em])]
    // fn gps_rx(mut cx: gps_rx::Context) {
    //     cx.shared.em.run(|| {
    //         let byte = cx.local.gps_uart.read().unwrap();
    //         info!("GPS: {}", byte);
    //         Ok(())
    //     });
    // }

    /// Handles the CAN1 interrupt.
    #[task(priority = 3, binds = CAN1, shared = [can_command_manager, data_manager])]
    fn can_command(mut cx: can_command::Context) {
        info!("CAN1 interrupt");
        cx.shared.can_command_manager.lock(|can| {
            cx.shared
                .data_manager
                .lock(|data_manager| can.process_data(data_manager));
        });
    }

    /// Handles the CAN0 interrupt.
    #[task(priority = 3, binds = CAN0, shared = [can0, data_manager])]
    fn can0(mut cx: can0::Context) {
        info!("CAN0 interrupt");
        cx.shared.can0.lock(|can| {
            cx.shared
                .data_manager
                .lock(|data_manager| can.process_data(data_manager));
        });
    }
    /**
     * Sends a mgps_txessage over CAN.
     */
    #[task(capacity = 10, shared = [can0, &em])]
    fn send_internal(mut cx: send_internal::Context, m: Message) {
        // info!("{}", m.clone());
        cx.shared.em.run(|| {
            cx.shared.can0.lock(|can| can.send_message(m))?;
            Ok(())
        });
    }

    /**
     * Sends a message over CAN.
     */
    #[task(priority = 3, capacity = 5, shared = [can_command_manager, &em])]
    fn send_command(mut cx: send_command::Context, m: Message) {
        info!("{}", m.clone());
        cx.shared.em.run(|| {
            cx.shared
                .can_command_manager
                .lock(|can| can.send_message(m))?;
            Ok(())
        });
    }

    /// Receives a log message from the custom logger so that it can be sent over the radio.
    pub fn queue_gs_message(d: impl Into<Data>) {
        let message = Message::new(
            cortex_m::interrupt::free(|cs| {
                let mut rc = RTC.borrow(cs).borrow_mut();
                let rtc = rc.as_mut().unwrap();
                rtc.count32()
            }),
            COM_ID,
            d.into(),
        );
        info!("Sending message {}", message);
        // send_gs::spawn(message).ok();
    }

    // /**
    //  * Sends a message to the radio over UART.
    //  */
    // #[task(capacity = 10, shared = [&em, radio_manager])]
    // fn send_gs(mut cx: send_gs::Context, m: Message) {
    //     cx.shared.radio_manager.lock(|radio_manager| {
    //         cx.shared.em.run(|| {
                // let mut buf = [0; 255];
                // let data = postcard::to_slice(&m, &mut buf)?;
    //             radio_manager.send_message(data)?;
    //             Ok(())
    //         })
    //     });
    // }

    // #[task(capacity = 10, local = [sd_manager], shared = [&em])]
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
    // }

    /**
     * Sends information about the sensors.
     */
    #[task(shared = [data_manager, radio_manager, gps_manager, &em])]
    fn sensor_send(mut cx: sensor_send::Context) {
        let (sensors, logging_rate) = cx
            .shared
            .data_manager
            .lock(|data_manager| (data_manager.take_sensors(), data_manager.get_logging_rate()));
        cx.shared.radio_manager.lock(|mut radio_manager| {
            for msg in sensors {
                match msg {
                    Some(x) => {
                        // spawn!(send_gs, x.clone())?;
                        cx.shared.em.run(|| {
                            let mut buf = [0; 255];
                            let data = postcard::to_slice(&x, &mut buf)?;
                            radio_manager.send_message(data)?;

                            Ok(())
                        });

                        //                     spawn!(sd_dump, x)?;
                    }
                    None => {
                        continue;
                    }
                }
            }
            info!("before take");
            if let Some(radio) = radio_manager.radio.take() {
                info!("after_take");
                let mut config = radio_manager.radio.take().unwrap().disable();

                let (mut sercom, mut pads) = config.free();
                let (rx, tx, _, _) = pads.free();
                radio_manager.rx = Some(rx);
                radio_manager.tx = Some(tx);

                cx.shared.gps_manager.lock(|mut gps_manager| {
                    let mut gps_uart = cortex_m::interrupt::free(|cs| {
                        let mut x = unsafe { crate::MCLK.borrow(cs).borrow_mut() };
                        let mclk = x.as_mut().unwrap();
                        let pads = atsamd_hal::sercom::uart::Pads::<
                            atsamd_hal::sercom::Sercom2,
                            atsamd_hal::sercom::IoSet1,
                        >::default()
                        .rx(gps_manager.rx.take().unwrap())
                        .tx(gps_manager.tx.take().unwrap());
                        let mut gps_uart_config =
                            GpsUartConfig::new(&mclk, sercom, pads, gps_manager.uart_clk_freq)
                                .baud(
                                    RateExtU32::Hz(9600),
                                    uart::BaudMode::Fractional(uart::Oversampling::Bits16),
                                );

                        gps_uart_config = gps_uart_config.immediate_overflow_notification(false);
                        gps_uart_config.enable()
                    });

                    let request =
                        UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
                    for byte in request {
                        nb::block!(gps_uart.write(byte)).unwrap();
                    }
                    cortex_m::asm::delay(300_000);
                    gps_manager.gps_uart = Some(gps_uart);
                });
            }
        });

        match logging_rate {
            RadioRate::Fast => {
                spawn_after!(sensor_send, ExtU64::millis(100)).ok();
            }
            RadioRate::Slow => {
                spawn_after!(sensor_send, ExtU64::millis(250)).ok();
            }
        }
    }

    // #[task(shared = [&em])]
    // fn generate_random_messages(cx: generate_random_messages::Context) {
    //     cx.shared.em.run(|| {
            // let message = Message::new(
            //     cortex_m::interrupt::free(|cs| {
            //         let mut rc = RTC.borrow(cs).borrow_mut();
            //         let rtc = rc.as_mut().unwrap();
            //         rtc.count32()
            //     }),
            //     COM_ID,
            //     State::new(messages::state::StateData::Initializing),
            // );
    //         info!("Sending message {}", message.clone());
    //         // spawn!(send_gs, message.clone())?;
    //         spawn!(send_gs, message)?;
    //         Ok(())
    //     });
    //     spawn_after!(generate_random_messages, ExtU64::millis(200)).ok();
    // }

    // #[task(shared = [&em])]
    // fn generate_random_command(cx: generate_random_command::Context) {
    //     cx.shared.em.run(|| {
    //         let message = Message::new(
    //             cortex_m::interrupt::free(|cs| {
    //                 let mut rc = RTC.borrow(cs).borrow_mut();
    //                 let rtc = rc.as_mut().unwrap();
    //                 rtc.count32()
    //             }),
    //             COM_ID,
    //             Command::new(CommandData::PowerDown(command::PowerDown {
    //                 board: Sender::BeaconBoard,
    //             })),
    //         );
    //         spawn!(send_command, message)?;
    //         Ok(())
    //     });
    //     spawn!(generate_random_command).ok();
    // }

    #[task(shared = [data_manager, &em, radio_manager])]
    fn state_send(mut cx: state_send::Context) {
        let state_data = cx
            .shared
            .data_manager
            .lock(|data_manager| data_manager.state.clone());
        if let Some(x) = state_data {
            let message = Message::new(
                cortex_m::interrupt::free(|cs| {
                    let mut rc = RTC.borrow(cs).borrow_mut();
                    let rtc = rc.as_mut().unwrap();
                    rtc.count32()
                }),
                COM_ID,
                State::new(x),
            );
            cx.shared.radio_manager.lock(|mut radio_manager| {
                cx.shared.em.run(|| {
                    let mut buf = [0; 255];
                    let data = postcard::to_slice(&message, &mut buf)?;
                    radio_manager.send_message(data)?;
                    // *radio_manager = Some(radio);
                    Ok(())
                });
            });
            // spawn!(send_gs, message)?;
        } // if there is none we still return since we simply don't have data yet.
        spawn_after!(state_send, ExtU64::secs(5)).ok();
    }

    // /// Handles the radio interrupt.
    // /// This only needs to be called when the radio has data to read, this is why an interrupt handler is used above polling which would waste resources.
    // /// We use a critical section to ensure that we are not interrupted while servicing the mavlink message.
    // #[task(priority = 3, binds = SERCOM2_2, shared = [&em, radio_manager])]
    // fn radio_rx(mut cx: radio_rx::Context) {
    //     cx.shared.radio_manager.lock(|radio_manager| {
    //         cx.shared.em.run(|| {
    //             cortex_m::interrupt::free(|cs| {
    //                 let mut radio = radio_manager.take();
    //                 let m = radio.receive_message()?;
    //                 radio_manager = Some(radio);
    //                 info!("Received message {}", m.clone());
    //                 spawn!(send_command, m)
    //             })?;
    //             Ok(())
    //         });
    //     });
    // }

    /**
     * Simple blink task to test the system.
     * Acts as a heartbeat for the system.
     */
    #[task(local = [led_green, led_red], shared = [&em])]
    fn blink(cx: blink::Context) {
        cx.shared.em.run(|| {
            if cx.shared.em.has_error() {
                cx.local.led_red.toggle()?;
                spawn_after!(blink, ExtU64::millis(200))?;
            } else {
                cx.local.led_green.toggle()?;
                spawn_after!(blink, ExtU64::secs(1))?;
            }
            Ok(())
        });
    }

    extern "Rust" {
        #[task(priority = 3, binds = DMAC_0, shared = [&em, gps_manager, data_manager, radio_manager])]
        fn gps_dma(context: gps_dma::Context);
    }
}
