#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod types;

use atsamd_hal as hal;
use atsamd_hal::dmac;
use atsamd_hal::dmac::DmaController;
use atsamd_hal::dmac::Transfer;
use atsamd_hal::rtc::Count32Mode;
use common_arm::*;
use common_arm_atsame::mcan;
use communication::CanCommandManager;
use communication::Capacities;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use data_manager::DataManager;
use defmt::info;
use defmt_rtt as _; // global logger
use fugit::ExtU64;
use types::GpsUartTx;
use fugit::RateExtU32;
use hal::clock::v2::pclk::Pclk;
use hal::clock::v2::Source;
use hal::gpio::Pins;
use hal::gpio::{Alternate, Pin, PushPullOutput, C, PA02, PA03, PA08, PA09};
use hal::prelude::*;
use hal::sercom::uart;
use hal::sercom::IoSet1;
use mavlink::embedded::Read;
use mcan::messageram::SharedMemory;
use messages::*;
use panic_probe as _;
use systick_monotonic::*;
use types::GPSTransfer;
use types::GPSBUFFER;
use types::*;
use ublox::{
    CfgPrtUartBuilder, DataBits, InProtoMask, OutProtoMask, Parity, StopBits, UartMode, UartPortId,
};

pub static mut BUF_DST: GPSBUFFER = &mut [0; 256];

#[inline(never)]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

#[global_allocator]
static HEAP: embedded_alloc::Heap = embedded_alloc::Heap::empty();

/// Hardfault handler.
///
/// Terminates the application and makes a semihosting-capable debug tool exit
/// with an error. This seems better than the default, which is to spin in a
/// loop.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    loop {}
}

static RTC: Mutex<RefCell<Option<hal::rtc::Rtc<Count32Mode>>>> = Mutex::new(RefCell::new(None));

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {

    use atsamd_hal::sercom::Sercom;
    use cortex_m::asm;
    use ublox::{
        CfgMsgAllPortsBuilder, CfgMsgSinglePortBuilder, NavPosLlh, NavPvt, PacketRef,
        UbxPacketRequest,
    };

    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        data_manager: DataManager,
        can0: communication::CanDevice0,
        can_command_manager: CanCommandManager,
    }

    #[local]
    struct Local {
        led_green: Pin<PA03, PushPullOutput>,
        led_red: Pin<PA02, PushPullOutput>,
        gps_tx: GpsUartTx,
        gps_dma_transfer: Option<GPSTransfer>,
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

        // /* SD config */
        // let (mut gclk1, dfll) =
        //     hal::clock::v2::gclk::Gclk::from_source(tokens.gclks.gclk1, clocks.dfll);
        // let gclk1 = gclk1.div(gclk::GclkDiv16::Div(3)).enable(); // 48 / 3 = 16 MHzs
        // let (pclk_sd, gclk1) = Pclk::enable(tokens.pclks.sercom0, gclk1);

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
        // let (pclk_radio, gclk0) = Pclk::enable(tokens.pclks.sercom2, gclk0);
        // /* Radio config */
        // let rx: Pin<PA08, Alternate<C>> = pins.pa08.into_alternate();
        // let tx: Pin<PA09, Alternate<C>> = pins.pa09.into_alternate();
        // let pads = uart::Pads::<hal::sercom::Sercom2, _>::default()
        //     .rx(rx)
        //     .tx(tx);
        // let mut uart =
        //     GroundStationUartConfig::new(&mclk, peripherals.SERCOM2, pads, pclk_radio.freq())
        //         .baud(
        //             RateExtU32::Hz(57600),
        //             uart::BaudMode::Fractional(uart::Oversampling::Bits16),
        //         )
        //         .enable();

        // loop {
        //     nb::block!(uart.write(0x55)).unwrap();
        // }

        let (pclk_gps, gclk0) = Pclk::enable(tokens.pclks.sercom2, gclk0);
        //info!("here");
        // let mut rx = pins.pa13.into_push_pull_output();
        // loop {
        //     rx.set_high().ok();
        //     cortex_m::asm::delay(300_000);
        //     rx.set_low().ok();
        //     cortex_m::asm::delay(300_000);
        // }

        let pads = hal::sercom::uart::Pads::<hal::sercom::Sercom2, IoSet1>::default()
            .rx(pins.pa13)
            .tx(pins.pa12);

        let mut gps_uart_config =
            GpsUartConfig::new(&mclk, peripherals.SERCOM2, pads, pclk_gps.freq()).baud(
                RateExtU32::Hz(9600),
                uart::BaudMode::Fractional(uart::Oversampling::Bits16),
            );

        gps_uart_config = gps_uart_config.immediate_overflow_notification(false);

        // loop {
        //     let (x,y) = gps_uart_config.get_baud();
        //     info!("Baud: {}", x);
        // }

        let mut gps_uart = gps_uart_config.enable();
        let (mut gps_rx, mut gps_tx) = gps_uart.split();
        // gps_uart.enable_interrupts(hal::sercom::uart::Flags::RXC);

        /* Status LED */
        // info!("Setting up LED");
        // let led = pins.pa02.into_push_pull_output();
        let mut gps_enable = pins.pb09.into_push_pull_output();
        let mut gps_reset = pins.pb07.into_push_pull_output();
        gps_reset.set_low().ok();
        cortex_m::asm::delay(300_000);
        gps_reset.set_high().ok();

        // gps_reset.set_low().ok();
        // gps_enable.set_high().ok();
        gps_enable.set_low().ok();
        let packet: [u8; 28] = CfgPrtUartBuilder {
            portid: UartPortId::Uart1,
            reserved0: 0,
            tx_ready: 0,
            mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
            baud_rate: 9600,
            in_proto_mask: InProtoMask::all(),
            out_proto_mask: OutProtoMask::UBLOX,
            flags: 0,
            reserved5: 0,
        }
        .into_packet_bytes();

        for byte in packet {
            // info!("Byte: {}", byte);
            nb::block!(gps_tx.write(byte)).unwrap();
        }

        // let packet_two = CfgMsgAllPortsBuilder::set_rate_for::<NavPvt>([1, 0, 0, 0, 0, 0]).into_packet_bytes();
        // for byte in packet_two {
        //     nb::block!(gps_uart.write(byte)).unwrap();
        // }
        let mut dmaCh0 = dmaChannels.0.init(dmac::PriorityLevel::LVL3);
        /* DMAC config */
        dmaCh0
            .as_mut()
            .enable_interrupts(dmac::InterruptFlags::new().with_tcmpl(true));
        let mut gps_dma_transfer: GPSTransfer =
            Transfer::new(dmaCh0, gps_rx, unsafe { &mut *BUF_DST }, false)
                .expect("DMA err")
                .begin(
                    atsamd_hal::sercom::Sercom2::DMA_RX_TRIGGER,
                    dmac::TriggerAction::BURST,
                );

        loop {
            if gps_dma_transfer.complete() {
                let (chan0, source, buf) = gps_dma_transfer.stop();
                gps_dma_transfer =
                    dmac::Transfer::new(chan0, source, unsafe { &mut *BUF_DST }, false)
                        .unwrap()
                        .begin(
                            atsamd_hal::sercom::Sercom2::DMA_RX_TRIGGER,
                            dmac::TriggerAction::BURST,
                        );
                let buf_clone = buf.clone();

                unsafe { BUF_DST.copy_from_slice(&[0; 256]) };
                gps_dma_transfer.block_transfer_interrupt();
                let request =
                    UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
                for byte in request {
                    nb::block!(gps_tx.write(byte)).unwrap();
                }
                cortex_m::asm::delay(300_000);
                let mut buf: [u8; 256] = [0; 256];
                let mut bytes: [u8; 256] = [0; 256];
                // for i in 0..buf.len() {
                //     let item = nb::block!(gps_uart.read()).unwrap();
                //     // info!("Byte: {}", item);
                //     bytes[i] = item;
                // }
                let buf = ublox::FixedLinearBuffer::new(&mut buf[..]);
                let mut parser = ublox::Parser::new(buf);
                let mut msgs = parser.consume(&buf_clone);
                while let Some(msg) = msgs.next() {
                    match msg {
                        Ok(msg) => match msg {
                            ublox::PacketRef::NavPosLlh(x) => {
                                let message_data = messages::sensor::NavPosLlh {
                                    height_msl: x.height_msl(),
                                    longitude: x.lon_degrees(),
                                    latitude: x.lat_degrees(),
                                };
                                // info!("GPS latitude: {:?}, longitude {:?}", x.lat_degrees(), x.lon_degrees());
                            }
                            ublox::PacketRef::NavStatus(x) => {
                                info!("GPS fix stat: {:?}", x.fix_stat_raw());
                            }
                            ublox::PacketRef::NavDop(x) => {
                                info!("GPS geometric drop: {:?}", x.geometric_dop());
                            }
                            ublox::PacketRef::NavSat(x) => {
                                info!("GPS num sats used: {:?}", x.num_svs());
                            }
                            ublox::PacketRef::NavVelNed(x) => {
                                info!("GPS velocity north: {:?}", x.vel_north());
                            }
                            ublox::PacketRef::NavPvt(x) => {
                                info!("GPS nun sats PVT: {:?}", x.num_satellites());
                            }
                            _ => {
                                info!("GPS Message not handled.");
                            }
                        },
                        Err(e) => {
                            info!("GPS parse Error");
                        }
                    }
                }
            }
        }
        /* Spawn tasks */
        // sensor_send::spawn().ok();
        // state_send::spawn().ok();
        let rtc = hal::rtc::Rtc::clock_mode(
            peripherals.RTC,
            atsamd_hal::fugit::RateExtU32::Hz(1024),
            &mut mclk,
        );
        let mut rtc = rtc.into_count32_mode(); // hal bug this must be done
        rtc.set_count32(0);
        cortex_m::interrupt::free(|cs| {
            RTC.borrow(cs).replace(Some(rtc));
        });
        let mut led_green = pins.pa03.into_push_pull_output();
        let mut led_red = pins.pa02.into_push_pull_output();
        led_green.set_low();
        led_red.set_low();
        blink::spawn().ok();
        let message = Message::new(
            0, // technically true time is not known yet.
            COM_ID,
            messages::command::Command {
                data: messages::command::CommandData::Online(messages::command::Online {
                    online: true,
                }),
            },
        );
        send_command::spawn(message).ok();
        let mono = Systick::new(core.SYST, gclk0.freq().to_Hz());

        (
            Shared {
                em: ErrorManager::new(),
                data_manager: DataManager::new(),
                can0,
                can_command_manager,
            },
            Local {
                led_green,
                led_red,
                gps_dma_transfer: Some(gps_dma_transfer),
                gps_tx, 
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

    #[task(priority = 3, binds = DMAC_0, local = [gps_dma_transfer, gps_tx], shared = [&em, data_manager])]
    fn gps_dma(mut cx: gps_dma::Context) {
        let mut gps_dma_transfer = cx.local.gps_dma_transfer.take().unwrap();
        let mut gps_tx = cx.local.gps_tx;
        if gps_dma_transfer.complete() {
            let (chan0, source, buf) = gps_dma_transfer.stop();
            gps_dma_transfer = dmac::Transfer::new(chan0, source, unsafe { &mut *BUF_DST }, false)
                .unwrap()
                .begin(
                    atsamd_hal::sercom::Sercom2::DMA_RX_TRIGGER,
                    dmac::TriggerAction::BURST,
                );
            let buf_clone = buf.clone();

            unsafe { BUF_DST.copy_from_slice(&[0; 256]) };
            gps_dma_transfer.block_transfer_interrupt();
            let request = UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
            for byte in request {
                nb::block!(gps_tx.write(byte)).unwrap();
            }
            cortex_m::asm::delay(300_000);
            let mut buf: [u8; 256] = [0; 256];
            let mut bytes: [u8; 256] = [0; 256];
            let buf = ublox::FixedLinearBuffer::new(&mut buf[..]);
            let mut parser = ublox::Parser::new(buf);
            let mut msgs = parser.consume(&buf_clone);
            while let Some(msg) = msgs.next() {
                match msg {
                    Ok(msg) => match msg {
                        ublox::PacketRef::NavPosLlh(x) => {
                            let message_data = messages::sensor::NavPosLlh {
                                height_msl: x.height_msl(),
                                longitude: x.lon_degrees(),
                                latitude: x.lat_degrees(),
                            };
                            // info!("GPS latitude: {:?}, longitude {:?}", x.lat_degrees(), x.lon_degrees());
                        }
                        ublox::PacketRef::NavStatus(x) => {
                            info!("GPS fix stat: {:?}", x.fix_stat_raw());
                        }
                        ublox::PacketRef::NavDop(x) => {
                            info!("GPS geometric drop: {:?}", x.geometric_dop());
                        }
                        ublox::PacketRef::NavSat(x) => {
                            info!("GPS num sats used: {:?}", x.num_svs());
                        }
                        ublox::PacketRef::NavVelNed(x) => {
                            info!("GPS velocity north: {:?}", x.vel_north());
                        }
                        ublox::PacketRef::NavPvt(x) => {
                            info!("GPS nun sats PVT: {:?}", x.num_satellites());
                        }
                        _ => {
                            info!("GPS Message not handled.");
                        }
                    },
                    Err(e) => {
                        info!("GPS parse Error");
                    }
                }
            }
        }
    }

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
        // info!("CAN0 interrupt");
        cx.shared.can0.lock(|can| {
            cx.shared
                .data_manager
                .lock(|data_manager| can.process_data(data_manager));
        });
    }
    /**
     * Sends a message over CAN.
     */
    #[task(capacity = 10, shared = [can0, &em])]
    fn send_internal(mut cx: send_internal::Context, m: Message) {
        info!("{}", m.clone());
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

        send_internal::spawn(message).ok();
    }

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

    // /**
    //  * Sends information about the sensors.
    //  */
    // #[task(shared = [data_manager, &em])]
    // fn sensor_send(mut cx: sensor_send::Context) {
    //     let (sensors, logging_rate) = cx
    //         .shared
    //         .data_manager
    //         .lock(|data_manager| (data_manager.take_sensors(), data_manager.get_logging_rate()));

    //     cx.shared.em.run(|| {
    //         for msg in sensors {
    //             match msg {
    //                 Some(x) => {
    //                     spawn!(send_gs, x.clone())?;
    //                     //                     spawn!(sd_dump, x)?;
    //                 }
    //                 None => {
    //                     continue;
    //                 }
    //             }
    //         }
    //         Ok(())
    //     });
    //     match logging_rate {
    //         RadioRate::Fast => {
    //             spawn_after!(sensor_send, ExtU64::millis(100)).ok();
    //         }
    //         RadioRate::Slow => {
    //             spawn_after!(sensor_send, ExtU64::millis(250)).ok();
    //         }
    //     }
    // }

    // /// Handles the radio interrupt.
    // /// This only needs to be called when the radio has data to read, this is why an interrupt handler is used above polling which would waste resources.
    // /// We use a critical section to ensure that we are not interrupted while servicing the mavlink message.
    // #[task(priority = 3, binds = SERCOM2_2, shared = [&em, radio_manager])]
    // fn radio_rx(mut cx: radio_rx::Context) {
    //     cx.shared.radio_manager.lock(|radio_manager| {
    //         cx.shared.em.run(|| {
    //            cortex_m::interrupt::free(|cs| {
    //                 let m = radio_manager.receive_message()?;
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


}
