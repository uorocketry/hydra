#![no_std]
#![no_main]
use atsamd_hal as hal;
use atsamd_hal::gpio::*;
use atsamd_hal::pac;
use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::uart::{Duplex, Uart};
use atsamd_hal::sercom::{uart, IoSet1, IoSet6};
use common_arm::*;
use defmt::info;
use embedded_sdmmc::File;
use fugit::RateExtU32;
use hal::dmac;
use hal::dmac::BufferPair;
use hal::gpio::Pins;
use hal::gpio::PA14;
use hal::gpio::{Pin, PushPullOutput};
use hal::prelude::*;
use hal::sercom::Sercom0;
use hal::sercom::Sercom5;
use hal::time::Hertz;
use hal::{dmac::Transfer, sercom::Sercom};
use heapless::Vec;
use mcan::embedded_can as ecan;
use mcan::generic_array::typenum::consts::*;
use mcan::interrupt::{Interrupt, InterruptLine, OwnedInterruptSet};
use mcan::message::rx;
use mcan::message::tx;
use mcan::messageram::SharedMemory;
use mcan::prelude::*;
use mcan::rx_fifo::Fifo0;
use mcan::rx_fifo::Fifo1;
use mcan::rx_fifo::RxFifo;
use mcan::{
    config::{BitTiming, Mode},
    filter::{Action, ExtFilter, Filter},
};
use messages::mav_message;
use messages::mavlink;
use messages::sender::Sender::MainBoard;
use messages::sensor::{Sbg, Sensor};
use messages::*;
use panic_halt as _;
use sbg_rs::sbg;
use systick_monotonic::*;
const SBG_BUFFER_SIZE: usize = 1024;
static mut BUF_DST: SBGBuffer = &mut [0; SBG_BUFFER_SIZE];
static mut BUF_DST2: SBGBuffer = &mut [0; SBG_BUFFER_SIZE];
type Pads = uart::PadsFromIds<Sercom5, IoSet6, PB00, PB02>;
type PadsSBG = uart::PadsFromIds<Sercom0, IoSet1, PA09, PA08>;
type Config = uart::Config<Pads, EightBit>;
type ConfigSBG = uart::Config<PadsSBG, EightBit>;
type SBGTransfer = dmac::Transfer<
    dmac::Channel<dmac::Ch0, dmac::Busy>,
    BufferPair<Uart<ConfigSBG, uart::RxDuplex>, SBGBuffer>,
>;
type SBGBuffer = &'static mut [u8; SBG_BUFFER_SIZE];

pub struct Capacities;

impl mcan::messageram::Capacities for Capacities {
    type StandardFilters = U1;
    type ExtendedFilters = U1;
    type RxBufferMessage = rx::Message<64>;
    type DedicatedRxBuffers = U0;
    type RxFifo0Message = rx::Message<64>;
    type RxFifo0 = U64;
    type RxFifo1Message = rx::Message<64>;
    type RxFifo1 = U64;
    type TxMessage = tx::Message<64>;
    type TxBuffers = U32;
    type DedicatedTxBuffers = U0;
    type TxEventFifo = U32;
}

type RxFifo0 = RxFifo<
    'static,
    Fifo0,
    hal::clock::v2::types::Can0,
    <Capacities as mcan::messageram::Capacities>::RxFifo0Message,
>;

type RxFifo1 = RxFifo<
    'static,
    Fifo1,
    hal::clock::v2::types::Can0,
    <Capacities as mcan::messageram::Capacities>::RxFifo1Message,
>;

type Tx = mcan::tx_buffers::Tx<'static, hal::clock::v2::types::Can0, Capacities>;
type TxEventFifo = mcan::tx_event_fifo::TxEventFifo<'static, hal::clock::v2::types::Can0>;
type Aux = mcan::bus::Aux<
    'static,
    hal::clock::v2::types::Can0,
    hal::can::Dependencies<
        hal::clock::v2::types::Can0,
        hal::clock::v2::gclk::Gclk0Id,
        Pin<PA23, AlternateI>, // Could be an issue. Need to find the right type level enum.
        Pin<PA22, AlternateI>,
        pac::CAN0,
    >,
>;

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        opt_xfer: SBGTransfer,
        buf_select: bool,
        sbg: sbg::SBG,
        sensor_data: Sbg,
    }

    #[local]
    struct Local {
        led: Pin<PA14, PushPullOutput>,
        uart: Uart<Config, Duplex>,
        sd: SdInterface,
        sbg_file: File,
        line_interrupts: OwnedInterruptSet<hal::clock::v2::types::Can0>,
        rx_fifo_0: RxFifo0,
        rx_fifo_1: RxFifo1,
        tx: Tx,
        tx_event_fifo: TxEventFifo,
        aux: Aux,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init(local = [#[link_section = ".can"] can_memory: SharedMemory<Capacities> = SharedMemory::new()])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut peripherals = cx.device;
        let core = cx.core;

        let (_buses, v2_clocks, tokens) = atsamd_hal::clock::v2::clock_system_at_reset(
            peripherals.OSCCTRL,
            peripherals.OSC32KCTRL,
            peripherals.GCLK,
            peripherals.MCLK,
            &mut peripherals.NVMCTRL,
        );

        // /// SAFTEY
        // /// Verify this won't cause issues
        let (_, _, _, mut mclk) = unsafe { v2_clocks.pac.steal() };

        // let mut clocks = hal::clock::GenericClockController::with_external_32kosc(
        //     gclk,
        //     &mut mclk,
        //     &mut osc32kctrl,
        //     &mut oscctrl,
        //     &mut peripherals.NVMCTRL,
        // );

        let pins = Pins::new(peripherals.PORT);
        let led = pins.pa14.into_push_pull_output();

        /* SD config */
        let (pclk_sd, gclk0) =
            atsamd_hal::clock::v2::pclk::Pclk::enable(tokens.pclks.sercom1, v2_clocks.gclk0);
        // clocks.configure_gclk_divider_and_source(
        //     pac::gclk::pchctrl::GEN_A::GCLK5,
        //     1,
        //     pac::gclk::genctrl::SRC_A::DFLL,
        //     false,
        // );
        // let gclk5 = clocks
        //     .get_gclk(pac::gclk::pchctrl::GEN_A::GCLK5)
        //     .expect("Could not get gclk 5.");
        // let spi_clk = clocks
        //     .sercom1_core(&gclk5)
        //     .expect("Could not configure Sercom 1 clock.");
        let sercom = peripherals.SERCOM1;
        let cs = pins.pa18.into_push_pull_output();
        let sck = pins.pa17.into_push_pull_output();
        let miso = pins.pa19.into_push_pull_output();
        let mosi = pins.pa16.into_push_pull_output();
        let mut sd = SdInterface::new(&mclk, sercom, pclk_sd.freq(), cs, sck, miso, mosi);
        let sbg_file = sd.open_file("raw.txt").expect("Could not open file");
        /* End SD config */
        /* Radio config */
        let (pclk_radio, gclk0) =
            atsamd_hal::clock::v2::pclk::Pclk::enable(tokens.pclks.sercom5, gclk0);

        let pads = uart::Pads::<hal::sercom::Sercom5, hal::sercom::IoSet6>::default()
            .rx(pins.pb00)
            .tx(pins.pb02);
        let uart = Config::new(&mclk, peripherals.SERCOM5, pads, pclk_radio.freq())
            .baud(
                57600.hz(),
                uart::BaudMode::Fractional(uart::Oversampling::Bits16),
            )
            .enable();
        /* End Radio config */

        /* CAN config */
        let (pclk_eic, gclk0) = atsamd_hal::clock::v2::pclk::Pclk::enable(tokens.pclks.eic, gclk0);
        let can0_rx = pins.pa23.into_mode();
        let can0_tx = pins.pa22.into_mode();
        // let mut cFLASHan1_standby =
        let (pclk_can0, gclk0) =
            atsamd_hal::clock::v2::pclk::Pclk::enable(tokens.pclks.can0, gclk0);

        let (can_dependencies, gclk0) = hal::can::Dependencies::new(
            gclk0,
            pclk_can0,
            v2_clocks.ahbs.can0,
            can0_rx,
            can0_tx,
            peripherals.CAN0,
        );

        let mut can = mcan::bus::CanConfigurable::new(
            200.kHz().into(),
            can_dependencies,
            cx.local.can_memory,
        )
        .unwrap();

        can.config().mode = Mode::Classic {

        };
        // can.config().mode = Mode::Fd {
        //     allow_bit_rate_switching: true,
        //     data_phase_timing: BitTiming::new(750.kHz().into()),
        // };

        let line_interrupts = can
            .interrupts()
            .enable(
                [
                    Interrupt::RxFifo0NewMessage,
                    Interrupt::RxFifo0Full,
                    Interrupt::RxFifo0MessageLost,
                    Interrupt::RxFifo1NewMessage,
                    Interrupt::RxFifo1Full,
                    Interrupt::RxFifo1MessageLost,
                ]
                .into_iter()
                .collect(),
                InterruptLine::Line0,
            )
            .unwrap();

        can.filters_standard()
            .push(Filter::Classic {
                action: Action::StoreFifo0,
                filter: ecan::StandardId::MAX,
                mask: ecan::StandardId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Standard filter application failed"));

        can.filters_extended()
            .push(ExtFilter::Classic {
                action: Action::StoreFifo1,
                filter: ecan::ExtendedId::MAX,
                mask: ecan::ExtendedId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Extended filter application failed"));

        let can = can.finalize().unwrap();

        let rx_fifo_0 = can.rx_fifo_0;
        let rx_fifo_1 = can.rx_fifo_1;
        let tx = can.tx;
        let tx_event_fifo = can.tx_event_fifo;
        let aux = can.aux;

        /* End CAN config */

        /* SBG config */
        let (pclk_sbg, gclk0) =
            atsamd_hal::clock::v2::pclk::Pclk::enable(tokens.pclks.sercom0, gclk0);

        let padsSBG = uart::Pads::<Sercom0, _>::default()
            .rx(pins.pa09)
            .tx(pins.pa08);
        let uart_sbg = ConfigSBG::new(&mclk, peripherals.SERCOM0, padsSBG, pclk_sbg.freq())
            .baud(
                115200.hz(),
                uart::BaudMode::Fractional(uart::Oversampling::Bits8),
            )
            .enable();

        let (sbg_rx, sbg_tx) = uart_sbg.split();

        /* DMAC config */
        let mut dmac = dmac::DmaController::init(peripherals.DMAC, &mut peripherals.PM);
        let channels = dmac.split();
        let mut chan0 = channels.0.init(dmac::PriorityLevel::LVL3);
        chan0
            .as_mut()
            .enable_interrupts(dmac::InterruptFlags::new().with_tcmpl(true));
        let xfer = Transfer::new(chan0, sbg_rx, unsafe { &mut *BUF_DST }, false)
            .expect("DMA err")
            .begin(
                hal::sercom::Sercom0::DMA_RX_TRIGGER,
                dmac::TriggerAction::BURST,
            );
        /* End DMAC config */

        // There is a bug within the HAL that improperly configures the RTC
        // in count32 mode. This is circumvented by first using clock mode then
        // converting to count32 mode.
        let rtc_temp = hal::rtc::Rtc::clock_mode(peripherals.RTC, 1024.hz(), &mut mclk);
        let mut rtc = rtc_temp.into_count32_mode();
        rtc.set_count32(0);

        state_send::spawn().ok();
        sensor_send::spawn().ok();
        blink::spawn().ok();
        // let sysclk: Hertz = gclk0.enable_gclk_out(pins.pa00.into_alternate()).0.freq();
        let mono = Systick::new(core.SYST, 48000000);

        let sbg: sbg::SBG = sbg::SBG::new(sbg_tx, rtc);
        (
            Shared {
                em: ErrorManager::new(),
                opt_xfer: xfer,
                buf_select: false,
                sbg,
                sensor_data: Sbg {
                    accel_x: 0.0,
                    accel_y: 0.0,
                    accel_z: 0.0,
                    velocity_n: 0.0,
                    velocity_e: 0.0,
                    velocity_d: 0.0,
                    quant_w: 0.0,
                    quant_x: 0.0,
                    quant_y: 0.0,
                    quant_z: 0.0,
                    pressure: 0.0,
                    height: 0.0,
                    roll: 0.0,
                    yaw: 0.0,
                    pitch: 0.0,
                    latitude: 0.0,
                    longitude: 0.0,
                },
            },
            Local {
                led,
                uart,
                sd,
                sbg_file,
                rx_fifo_0,
                rx_fifo_1,
                line_interrupts,
                tx,
                tx_event_fifo,
                aux,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    #[task(priority = 3, binds = CAN0, local = [line_interrupts, rx_fifo_0, rx_fifo_1])]
    fn can0(mut cx: can0::Context) {
        let line_interrupts = cx.local.line_interrupts;
        for interrupt in line_interrupts.iter_flagged() {
            match interrupt {
                Interrupt::RxFifo0NewMessage => {
                    for message in &mut cx.local.rx_fifo_0 {
                        info!("Message: {:?}", message.data())
                    }
                }
                Interrupt::RxFifo1NewMessage => {
                    for message in &mut cx.local.rx_fifo_1 {
                        info!("Message: {:?}", message.data())
                    }
                }
                i => info!("interrupt triggered"),
            }
        }
    }
    /**
     * Handles the DMA interrupt.
     * Handles the SBG data.
     * Logs data to the SD card.
     */
    #[task(binds = DMAC_0, local = [sd, sbg_file], shared = [sensor_data, opt_xfer, buf_select, sbg, &em])]
    fn dmac0(mut cx: dmac0::Context) {
        cx.shared.opt_xfer.lock(|xfer| {
            if xfer.complete() {
                cx.shared.buf_select.lock(|buf_select| {
                    cx.shared.em.run(|| {
                        match buf_select {
                            false => {
                                *buf_select = true;
                                let buf = xfer.recycle_source(unsafe { &mut *BUF_DST })?;
                                cx.shared.sbg.lock(|sbg| {
                                    cx.shared.sensor_data.lock(|sensor_data| {
                                        *sensor_data = sbg.read_data(buf);
                                    });
                                    cx.local.sd.write(&mut cx.local.sbg_file, buf)
                                })?;
                            }
                            true => {
                                *buf_select = false;
                                let buf = xfer.recycle_source(unsafe { &mut *BUF_DST2 })?;
                                cx.shared.sbg.lock(|sbg| {
                                    cx.shared.sensor_data.lock(|sensor_data| {
                                        *sensor_data = sbg.read_data(buf);
                                    });
                                    cx.local.sd.write(&mut cx.local.sbg_file, buf)
                                })?;
                            }
                        }
                        Ok(())
                    });
                });
            }
        });
    }

    /**
     * Sends a message to the radio over UART.
     */
    #[task(priority = 3, capacity = 10, local = [uart, tx_event_fifo, aux, tx], shared = [&em])]
    fn send_message(cx: send_message::Context, m: Message) {
        cx.shared.em.run(|| {
            let uart = cx.local.uart;

            let payload: Vec<u8, 255> = postcard::to_vec(&m)?;

            let mav_message = mav_message::mavlink_postcard_message(payload);

            mavlink::write_versioned_msg(
                uart,
                mavlink::MavlinkVersion::V2,
                mav_message::get_mav_header(),
                &mav_message,
            )?;

            let mut payload = [0_u8; 8];
            payload.fill(0);
            cx.local.tx.transmit_queued(
                tx::MessageBuilder {
                    id: ecan::Id::Standard(ecan::StandardId::new(0).unwrap()),
                    frame_type: tx::FrameType::Classic(tx::ClassicFrameType::Data(&payload)),
                    // }, 
                    // // {
                    // //     payload: &payload,
                    // //     bit_rate_switching: true,
                    // //     force_error_state_indicator: false,
                    // // },
                    store_tx_event: Some(0),
                }
                .build()
                .unwrap(),
            )?;

            info!("Sent message: {:?}", m);

            Ok(())
        });
    }

    /**
     * Sends the state of the system.
     */
    #[task(shared = [&em])]
    fn state_send(cx: state_send::Context) {
        let em = cx.shared.em;

        let state = State {
            status: Status::Uninitialized,
            has_error: em.has_error(),
            voltage: 12.1,
        };

        let message = Message::new(&monotonics::now(), MainBoard, state);

        cx.shared.em.run(|| {
            spawn!(send_message, message)?;

            Ok(())
        });

        spawn_after!(state_send, 10.secs()).ok();
    }

    /**
     * Sends information about the sensors.
     */
    #[task(shared = [sensor_data, &em])]
    fn sensor_send(mut cx: sensor_send::Context) {
        let data = cx
            .shared
            .sensor_data
            .lock(|sensor_data| sensor_data.clone());
        cx.shared.em.run(|| {
            let message = Message::new(&monotonics::now(), MainBoard, Sensor::new(9, data));

            spawn!(send_message, message)?;

            Ok(())
        });

        spawn_after!(sensor_send, 250.millis()).ok();
    }

    /**
     * Simple blink task to test the system.
     */
    #[task(local = [led], shared = [&em])]
    fn blink(cx: blink::Context) {
        cx.shared.em.run(|| {
            cx.local.led.toggle()?;

            let time = monotonics::now().duration_since_epoch().to_secs();
            info!("Seconds since epoch: {}", time);
            if cx.shared.em.has_error() {
                spawn_after!(blink, 200.millis())?;
            } else {
                spawn_after!(blink, 1.secs())?;
            }
            Ok(())
        });
    }
}

#[no_mangle]
pub extern "C" fn _sbrk() {}

#[no_mangle]
pub extern "C" fn _write() {}

#[no_mangle]
pub extern "C" fn _close() {}

#[no_mangle]
pub extern "C" fn _lseek() {}

#[no_mangle]
pub extern "C" fn _read() {}

#[no_mangle]
pub extern "C" fn _fstat() {}

#[no_mangle]
pub extern "C" fn _isatty() {}

#[no_mangle]
pub extern "C" fn _exit() {}

#[no_mangle]
pub extern "C" fn _open() {}

#[no_mangle]
pub extern "C" fn _kill() {}

#[no_mangle]
pub extern "C" fn _getpid() {}
