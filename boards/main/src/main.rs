#![no_std]
#![no_main]
#[link(name = "c", kind = "static")]
extern "C" {}
use core::sync::atomic::AtomicBool;

use atsamd_hal as hal;
use atsamd_hal::gpio::*;
use atsamd_hal::pac;
use atsamd_hal::prelude::nb::block;
use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::uart::{Duplex, Uart};
use atsamd_hal::sercom::{uart, IoSet1, Sercom1, Sercom5};
use common_arm::*;
use defmt::info;
use defmt_rtt as _;
use hal::dmac::BufferPair;
use hal::gpio::Pins;
use hal::gpio::PA14;
use hal::gpio::{Pin, PushPullOutput};
use hal::prelude::*;
use hal::sercom::uart::NineBit;
use hal::sercom::Sercom0;
use hal::sercom::Sercom3;
use hal::time::Hertz;
use hal::timer;
use heapless::Vec;
use messages::sender::Sender::MainBoard;
use messages::sensor::{Sbg, Sensor};
use messages::*;

use panic_halt as _;
use postcard::to_vec_cobs;
use systick_monotonic::*;
/* SBG */
use sbg_rs::sbg::SBG_COUNT;
use sbg_rs::sbg::SBG_RING_BUFFER;
/* Type Def */
type Pads = uart::PadsFromIds<Sercom3, IoSet1, PA23, PA22>;
type PadsCDC = uart::PadsFromIds<Sercom5, IoSet1, PB17, PB16>;
type PadsSBG = uart::PadsFromIds<Sercom0, IoSet1, PA09, PA08>;
type Config = uart::Config<Pads, EightBit>;
type ConfigCDC = uart::Config<PadsCDC, EightBit>;
type ConfigSBG = uart::Config<PadsSBG, EightBit>;
use core::sync::atomic::AtomicU8;
use embedded_sdmmc::File;
use hal::dmac;
use hal::time::{Milliseconds, Nanoseconds};
use sbg_rs::sbg;

// type SBGWaker = fn(dmac::CallbackStatus) -> ();
type SBGTransfer = dmac::Transfer<
    dmac::Channel<dmac::Ch0, dmac::Busy>,
    BufferPair<Uart<ConfigSBG, uart::RxDuplex>, SBGBuffer>,
>;
const SBG_BUFFER_SIZE: usize = 4096;
type SBGBuffer = &'static mut [u8; SBG_BUFFER_SIZE];
static mut BUF_DST: SBGBuffer = &mut [0; SBG_BUFFER_SIZE];
static mut BUF_DST2: SBGBuffer = &mut [0; SBG_BUFFER_SIZE];

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {
    use hal::{dmac::Transfer, sercom::Sercom};

    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        opt_xfer: SBGTransfer,
        buf_select: AtomicU8,
        sbg: sbg::SBG,
        sensor_data: Sbg,
    }

    #[local]
    struct Local {
        led: Pin<PA14, PushPullOutput>,
        uart: Uart<Config, Duplex>,
        sd: SdInterface,
        sbg_file: File,
        // sbg_rx: Option<Uart<ConfigSBG, uart::RxDuplex>>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut peripherals = cx.device;
        let core = cx.core;

        let mut clocks = hal::clock::GenericClockController::with_external_32kosc(
            peripherals.GCLK,
            &mut peripherals.MCLK,
            &mut peripherals.OSC32KCTRL,
            &mut peripherals.OSCCTRL,
            &mut peripherals.NVMCTRL,
        );

        let pins = Pins::new(peripherals.PORT);
        let led = pins.pa14.into_push_pull_output();

        clocks.configure_gclk_divider_and_source(
            pac::gclk::pchctrl::GEN_A::GCLK2,
            1,
            pac::gclk::genctrl::SRC_A::DFLL,
            false,
        );

        // UART
        let gclk2 = clocks
            .get_gclk(pac::gclk::pchctrl::GEN_A::GCLK2)
            .expect("Could not get gclk 2.");

        /* Start SD config */
        let mclk = &mut peripherals.MCLK;
        clocks.configure_gclk_divider_and_source(
            pac::gclk::pchctrl::GEN_A::GCLK5,
            1,
            pac::gclk::genctrl::SRC_A::DFLL,
            false,
        );
        let gclk5 = clocks
            .get_gclk(pac::gclk::pchctrl::GEN_A::GCLK5)
            .expect("Could not get gclk 5.");
        let spi_clk = clocks
            .sercom1_core(&gclk5)
            .expect("Could not configure Sercom 1 clock.");
        let sercom = peripherals.SERCOM1;
        let cs = pins.pa18.into_push_pull_output();
        let sck = pins.pa17.into_push_pull_output();
        let miso = pins.pa19.into_push_pull_output();
        let mosi = pins.pa16.into_push_pull_output();
        let mut sd = sd::SdInterface::new(mclk, sercom, spi_clk, cs, sck, miso, mosi);
        let sbg_file = sd.open_file("raw.txt").expect("Could not open file");
        /* End SD config */
        /* Start Radio config */
        let uart_clk = clocks
            .sercom3_core(&gclk2)
            .expect("Could not configure Sercom 1 clock.");

        let pads = uart::Pads::<hal::sercom::Sercom3, _>::default()
            .rx(pins.pa23)
            .tx(pins.pa22);
        let uart = Config::new(
            &peripherals.MCLK,
            peripherals.SERCOM3,
            pads,
            uart_clk.freq(),
        )
        .baud(
            9600.hz(),
            uart::BaudMode::Fractional(uart::Oversampling::Bits16),
        )
        .enable();
        /* End Radio config */

        /* DMAC config */
        let mut dmac = dmac::DmaController::init(peripherals.DMAC, &mut peripherals.PM);
        let channels = dmac.split();
        let mut chan0 = channels.0.init(dmac::PriorityLevel::LVL3);
        // chan0.as_mut().enable_interrupts(dmac::InterruptFlags::new().with_tcmpl(true));
        chan0
            .as_mut()
            .enable_interrupts(dmac::InterruptFlags::new().with_tcmpl(true));
        /* End DMAC config */
        clocks.configure_gclk_divider_and_source(
            pac::gclk::pchctrl::GEN_A::GCLK3,
            1,
            pac::gclk::genctrl::SRC_A::DFLL,
            false,
        );
        let gclk3 = clocks
            .get_gclk(pac::gclk::pchctrl::GEN_A::GCLK3)
            .expect("Could not get gclk 2.");

        /* Start UART CDC config */
        let cdc_clk = clocks
            .sercom5_core(&gclk3)
            .expect("Could not configure Sercom 5 clock.");

        /* Start UART CDC config */
        let sbg_clk = clocks
            .sercom0_core(&gclk3)
            .expect("Could not configure Sercom 0 clock.");

        let pads = uart::Pads::<Sercom5, _>::default()
            .rx(pins.pb17)
            .tx(pins.pb16);
        let uart_cdc = ConfigCDC::new(&peripherals.MCLK, peripherals.SERCOM5, pads, cdc_clk.freq())
            .baud(
                115200.hz(),
                uart::BaudMode::Fractional(uart::Oversampling::Bits16),
            )
            .enable();
        let padsSBG = uart::Pads::<Sercom0, _>::default()
            .rx(pins.pa09)
            .tx(pins.pa08);
        let mut uart_sbg = ConfigSBG::new(
            &peripherals.MCLK,
            peripherals.SERCOM0,
            padsSBG,
            sbg_clk.freq(),
        )
        .baud(
            115200.hz(),
            uart::BaudMode::Fractional(uart::Oversampling::Bits8),
        )
        .enable();

        let (mut sbg_rx, sbg_tx) = uart_sbg.split();
        // let waker: SBGWaker = |_| {
        //     handleTransfer::spawn().ok();
        // };
        let xfer = Transfer::new(chan0, sbg_rx, unsafe { &mut *BUF_DST }, false)
            .expect("DMA err")
            .begin(
                hal::sercom::Sercom0::DMA_RX_TRIGGER,
                dmac::TriggerAction::BURST,
            );

        // There is a bug within the HAL that improperly configures the RTC
        // in count32 mode. This is circumvented by first using clock mode then
        // converting to count32 mode.
        let mut rtc_temp =
            hal::rtc::Rtc::clock_mode(peripherals.RTC, 1024.hz(), &mut peripherals.MCLK);
        let mut rtc = rtc_temp.into_count32_mode();
        rtc.set_count32(0);

        sbg_init::spawn().ok();
        state_send::spawn().ok();
        sensor_send::spawn().ok();
        blink::spawn().ok();
        let sysclk: Hertz = clocks.gclk0().into();
        let mono = Systick::new(core.SYST, sysclk.0);

        let mut sbg: sbg::SBG = sbg::SBG::new(sbg_tx, rtc);
        (
            Shared {
                em: ErrorManager::new(),
                opt_xfer: xfer,
                buf_select: AtomicU8::new(0),
                sbg,
                sensor_data: Sbg {
                    accel: 0.0,
                    speed: 0.0,
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

    #[task(priority = 3, shared = [sbg])]
    fn sbg_init(mut cx: sbg_init::Context) {
        cx.shared.sbg.lock(|sbg| {
            sbg.setup();
        });
    }

    #[task(binds = DMAC_0, local = [sd, sbg_file], shared = [sensor_data, opt_xfer, buf_select, sbg])]
    fn dmac0(mut cx: dmac0::Context) {
        cx.shared.opt_xfer.lock(|xfer| {
            cx.shared.sbg.lock(|sbg| {
                if xfer.complete() {
                    let mut buf_select = cx
                        .shared
                        .buf_select
                        .lock(|buf_select| *buf_select.get_mut());
                    match buf_select {
                        0 => {
                            let buf: &'static [u8; SBG_BUFFER_SIZE] =
                                xfer.recycle_source(unsafe { BUF_DST }).expect("err");
                            cx.local.sd.write(&mut cx.local.sbg_file, buf);
                            cx.shared
                                .sensor_data
                                .lock(|sensor_data| *sensor_data = sbg.readData(buf));
                            cx.shared
                                .buf_select
                                .lock(|buf_select| *buf_select.get_mut() = 1)
                        }
                        1 => {
                            let buf: &'static [u8; SBG_BUFFER_SIZE] =
                                xfer.recycle_source(unsafe { BUF_DST2 }).expect("err");
                            cx.local.sd.write(&mut cx.local.sbg_file, buf);
                            cx.shared
                                .sensor_data
                                .lock(|sensor_data| *sensor_data = sbg.readData(buf));
                            cx.shared
                                .buf_select
                                .lock(|buf_select| *buf_select.get_mut() = 0)
                        }
                        _ => (),
                    }
                }
            });
        });
    }

    #[task(capacity = 10, local = [uart], shared = [&em])]
    fn send_message(cx: send_message::Context, m: Message) {
        cx.shared.em.run(|| {
            let uart = cx.local.uart;

            let payload: Vec<u8, 64> = to_vec_cobs(&m)?;

            for x in payload {
                block!(uart.write(x))?;
            }

            info!("Sent message: {:?}", m);

            Ok(())
        });
    }

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

    #[task(shared = [sensor_data, &em])]
    fn sensor_send(mut cx: sensor_send::Context) {
        cx.shared.sensor_data.lock(|sensor_data| {
            cx.shared.em.run(|| {
                let message = Message::new(
                    &monotonics::now(),
                    MainBoard,
                    Sensor::new(9, sensor_data.clone()),
                );

                spawn!(send_message, message)?;

                Ok(())
            });
        });

        spawn_after!(sensor_send, 2.secs()).ok();
    }

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
