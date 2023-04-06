#![no_std]
#![no_main]
#[link(name = "c", kind = "static")]
extern {}
use atsamd_hal as hal;
use atsamd_hal::gpio::*;
use atsamd_hal::pac;
use atsamd_hal::prelude::nb::block;
use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::uart::{Duplex, Uart};
use atsamd_hal::sercom::{uart, IoSet1, Sercom1, Sercom5};
use hal::dmac::BufferPair;
use pac::interrupt::DMAC_0;
use hal::sercom::Sercom0;
use hal::sercom::Sercom3;
use hal::sercom::uart::NineBit;
use hal::timer;
use common_arm::*;
use defmt::info;
use defmt_rtt as _;
use hal::gpio::Pins;
use hal::gpio::PA14;
use hal::gpio::{Pin, PushPullOutput};
use hal::prelude::*;
use hal::time::Hertz;
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
use hal::{time::{Nanoseconds, Milliseconds}};
use sbg_rs::sbg;
use hal::dmac;
use embedded_sdmmc::File;
// type TransferBuffer = &'static mut [u8; LENGTH];
// static mut BUF_SRC: TransferBuffer = &mut [0; LENGTH];
// static mut BUF_DST: TransferBuffer = &mut [0; LENGTH];
type SBGWaker = fn(dmac::CallbackStatus) -> ();
type SBGTransfer = dmac::Transfer<dmac::Channel<dmac::Ch0, dmac::Busy>, BufferPair<Uart<ConfigSBG, uart::RxDuplex>, SBGBuffer>, SBGWaker>;
type SBGBuffer = &'static mut [u8; 4096];
static mut BUF_DST: SBGBuffer = &mut [0; 4096];

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        sbg: sbg::SBG,
        opt_xfer: Option<SBGTransfer>,
        // sbg_rx: Uart<ConfigSBG, uart::RxDuplex>,
        // opt_channel: Option<dmac::Channel<dmac::Ch0, dmac::Ready>>,
    }

    #[local]
    struct Local {
        led: Pin<PA14, PushPullOutput>,
        uart: Uart<Config, Duplex>,
        sd: SdInterface,
        sbg_file: File,
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
        clocks.configure_gclk_divider_and_source(pac::gclk::pchctrl::GEN_A::GCLK5, 1, pac::gclk::genctrl::SRC_A::DFLL, false);
        let gclk5 = clocks.get_gclk(pac::gclk::pchctrl::GEN_A::GCLK5).expect("Could not get gclk 5.");
        let spi_clk = clocks.sercom1_core(&gclk5).expect("Could not configure Sercom 1 clock.");
        let sercom = peripherals.SERCOM1;
        let cs = pins.pa18.into_push_pull_output();
        let sck = pins.pa17.into_push_pull_output();
        let miso = pins.pa19.into_push_pull_output();
        let mosi = pins.pa16.into_push_pull_output();
        let mut sd = sd::SdInterface::new(mclk, sercom, spi_clk, cs, sck, miso, mosi);
        let sbg_file = sd.open_file("SBG_Raw.txt").expect("Could not open file");
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
        let mut dmac = dmac::DmaController::init(
            peripherals.DMAC,
            &mut peripherals.PM,
        );
        let channels = dmac.split();
        let mut chan0 = channels.0.init(dmac::PriorityLevel::LVL0);
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
        let uart_cdc = ConfigCDC::new(
            &peripherals.MCLK,
            peripherals.SERCOM5,
            pads,
            cdc_clk.freq(),
        )
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

        let (sbg_rx, sbg_tx) = uart_sbg.split();
        let waker: SBGWaker = |_| {handleTransfer::spawn().ok();};
        let xfer = sbg_rx.receive_with_dma(unsafe {&mut *BUF_DST}, chan0, waker);        
        // uart_sbg.enable_interrupts(hal::sercom::uart::Flags::RXC);
        // uart_cdc.receive_with_dma(unsafe{&mut *BUF_DST}, chan0, |_| {});

        tc2::spawn().ok();
        state_send::spawn().ok();
        sensor_send::spawn().ok();
        blink::spawn().ok();
        let sysclk: Hertz = clocks.gclk0().into();
        let mono = Systick::new(core.SYST, sysclk.0);

        let mut sbg: sbg::SBG = sbg::SBG::new(sbg_tx);

        (
            Shared {
                em: ErrorManager::new(),
                sbg,
                opt_xfer: Some(xfer),
                // opt_channel: Some(chan0),
            },
            Local { led, uart, sd, sbg_file},
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    #[task(shared = [opt_xfer])]
    fn handleTransfer(mut cx: handleTransfer::Context) {
        match cx.shared.opt_xfer.lock(|xfer| xfer.take()) {
            Some(xfer) => {
                let (xfer, buf, chan) = xfer.wait();
            },
            None => {
                ()
            }
        }
    }

    #[task(binds = DMAC_0, local = [sd, sbg_file], shared = [opt_xfer])]
    fn dmac0(mut cx: dmac0::Context) {
        let mut sd = cx.local.sd;
        match cx.shared.opt_xfer.lock(|xfer| xfer.take()) {
            Some(mut xfer) => {
                let (chan, rx, buf) = xfer.wait();
                let waker: SBGWaker = |_| {handleTransfer::spawn().ok();};
                sd.write(&mut cx.local.sbg_file, buf).unwrap();
                for i in 0..buf.len() {                
                    unsafe{SBG_RING_BUFFER.push(buf[i])};
                }
                spawn!(sensor_send).ok();
                let newXfer = rx.receive_with_dma(buf, chan, waker);
                cx.shared.opt_xfer.lock(|xfer| *xfer = Some(newXfer));
                // cx.shared.opt_xfer.lock(|xfer| *xfer = Some(xfer));
            },
            None => {
                // match cx.shared.opt_channel.lock(|chan| chan.take()) {
                //     Some(chan) => {
                //         let waker: SBGWaker = |_| {handleTransfer::spawn().ok();};
                //     },
                //     None => {
                //         ()
                //     }
                // }
            }
        }
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

    #[task(priority = 3, shared = [&em, sbg])]
    fn sensor_send(mut cx: sensor_send::Context) {   

        cx.shared.em.run(|| {
            cx.shared.sbg.lock(|shared| {
                // if shared.isInitialized == false {
                //     shared.setup();
                // } else {
                    shared.readData();
                // }
            });

            let sbg = Sbg {
                accel: 9.8,
                speed: 0.0,
                pressure: 100.1,
                height: 200.4,
            };

            let message = Message::new(&monotonics::now(), MainBoard, Sensor::new(9, sbg));

            spawn!(send_message, message)?;

            Ok(())
        });

        // spawn_after!(sensor_send, 2.secs()).ok();
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

    #[task(priority = 2, shared = [&em])] 
    fn tc2(cx: tc2::Context) {
        cx.shared.em.run(|| {
            unsafe {*SBG_COUNT.get_mut() += 100;}
            spawn_after!(tc2, 100.millis());
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