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
use atsamd_hal::timer::TimerCounter;
use hal::sercom::Sercom0;
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
use pac::interrupt;
/* Type Def */
type Pads = uart::PadsFromIds<Sercom1, IoSet1, PA17, PA16>;
type PadsCDC = uart::PadsFromIds<Sercom5, IoSet1, PB17, PB16>;
type PadsSBG = uart::PadsFromIds<Sercom0, IoSet1, PA09, PA08>;
type Config = uart::Config<Pads, EightBit>;
type ConfigCDC = uart::Config<PadsCDC, EightBit>;
type ConfigSBG = uart::Config<PadsSBG, EightBit>;
use hal::{time::{Nanoseconds, Milliseconds}};
use sbg_rs::sbg;

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {

    use hal::pac::TC2;

    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
    }

    #[local]
    struct Local {
        led: Pin<PA14, PushPullOutput>,
        uart: Uart<Config, Duplex>,
        // uart_cdc: Uart<ConfigCDC, Duplex>,
        sbg: sbg::SBG<Uart<ConfigSBG, Duplex>>,
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

        // // let rtc = hal::rtc::Rtc::count32_mode(rtc, rtc_clock_freq, &mut peripherals.MCLK);
        // clocks.configure_gclk_divider_and_source(
        //     pac::gclk::pchctrl::GEN_A::GCLK4,
        //     32,
        //     pac::gclk::genctrl::SRC_A::GCLKGEN1,
        //     false,
        // );
        // let gclk4 = clocks.get_gclk(pac::gclk::pchctrl::GEN_A::GCLK4).expect("Could not get gclk 4.");
        // let tc2_clk = clocks.tc2_tc3(&gclk4).expect("Could not configure the TC2 clock");
        // let mut sbg_timer = TimerCounter::tc2_(&tc2_clk, peripherals.TC2, &mut peripherals.MCLK);
        // Could use an interrupt timer
        
        // UART
        let gclk2 = clocks
            .get_gclk(pac::gclk::pchctrl::GEN_A::GCLK2)
            .expect("Could not get gclk 2.");

        /* Start Radio config */
        let uart_clk = clocks
            .sercom1_core(&gclk2)
            .expect("Could not configure Sercom 1 clock.");

        let pads = uart::Pads::<Sercom1, _>::default()
            .rx(pins.pa17)
            .tx(pins.pa16);
        let uart = Config::new(
            &peripherals.MCLK,
            peripherals.SERCOM1,
            pads,
            uart_clk.freq(),
        )
        .baud(
            9600.hz(),
            uart::BaudMode::Fractional(uart::Oversampling::Bits16),
        )
        .enable();
        /* End Radio config */

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

        clocks.configure_gclk_divider_and_source(
            pac::gclk::pchctrl::GEN_A::GCLK5,
            1,
            pac::gclk::genctrl::SRC_A::DFLL,
            false,
        );
        let gclk5 = clocks
            .get_gclk(pac::gclk::pchctrl::GEN_A::GCLK5)
            .expect("Could not get gclk 2.");

        /* Start UART CDC config */
        let sbg_clk = clocks
            .sercom0_core(&gclk5)
            .expect("Could not configure Sercom 0 clock.");

        let pads = uart::Pads::<Sercom5, _>::default()
            .rx(pins.pb17)
            .tx(pins.pb16);
        let mut uart_cdc = ConfigCDC::new(
            &peripherals.MCLK,
            peripherals.SERCOM5,
            pads,
            cdc_clk.freq(),
        )
        .stop_bits(uart::StopBits::OneBit)
        .parity(
            uart::Parity::Odd,
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
        .stop_bits(uart::StopBits::OneBit)
        .parity(
            uart::Parity::Odd,
        )
        .baud(
            115200.hz(),
            uart::BaudMode::Fractional(uart::Oversampling::Bits8),
        )
        .enable();

        state_send::spawn().ok();
        sensor_send::spawn().ok();
        blink::spawn().ok();
        tc2::spawn().ok();
        let sysclk: Hertz = clocks.gclk0().into();
        let mono = Systick::new(core.SYST, sysclk.0);

        let mut sbg = sbg::SBG::new(uart_sbg);
        // rtic::pend(pac::Interrupt::TC2);
        // sbg_timer.enable_interrupt();
        // sbg_timer.start(Milliseconds(1));
        (
            Shared {
                em: ErrorManager::new(),
            },
            Local { led, uart, sbg },
            init::Monotonics(mono),
        )
    }

    #[idle(local = [sbg])]
    fn idle(cx: idle::Context) -> ! {
        // let mut cdc_uart = cx.local.uart_cdc;
        loop {
            if cx.local.sbg.isInitialized == false {
                cx.local.sbg.setup();
            } else {
                cx.local.sbg.readData();
            }
            // let data = match cdc_uart.read() {
            //     Ok(data) =>  cdc_uart.write(data).unwrap(),
            //     Err(_) => (),
            // };
                       // rtic::pend(pac::Interrupt::TC2);
            rtic::export::wfi();
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

    #[task(priority = 2, shared = [&em])]
    fn sensor_send(cx: sensor_send::Context) {   

        cx.shared.em.run(|| {
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

        spawn_after!(sensor_send, 2.secs()).ok();
    }

    #[task(priority = 1, local = [led], shared = [&em])]
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

    #[task(priority = 1, shared = [&em])] 
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