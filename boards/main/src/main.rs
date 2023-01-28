#![no_std]
#![no_main]

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
use sbg_rs;
use cortex_m::interrupt::Mutex;
/* Type Def */
type Pads = uart::PadsFromIds<Sercom1, IoSet1, PA17, PA16>;
type PadsCDC = uart::PadsFromIds<Sercom5, IoSet1, PB17, PB16>;
type Config = uart::Config<Pads, EightBit>;
type ConfigCDC = uart::Config<PadsCDC, EightBit>;

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0])]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
    }

    #[local]
    struct Local {
        led: Pin<PA14, PushPullOutput>,
        uart: Uart<Config, Duplex>,
        sbg: Mutex<sbg_rs::sbg::SBG<Uart<ConfigCDC, Duplex>>>,
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

        let pads = uart::Pads::<Sercom5, _>::default()
            .rx(pins.pb17)
            .tx(pins.pb16);
        let mut uart_cdc = ConfigCDC::new(
            &peripherals.MCLK,
            peripherals.SERCOM5,
            pads,
            cdc_clk.freq(),
        )
        .baud(
            9600.hz(),
            uart::BaudMode::Fractional(uart::Oversampling::Bits16),
        )
        .enable();

        state_send::spawn().ok();
        sensor_send::spawn().ok();
        blink::spawn().ok();

        let sysclk: Hertz = clocks.gclk0().into();
        let mono = Systick::new(core.SYST, sysclk.0);

        // Put the SBG object into a mutex 
        let mut sbg = Mutex::new(sbg_rs::sbg::SBG::new(uart_cdc));

        (
            Shared {
                em: ErrorManager::new(),
            },
            Local { led, uart, sbg },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
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

    #[task(shared = [&em])]
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

    #[task(local = [led, sbg], shared = [&em])]
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
