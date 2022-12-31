#![no_std]
#![no_main]

use atsamd_hal as hal;
use atsamd_hal::gpio::*;
use atsamd_hal::pac;
use atsamd_hal::prelude::nb::block;
use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::uart::{Duplex, Uart};
use atsamd_hal::sercom::{uart, IoSet1, Sercom1};
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

type Pads = uart::PadsFromIds<Sercom1, IoSet1, PA17, PA16>;
type Config = uart::Config<Pads, EightBit>;

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0])]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: Pin<PA14, PushPullOutput>,
        uart: Uart<Config, Duplex>,
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

        /* Start UART CDC config */
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
        /* End UART CDC config */

        state_send::spawn().ok();
        sensor_send::spawn().ok();
        blink::spawn().ok();

        let sysclk: Hertz = clocks.gclk0().into();
        let mono = Systick::new(core.SYST, sysclk.0);

        (Shared {}, Local { led, uart }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    #[task(capacity = 10, local = [uart])]
    fn send_message(cx: send_message::Context, m: Message) {
        let uart = cx.local.uart;

        let payload: Vec<u8, 64> = match to_vec_cobs(&m) {
            Ok(x) => x,
            Err(_) => return,
        };

        for x in payload {
            block!(uart.write(x)).ok();
        }

        info!("Sent message: {:?}", m);
    }

    #[task]
    fn state_send(_: state_send::Context) {
        let state = State {
            status: Status::Uninitialized,
            has_error: false,
            voltage: 12.1,
        };

        let message = Message::new(&monotonics::now(), MainBoard, state);

        send_message::spawn(message).ok();
        state_send::spawn_after(10.secs()).ok();
    }

    #[task]
    fn sensor_send(_: sensor_send::Context) {
        let sbg = Sbg {
            accel: 9.8,
            speed: 0.0,
            pressure: 100.1,
            height: 200.4,
        };

        let message = Message::new(&monotonics::now(), MainBoard, Sensor::new(9, sbg));

        send_message::spawn(message).ok();
        sensor_send::spawn_after(2.secs()).ok();
    }

    #[task(local = [led])]
    fn blink(cx: blink::Context) {
        cx.local.led.toggle().unwrap();

        let time = monotonics::now().duration_since_epoch().to_secs();
        info!("Seconds since epoch: {}", time);

        blink::spawn_after(1.secs()).ok();
    }
}
