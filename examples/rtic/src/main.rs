#![no_std]
#![no_main]

use atsamd_hal as hal;
use defmt::info;
use defmt_rtt as _;
use hal::gpio::Pins;
use hal::gpio::PA14;
use hal::gpio::{Pin, PushPullOutput};
use hal::prelude::*;
use hal::time::Hertz;
use panic_halt as _;
use systick_monotonic::*;

// "dispatchers" here can be any unused interrupts
#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0])]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: Pin<PA14, PushPullOutput>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut peripherals = cx.device;
        let mut core = cx.core;

        // External 32KHz clock for stability
        let mut clocks = hal::clock::GenericClockController::with_external_32kosc(
            peripherals.GCLK,
            &mut peripherals.MCLK,
            &mut peripherals.OSC32KCTRL,
            &mut peripherals.OSCCTRL,
            &mut peripherals.NVMCTRL,
        );

        let pins = Pins::new(peripherals.PORT);
        let led = pins.pa14.into_push_pull_output();

        // Tell the MCU to sleep deeply for maximum power savings
        core.SCB.set_sleepdeep();

        // Spawn the LED blink task right after init
        blink::spawn().ok();

        // Use the system's Systick for RTIC to keep track of the time
        let sysclk: Hertz = clocks.gclk0().into();
        let mono = Systick::new(core.SYST, sysclk.0);

        (Shared {}, Local { led }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            // Put the MCU to sleep until an interrupt happens
            rtic::export::wfi();
        }
    }

    #[task(local = [led])]
    fn blink(cx: blink::Context) {
        cx.local.led.toggle().unwrap();

        let time = monotonics::now().duration_since_epoch().to_secs();
        info!("Seconds since epoch: {}", time);

        blink::spawn_after(1.secs()).ok();
    }
}
