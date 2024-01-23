#![no_std]
#![no_main]

use atsamd_hal as hal;
use atsamd_hal::prelude::*;
use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use hal::gpio::Pins;
use hal::pac;
use pac::Peripherals;
use panic_halt as _;

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let p2 = cortex_m::peripheral::Peripherals::take().unwrap();

    let pins = Pins::new(peripherals.PORT);
    let mut led = pins.pa14.into_push_pull_output();

    // External 32KHz clock for stability
    let mut clock = hal::clock::GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );

    clock.configure_gclk_divider_and_source(
        pac::gclk::pchctrl::GENSELECT_A::GCLK2,
        1,
        pac::gclk::genctrl::SRCSELECT_A::DFLL,
        false,
    );

    let mut delay = hal::delay::Delay::new(p2.SYST, &mut clock);

    loop {
        led.set_high().unwrap();
        delay.delay_ms(1000_u16);
        info!("Test");
        led.set_low().unwrap();
        delay.delay_ms(1000_u16);
    }
}
