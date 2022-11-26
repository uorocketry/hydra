#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::asm;
use cortex_m_rt::entry;

use atsamd_hal::prelude::*;
use atsamd_hal as hal;
use hal::pac as pac;

use hal::gpio::Pins;

use pac::Peripherals;

#[entry]
fn main() -> ! {
    asm::nop(); // To not have main optimize to abort in release mode, remove when you add code

    let mut peripherals = Peripherals::take().unwrap();
    let mut p2 = cortex_m::peripheral::Peripherals::take().unwrap();

    let pins = Pins::new(peripherals.PORT);
    let mut led = pins.pa14.into_push_pull_output();
    

    let mut clock = hal::clock::GenericClockController::with_internal_32kosc(
            peripherals.GCLK,
            &mut peripherals.MCLK,
            &mut peripherals.OSC32KCTRL,
            &mut peripherals.OSCCTRL,
            &mut peripherals.NVMCTRL);

    let mut delay = hal::delay::Delay::new(p2.SYST, &mut clock);

    loop {
        led.set_high().unwrap();
        delay.delay_ms(1000_u16);
        led.set_low().unwrap();
        delay.delay_ms(1000_u16);

    }
}
