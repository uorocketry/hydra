#![no_std]
#![no_main]

use atsamd_hal as hal;
use atsamd_hal::prelude::*;
use cortex_m_rt::entry;
use defmt::println;
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
    let mut clock = hal::clock::GenericClockController::with_internal_32kosc(
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
    println!("Clock configured");
    let mut tx_pin = pins.pa09.into_push_pull_output();
    let mut rx_pin = pins.pa08.into_push_pull_output();
    match rx_pin.set_high() {
        Ok(_) => println!("RX pin set high"),
        Err(_) => println!("RX pin set low"),
    };
    tx_pin.set_high().unwrap();

    let mut delay = hal::delay::Delay::new(p2.SYST, &mut clock);

    loop {
        led.set_high().unwrap();
        delay.delay_ms(1000_u16);
        println!("Test");
        led.set_low().unwrap();
        delay.delay_ms(1000_u16);
    }
}
