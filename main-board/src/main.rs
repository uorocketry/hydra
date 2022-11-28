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

use sdio_host as sdio;

#[entry]
fn main() -> ! {
    asm::nop(); // To not have main optimize to abort in release mode, remove when you add code

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
            &mut peripherals.NVMCTRL);

    clock.configure_gclk_divider_and_source(pac::gclk::pchctrl::GEN_A::GCLK2, 1, pac::gclk::genctrl::SRC_A::DFLL, false);
    let gclk2 = clock.get_gclk(pac::gclk::pchctrl::GEN_A::GCLK2).expect("Could not get generic clock 2");

    let mut delay = hal::delay::Delay::new(p2.SYST, &mut clock);
    
    let uart_clk = clock.sercom5_core(&gclk2).expect("Could not configure sercom 5 clock.");

    // _ defaults the character size to 8bit.
    let pads = hal::sercom::uart::Pads::<hal::sercom::Sercom5, _>::default().rx(pins.pb17).tx(pins.pb16);
    let mut uart = hal::sercom::uart::Config::new(&peripherals.MCLK, peripherals.SERCOM5, pads, uart_clk.freq())
    .baud(9600.hz(), hal::sercom::uart::BaudMode::Fractional(hal::sercom::uart::Oversampling::Bits16))
    .enable();

    loop {
        led.set_high().unwrap();
        delay.delay_ms(1000_u16);
        for byte in b"Hello, world!\r\n" {
            nb::block!(uart.write(*byte)).unwrap();
        }
        led.set_low().unwrap();
        delay.delay_ms(1000_u16);

    }
}
