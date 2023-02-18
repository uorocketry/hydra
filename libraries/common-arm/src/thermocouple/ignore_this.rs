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

//********* Me ************
use atsamd_hal::sercom::pad::IoSet1;
use atsamd_hal::sercom::spi::lengths::U2;
use atsamd_hal::sercom::spi::{BitOrder, MODE_1};
use atsamd_hal::sercom::{spi, Sercom0};
use nb::block;
//*********************

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
        pac::gclk::pchctrl::GEN_A::GCLK2,
        1,
        pac::gclk::genctrl::SRC_A::DFLL,
        false,
    );

    let mut delay = hal::delay::Delay::new(p2.SYST, &mut clock);

    /*******************************************
     * SPI ADC thermocouple
     *******************************************/
    // 1. # configure pins
    let pads = spi::Pads::<Sercom0, IoSet1>::default()
        .sclk(pins.pa09)
        .data_in(pins.pa08)
        .data_out(pins.pa11);

    // 2. # SPI config
    let mclk = peripherals.MCLK;
    let sercom = peripherals.SERCOM0;
    let freq = 10.mhz();
    // 3. # Instantiate SPI
    let mut spi = spi::Config::new(&mclk, sercom, pads, freq)
        .baud(2.mhz())
        .length::<U2>()
        .bit_order(BitOrder::LsbFirst)
        .spi_mode(MODE_1)
        .enable();

    block!(spi.send(0xAA55)).expect("SPI send failed.");

    let rcvd = block!(spi.read()).unwrap();

    loop {
        led.set_high().unwrap();
        delay.delay_ms(1000_u16);
        info!("Test");
        led.set_low().unwrap();
        delay.delay_ms(1000_u16);
    }
