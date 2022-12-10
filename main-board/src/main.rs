#![no_std]
#![no_main]

use atsamd_hal as hal;
use atsamd_hal::prelude::*;
use cortex_m::asm;
use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use embedded_sdmmc as sd;
use hal::gpio::Pins;
use hal::pac;
use pac::Peripherals;
use panic_halt as _;
mod sd_interface;
mod cdc;

#[entry]
fn main() -> ! {
    asm::nop(); // To not have main optimize to abort in release mode, remove when you add code

    let mut peripherals = Peripherals::take().unwrap();
    let p2 = cortex_m::peripheral::Peripherals::take().unwrap();
    let pins = Pins::new(peripherals.PORT);
    let mut led = pins.pa14.into_push_pull_output();

    /* Setup clocks */
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
    let gclk2 = clock
        .get_gclk(pac::gclk::pchctrl::GEN_A::GCLK2)
        .expect("Could not get gclk 2.");

    let _uart_clk = clock
        .sercom5_core(&gclk2)
        .expect("Could not configure sercom 5 clock.");

    clock.configure_gclk_divider_and_source(
        pac::gclk::pchctrl::GEN_A::GCLK3,
        3,
        pac::gclk::genctrl::SRC_A::DFLL,
        false,
    ); // 16MHz clock
    let gclk3 = clock
        .get_gclk(pac::gclk::pchctrl::GEN_A::GCLK3)
        .expect("Cannot get gclk 3.");
    let spi_clk = clock
        .sercom1_core(&gclk3)
        .expect("Cannot configure sercom 1 clock.");

    /* Create delay */
    let mut delay = hal::delay::Delay::new(p2.SYST, &mut clock);

    /* Setup peripherals */
    let mut micro_sd = sd_interface::SdInterface::new(
        &peripherals.MCLK, 
        peripherals.SERCOM1, 
        spi_clk, 
        pins.pa18.into_push_pull_output(),
        pins.pa17.into_push_pull_output(),
        pins.pa19.into_push_pull_output(),
        pins.pa16.into_push_pull_output()
    );
    let mut test_file = micro_sd.open_file("testing.txt");
    micro_sd.write(&mut test_file, b"Hello this is a test!");
    micro_sd.close_file(test_file); // we are done with the file so destroy it
    micro_sd.close(); // we are done our test so destroy


    /* Start RTC config */
    let start_time = hal::rtc::Datetime {
        seconds: 0,
        minutes: 0,
        hours: 0,
        day: 6,
        month: 12,
        year: 52,
    };
    let mut rtc = hal::rtc::Rtc::clock_mode(peripherals.RTC, 1024.hz(), &mut peripherals.MCLK);
    hal::rtc::Rtc::set_time(&mut rtc, start_time);

    loop {
        led.set_high().unwrap();
        delay.delay_ms(1000_u16);
        info!("Test");
        led.set_low().unwrap();
        delay.delay_ms(1000_u16);
    }
}
