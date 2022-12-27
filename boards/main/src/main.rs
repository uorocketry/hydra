#![no_std]
#![no_main]

use atsamd_hal as hal;
use atsamd_hal::prelude::*;
use common_arm;
use cortex_m::asm;
use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use hal::gpio::Pins;
use hal::pac;
use pac::Peripherals;
use panic_halt as _;

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

    let mut delay = hal::delay::Delay::new(p2.SYST, &mut clock);

    /* Start UART CDC config */
    let uart_clk = clock
        .sercom5_core(&gclk2)
        .expect("Could not configure sercom 5 clock.");

    let pads = hal::sercom::uart::Pads::<hal::sercom::Sercom5, _>::default()
        .rx(pins.pb17)
        .tx(pins.pb16);
    let mut _uart = hal::sercom::uart::Config::new(
        &peripherals.MCLK,
        peripherals.SERCOM5,
        pads,
        uart_clk.freq(),
    )
    .baud(
        9600.hz(),
        hal::sercom::uart::BaudMode::Fractional(hal::sercom::uart::Oversampling::Bits16),
    )
    .enable();
    /* End UART CDC config */

    /* Start SPI config */
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

    let mut micro_sd = common_arm::SdInterface::new(
        &peripherals.MCLK,
        peripherals.SERCOM1,
        spi_clk,
        pins.pa18.into_push_pull_output(),
        pins.pa17.into_push_pull_output(),
        pins.pa19.into_push_pull_output(),
        pins.pa16.into_push_pull_output(),
    );
    /* Test peripherals */
    let mut test_file = micro_sd.open_file("testing.txt").expect("Cannot open file");
    micro_sd
        .write(&mut test_file, b"Hello this is a test!")
        .expect("Could not write file.");
    micro_sd
        .write_str(&mut test_file, "Testing Strings")
        .expect("Could not write string");
    micro_sd.close_file(test_file).expect("Could not close file."); // we are done with the file so destroy it
    micro_sd.close(); // we are done our test so destroy

    loop {
        led.set_high().unwrap();
        delay.delay_ms(1000_u16);
        info!("Test");
        led.set_low().unwrap();
        delay.delay_ms(1000_u16);
    }
}
