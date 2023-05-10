#![no_std]
#![no_main]

use atsamd_hal as hal;
use hal::gpio::Pins;
use hal::pac;
use pac::Peripherals;

use defmt_rtt as _;
use panic_probe as _; // global logger

use common_arm::SdInterface;

struct State {
    sd_interface: SdInterface,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> State {
        let mut peripherals = Peripherals::take().unwrap();
        let pins = Pins::new(peripherals.PORT);

        let mut clock = hal::clock::GenericClockController::with_external_32kosc(
            peripherals.GCLK,
            &mut peripherals.MCLK,
            &mut peripherals.OSC32KCTRL,
            &mut peripherals.OSCCTRL,
            &mut peripherals.NVMCTRL,
        );

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

        let micro_sd = common_arm::SdInterface::new(
            &peripherals.MCLK,
            peripherals.SERCOM1,
            spi_clk,
            pins.pa18.into_push_pull_output(),
            pins.pa17.into_push_pull_output(),
            pins.pa19.into_push_pull_output(),
            pins.pa16.into_push_pull_output(),
        );

        State {
            sd_interface: micro_sd,
        }
    }

    #[test]
    fn writing_file(state: &mut State) {
        let sd_interface = &mut state.sd_interface;

        let mut test_file = sd_interface
            .open_file("testing.txt")
            .expect("Cannot open file");
        sd_interface
            .write(&mut test_file, b"Hello this is a test!")
            .expect("Could not write file.");
        sd_interface
            .write_str(&mut test_file, "Testing Strings")
            .expect("Could not write string");
        sd_interface
            .close_file(test_file)
            .expect("Could not close file."); // we are done with the file so destroy it
    }
}
