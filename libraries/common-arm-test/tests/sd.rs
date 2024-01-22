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
        let (_, clocks, tokens) = atsamd_hal::clock::v2::clock_system_at_reset(
            peripherals.OSCCTRL,
            peripherals.OSC32KCTRL,
            peripherals.GCLK,
            peripherals.MCLK,
            &mut peripherals.NVMCTRL,
        );

        // SAFETY: Misusing the PAC API can break the system.
        // This is safe because we only steal the MCLK.
        let (_, _, _, mut mclk) = unsafe { clocks.pac.steal() };
        let gclk0 = clocks.gclk0;
        let (pclk_sd, gclk0) =
            atsamd_hal::clock::v2::pclk::Pclk::enable(tokens.pclks.sercom1, gclk0);
        let mut sd = SdInterface::new(
            &mclk,
            peripherals.SERCOM1,
            pclk_sd.freq(),
            pins.pa18.into_push_pull_output(),
            pins.pa17.into_push_pull_output(),
            pins.pa19.into_push_pull_output(),
            pins.pa16.into_push_pull_output(),
        );

        State { sd_interface: sd }
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
