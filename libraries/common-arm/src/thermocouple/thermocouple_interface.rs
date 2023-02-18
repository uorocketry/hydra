/*******************************************
 * SPI ADC thermocouple
 *******************************************/
use atsamd_hal as hal;
use atsamd_hal::prelude::*;
use hal::gpio::{Pin, PinId, PinMode, Pins};
use hal::pac;
use pac::Peripherals;

use atsamd_hal::sercom::pad::IoSet1;
use atsamd_hal::sercom::spi::lengths::U2;
use atsamd_hal::sercom::spi::{BitOrder, MODE_1};
use atsamd_hal::sercom::{spi, Sercom0};
use nb::block;

use super::lookup_table;

/**
 * Thermocouple struct
 * pins:
 *  - SCK:
 *  - MISO:
 *  - MOSI:
 */
pub struct Thermocouple {
    pub thermo_controller: spi::Duplex,
}

impl Thermocouple {
    pub fn new<I: PinId, M: PinMode>(
        mclk: &pac::MCLK,
        sercom: pac::SERCOM1,
        sck: Pin<I, M>,
        cs: Pin<I, M>,
        miso: Pin<I, M>,
        mosi: Pin<I, M>,
    ) -> Self {
        // 1. Configure pins
        let pads = spi::Pads::<Sercom0, IoSet1>::default()
            .sclk(spi_clk)
            .data_in(miso)
            .data_out(mosi);

        //2. instantiate SPI object
        let mut spi = spi::Config::new(&mclk, sercom, pads, freq)
            .baud(2.mhz())
            .length::<U2>()
            .bit_order(BitOrder::LsbFirst)
            .spi_mode(MODE_1)
            .enable();

        //return SPI connection object to be used to communicate with the ADC
        Thermocouple {
            thermo_controller: spi,
        }
    }
}

impl Thermocouple {
    pub fn config(&mut self) {
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

        // 1. Configure pins
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
    }

    pub fn get_temperature(&mut self) -> f64 {
        return Thermocouple::convert_to_c(Thermocouple::get_voltage(self));
    }

    pub fn get_voltage(&mut self) -> f64 {
        //Sample implimentation given the right addresses to read from
        // Thermocouple::send(self, 0x0000);
        // Thermocouple::read(self);

        1.1 //TBD - read voltage from ADC
    }

    pub fn convert_to_c(mv: f64) -> f64 {
        Thermocouple::binary_search(&lookup_table::LOOKUP_TABLE, mv, 0, 1807)
    }
    pub fn binary_search(arr: &[[f64; 2]; 1808], value: f64, lower: usize, upper: usize) -> f64 {
        if lower > upper {
            -1.0
        } else {
            let mid: usize = (lower + upper) / 2;

            if value == arr[mid][0] {
                arr[mid][1]
            }
            // if value is really close to mid. when value is between mid and value next to it => Interpolate. https://www.wallstreetmojo.com/interpolation/
            else if value > arr[mid][0] && value < arr[mid + 1][0] {
                arr[mid][1]
                    + ((arr[mid + 1][1] - arr[mid][1]) / (arr[mid + 1][0] - arr[mid][0]))
                        * (value - arr[mid][0])
            }
            // right side
            else if value > arr[mid][0] {
                binary_search(arr, value, mid + 1, upper)
            }
            // left side
            else {
                binary_search(arr, value, lower, mid - 1)
            }
        }
    }

    pub fn send(&mut self, msg: u16) {
        block!(self.thermo_controller.send(msg)).expect("SPI send failed.");
    }

    pub fn read(&mut self) -> u16 {
        block!(self.thermo_controller.read()).expect("SPI receive failed.");
    }
}
