/*******************************************
 * SPI ADC thermocouple
 *******************************************/
use atsamd_hal as hal;
use atsamd_hal::prelude::*;
use embedded_hal::spi::MODE_0;
use hal::clock::Sercom1CoreClock;
use hal::gpio::{Pin, PinId, PinMode};
use hal::pac;
use hal::sercom::Sercom1;

use atsamd_hal::gpio::{Alternate, Output, PushPull, C, PA16, PA17, PA19};
use atsamd_hal::sercom::pad::IoSet1;
use atsamd_hal::sercom::spi;
use atsamd_hal::sercom::spi::BitOrder;
use nb::block;

use super::lookup_table;

/**
 * Thermocouple struct
 */
pub struct Thermocouple {
    pub thermo_controller: spi::Spi<
        spi::Config<
            spi::Pads<
                Sercom1,
                IoSet1,
                Pin<PA19, Alternate<C>>,
                Pin<PA16, Alternate<C>>,
                Pin<PA17, Alternate<C>>,
            >,
        >,
        spi::Duplex,
    >,
}

impl Thermocouple {
    /**
     * ### Constructor for Thermocouple module
     * `mclk`: reference to MCLK
     * `sercom`: reference to SERCOM1
     * `sck`: reference to SCK(Serial clock) pin
     * `cs`: reference to Chip Select pin
     * `miso`: reference to Master In Slave Out pin
     * `mosi`: reference to Master Out Slave In pin
     */
    pub fn new<I: PinId, M: PinMode>(
        mclk: &pac::MCLK,
        sercom: hal::sercom::Sercom1,
        spi_clk: Sercom1CoreClock,
        _cs: Pin<I, M>,
        sck: Pin<PA17, Output<PushPull>>,
        miso: Pin<PA19, Output<PushPull>>,
        mosi: Pin<PA16, Output<PushPull>>,
    ) -> Self {
        // 1. Configure pins
        let pads = spi::Pads::<hal::sercom::Sercom1, IoSet1>::default()
            .sclk(sck)
            .data_in(miso)
            .data_out(mosi);

        //2. instantiate SPI object
        let spi = spi::Config::new(&mclk, sercom, pads, spi_clk.freq())
            .baud(2.mhz())
            .length::<spi::lengths::U1>()
            .bit_order(BitOrder::LsbFirst)
            .spi_mode(MODE_0)
            .enable();

        //return SPI connection object to be used to communicate with the ADC
        Thermocouple {
            thermo_controller: spi,
        }
    }
}

impl Thermocouple {
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
                Thermocouple::binary_search(arr, value, mid + 1, upper)
            }
            // left side
            else {
                Thermocouple::binary_search(arr, value, lower, mid - 1)
            }
        }
    }

    //returns true if it can successfully communicate with the ADC by getting its ID
    pub fn test_connection(&mut self) -> bool {
        // 0x05 is the address to read the ID from
        self.thermo_controller.send(0x05).expect("SPI send failed.");

        let rcvd = block!(self.thermo_controller.read());

        let _ = match rcvd {
            Ok(_data) => return true,
            Err(_) => return false,
        };
    }

    pub fn send(&mut self, msg: u8) {
        block!(self.thermo_controller.send(msg)).expect("SPI send failed.");
    }

    pub fn read(&mut self) -> u8 {
        //read ADC for voltage
        // Address to be determined
        self.thermo_controller.send(0x00).expect("SPI send failed.");

        let rcvd = block!(self.thermo_controller.read()).expect("SPI receive failed.");

        return rcvd;
    }
}
