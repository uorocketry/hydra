//! Health related code
//! Would've liked to have this live in common-arm-atsame but the pins and adc are not standardised
//! for all boards which poses the problem of giving an adc to a wrong pin in a generic way.

use atsamd_hal::gpio::{Alternate, Pin, B, PA02, PB01, PB02, PB03, PB00};
use atsamd_hal::{adc::Adc, ehal::adc::OneShot, pac::ADC0, pac::ADC1};
use common_arm::HealthMonitorChannels;

// make sure to define the ADC types in types.rs

// I don't think this should own the ADC object, but rather when a call to evaluate is invoke it should be taken control
// and then released when the function returns.
struct HealthMonitorChannelsCommunication {
    reader: Adc<ADC0>,
    reader1: Adc<ADC1>,
    pin_3v3: Pin<PB01, Alternate<B>>,
    pin_5v: Pin<PB02, Alternate<B>>,
    pin_pyro: Pin<PB03, Alternate<B>>,
    pin_vcc: Pin<PB00, Alternate<B>>,
}

impl HealthMonitorChannels for HealthMonitorChannelsCommunication {
    fn get_3v3(&mut self) -> Option<u16> {
        self.reader.read(&mut self.pin_3v3).ok()
    }
    fn get_5v(&mut self) -> Option<u16> {
        self.reader.read(&mut self.pin_5v).ok()
    }
    fn get_pyro(&mut self) -> Option<u16> {
        self.reader.read(&mut self.pin_pyro).ok()
    }
    fn get_vcc(&mut self) -> Option<u16> {
        self.reader.read(&mut self.pin_vcc).ok()
    }
    fn get_int_5v(&mut self) -> Option<u16> {
        Some(0)
    }
    fn get_int_3v3(&mut self) -> Option<u16> {
        Some(0)
    }
    fn get_ext_5v(&mut self) -> Option<u16> {
        Some(0)
    }
    fn get_ext_3v3(&mut self) -> Option<u16> {
        Some(0)
    }
    fn get_failover(&mut self) -> Option<u16> {
        Some(0)
    }
}

impl HealthMonitorChannelsCommunication {
    fn new(
        reader: Adc<ADC0>,
        reader1: Adc<ADC1>,
        pin_3v3: Pin<PB01, Alternate<B>>,
        pin_5v: Pin<PB02, Alternate<B>>,
        pin_pyro: Pin<PB03, Alternate<B>>,
        pin_vcc: Pin<PB00, Alternate<B>>,
    ) -> Self {
        HealthMonitorChannelsCommunication {
            reader,
            reader1,
            pin_3v3,
            pin_5v,
            pin_pyro,
            pin_vcc,
        }
    }
}
