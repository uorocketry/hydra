//! Health related code
//! Would've liked to have this live in common-arm-atsame but the pins and adc are not standardised
//! for all boards which poses the problem of giving an adc to a wrong pin in a generic way.

use atsamd_hal::gpio::{Alternate, Pin, B, PB00, PB01, PB02, PB03, PB05, PB06, PB07, PB08, PB09};
use atsamd_hal::{adc::Adc, ehal::adc::OneShot, pac::ADC0, pac::ADC1};
use common_arm::{HealthMonitorChannels, SdManager};

// make sure to define the ADC types in types.rs

// I don't think this should own the ADC object, but rather when a call to evaluate is invoke it should be taken control
// and then released when the function returns. Refactor this later.
pub struct HealthMonitorChannelsCommunication {
    reader: Adc<ADC0>,
    reader1: Adc<ADC1>,
    sd_manager: SdManager,
    pin_3v3: Pin<PB01, Alternate<B>>,
    pin_5v: Pin<PB02, Alternate<B>>,
    pin_pyro: Pin<PB03, Alternate<B>>,
    pin_vcc: Pin<PB00, Alternate<B>>,
    pin_ext_3v3: Pin<PB06, Alternate<B>>,
    pin_ext_5v: Pin<PB07, Alternate<B>>,
    pin_int_5v: Pin<PB08, Alternate<B>>,
    pin_int_3v3: Pin<PB09, Alternate<B>>,
    pin_failover: Pin<PB05, Alternate<B>>,
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
        self.reader.read(&mut self.pin_int_5v).ok()
    }
    fn get_int_3v3(&mut self) -> Option<u16> {
        self.reader.read(&mut self.pin_int_3v3).ok()
    }
    fn get_ext_5v(&mut self) -> Option<u16> {
        self.reader1.read(&mut self.pin_ext_5v).ok()
    }
    fn get_ext_3v3(&mut self) -> Option<u16> {
        self.reader1.read(&mut self.pin_ext_3v3).ok()
    }
    fn get_failover(&mut self) -> Option<u16> {
        self.reader1.read(&mut self.pin_failover).ok()
    }
    fn get_sd_card_status(&mut self) -> Option<bool> {
        self.sd_manager.is_mounted()
    }
}

impl HealthMonitorChannelsCommunication {
    pub fn new(
        reader: Adc<ADC0>,
        reader1: Adc<ADC1>,
        sd_manager: SdManager,
        pin_3v3: Pin<PB01, Alternate<B>>,
        pin_5v: Pin<PB02, Alternate<B>>,
        pin_pyro: Pin<PB03, Alternate<B>>,
        pin_vcc: Pin<PB00, Alternate<B>>,
        pin_ext_3v3: Pin<PB06, Alternate<B>>,
        pin_ext_5v: Pin<PB07, Alternate<B>>,
        pin_int_5v: Pin<PB08, Alternate<B>>,
        pin_int_3v3: Pin<PB09, Alternate<B>>,
        pin_failover: Pin<PB05, Alternate<B>>,
    ) -> Self {
        HealthMonitorChannelsCommunication {
            reader,
            reader1,
            sd_manager,
            pin_3v3,
            pin_5v,
            pin_pyro,
            pin_vcc,
            pin_ext_3v3,
            pin_ext_5v,
            pin_int_5v,
            pin_int_3v3,
            pin_failover,
        }
    }
}
