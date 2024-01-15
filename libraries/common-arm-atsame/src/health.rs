// use common_arm::HealthMonitorChannels;
// use atsamd_hal::adc::Adc;
// use embedded_hal::adc::{OneShot, Channel};

// /// Take control of the ADC pins here and use the digital pin trait to get any possible pins. 
// /// There could also be a more adc specific pin trait. 
// /// We also will need to take an ADC, however this may complicate things since some do not support the same pins. 
// struct HealthMonitorChannelsATSAME{
//     reader: Option<Box<dyn OneShot<ADC, Word, Pin>>>,
//     pin_3v3: Option<Box<dyn Channel<ADC>>>
// }

// impl HealthMonitorChannels for HealthMonitorChannelsATSAME {
//     fn get_3v3(&self) -> Option<u16> {
    
//         Some(0)
//     }
//     fn get_5v(&self) -> Option<u16> {
//         Some(0)
//     }
//     fn get_pyro(&self) -> Option<u16> {
//         Some(0)
//     }
//     fn get_vcc(&self) -> Option<u16> {
//         Some(0)
//     }
//     fn get_int_5v(&self) -> Option<u16> {
//         Some(0)
//     }
//     fn get_int_3v3(&self) -> Option<u16> {
//         Some(0)
//     }
//     fn get_ext_5v(&self) -> Option<u16> {
//         Some(0)
//     }
//     fn get_ext_3v3(&self) -> Option<u16> {
//         Some(0)
//     }
//     fn get_failover(&self) -> Option<u16> {
//         Some(0)
//     }
// }
