use embedded_hal::adc::OneShot;
use embedded_hal::digital::v2::OutputPin;

pub struct HealthMonitor {
    // we will store a set of readings here that can be propagated up to the manager
    // we also 
    channels: HealthMonitorChannels

}

/// Manages and reports the health of the system.
/// I think OutputPin trait is the right one that should be good for atsame and stm32. 
pub struct HealthMonitorChannels<T: OutputPin>
{
    sense_5v: Option<T>, 
    sense_3v: Option<T>
}
