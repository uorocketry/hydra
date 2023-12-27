/// Health monitor module
/// This may not actually be feasible. 
/// So, we have a health monitor module that is responsible for monitoring the health of the system.
/// But if we want this to be generic to the point where we can use it on any board, then we need to
/// ensure that the ADC channels will accept the pins that we want to use.


use embedded_hal::adc::OneShot;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::adc::Channel;
use messages::health::HealthData;

/// Add a note about what each means and what the range is.
pub trait HealthMonitorChannels {
    fn get_5v(&self) -> Option<u16>;
    fn get_3v(&self) -> Option<u16>;
    fn get_pyro(&self) -> Option<u16>;
    fn get_vcc(&self) -> Option<u16>;
    fn get_int_5v(&self) -> Option<u16>;
    fn get_int_3v(&self) -> Option<u16>;
    fn get_ext_5v(&self) -> Option<u16>;
    fn get_ext_3v(&self) -> Option<u16>;
    fn get_failover(&self) -> Option<u16>;
}

pub struct HealthMonitor<T: HealthMonitorChannels> {
    // we will store a set of readings here that can be propagated up to the manager
    // we also 
    channels: T,
    data: HealthData
}

impl HealthMonitor<T> where T: HealthMonitorChannels {
    pub fn new(channels: T) -> Self {
        Self {
            channels,
            data: HealthData::new()
        }
    }

    /// Should this really be unwrap?
    /// Or should we return this to the manager and let it decide what to do with it?
    /// For now we will just unwrap it.
    /// Also update will be called by the manager. 
    /// This could be intensive if we are reading all of the channels at once. 
    pub fn update(&self) {
        self.data.v5 = self.channels.get_5v().unwrap_or(0);
        self.data.v3 = self.channels.get_3v().unwrap_or(0);
        self.data.pyro_sense = self.channels.get_pyro().unwrap_or(0);
        self.data.vcc_sense = self.channels.get_vcc().unwrap_or(0);
        self.data.int_v5 = self.channels.get_int_5v().unwrap_or(0);
        self.data.int_v3 = self.channels.get_int_3v().unwrap_or(0);
        self.data.ext_v5 = self.channels.get_ext_5v().unwrap_or(0);
        self.data.failover_sense = self.channels.get_failover().unwrap_or(0);
    }
}