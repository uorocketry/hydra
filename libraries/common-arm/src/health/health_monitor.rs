/// Health monitor module
/// This may not actually be feasible.
/// So, we have a health monitor module that is responsible for monitoring the health of the system.
/// But if we want this to be generic to the point where we can use it on any board, then we need to
/// ensure that the ADC channels will accept the pins that we want to use.
use messages::health::HealthStatus;

/// Add a note about what each means and what the range is.
pub trait HealthMonitorChannels {
    fn get_5v(&mut self) -> Option<u16>;
    fn get_3v3(&mut self) -> Option<u16>;
    fn get_pyro(&mut self) -> Option<u16>;
    fn get_vcc(&mut self) -> Option<u16>;
    fn get_int_5v(&mut self) -> Option<u16>;
    fn get_int_3v3(&mut self) -> Option<u16>;
    fn get_ext_5v(&mut self) -> Option<u16>;
    fn get_ext_3v3(&mut self) -> Option<u16>;
    fn get_failover(&mut self) -> Option<u16>;
}

pub struct HealthMonitor<T: HealthMonitorChannels> {
    // we will store a set of readings here that can be propagated up to the manager
    // we also
    channels: T,
    pub data: HealthStatus,
}

impl<T> HealthMonitor<T>
where
    T: HealthMonitorChannels,
{
    pub fn new(channels: T) -> Self {
        Self {
            channels,
            data: HealthStatus {
                v5: None, 
                v3_3: None,
                pyro_sense: None,
                vcc_sense: None, 
                int_v5: None, 
                int_v3_3: None, 
                ext_v5: None, 
                ext_3v3: None, 
                failover_sense: None, 
            },
        }
    }

    /// Should this really be unwrap?
    /// Or should we return this to the manager and let it decide what to do with it?
    /// For now we will just unwrap it.
    /// Also update will be called by the manager.
    /// This could be intensive if we are reading all of the channels at once.
    pub fn update(&mut self) {
        self.data.v5 = self.channels.get_5v();
        self.data.v3_3 = self.channels.get_3v3();
        self.data.pyro_sense = self.channels.get_pyro();
        self.data.vcc_sense = self.channels.get_vcc();
        self.data.int_v5 = self.channels.get_int_5v();
        self.data.int_v3_3 = self.channels.get_int_3v3();
        self.data.ext_v5 = self.channels.get_ext_5v();
        self.data.ext_3v3 = self.channels.get_ext_3v3();
        self.data.failover_sense = self.channels.get_failover();
    }
}
