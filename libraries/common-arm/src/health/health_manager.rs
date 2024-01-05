use crate::health::health_monitor::{HealthMonitor, HealthMonitorChannels};
use defmt::warn;
use messages::health::HealthState;

/// Manages and reports the health of the system.
/// I think OutputPin trait is the right one that should be good for atsame and stm32.
pub struct HealthManager<T: HealthMonitorChannels> {
    monitor: HealthMonitor<T>,
}

impl<T> HealthManager<T>
where
    T: HealthMonitorChannels,
{
    pub fn new(monitor: HealthMonitor<T>) -> Self {
        Self { monitor }
    }
    /// !! Values are not correct !!
    /// They need to be updated with the proper dividers
    /// Evaluate the health of the system.
    /// All values must be nominal for the system to be in a nominal state.
    /// could be nice to send out a log message if something fails, tell the system what failed.
    pub fn evaluate(&mut self) -> HealthState {
        // what does a unhealthy state look like?
        // we can penalize the health status based on the readings that we get from the monitor
        self.monitor.update(); // We need new values before we can evaluate the health status.
        // let data = match self.monitor.data {
        //     messages::health::HealthData::HealthStatus(x) => x,
        // };
        let data = self.monitor.data.clone();

        // Do we assume all this nominal or everything is in error and prove otherwise?
        let mut status_array: [HealthState; 4] = [HealthState::Nominal,HealthState::Nominal,HealthState::Nominal,HealthState::Nominal];

        status_array[0] = match data.v3_3 {
            Some(x) => match x {
                45..=47 => {
                    warn!("3v3 warning");
                    HealthState::Warning
                }
                47..=53 => HealthState::Nominal,
                53..=55 => HealthState::Warning,
                _ => {
                    // we could invoke the error manager here.
                    HealthState::Error
                }
            },
            // return a warning because we cannot verify the state of the system.
            None => return HealthState::Warning
        };

        // I think this properly covers the range of values that we can get from the ADC.
        // if we are between 3 and 2.5 then we are in a warning state.
        // if we are between 3.5 and 3.7 then we are in a warning state.
        // between 3 and 3.5 is nominal.
        // otherwise we are in an error state. (either over or under voltage)
        status_array[1] = match data.v3_3 {
            Some(x) => match x {
                25..=30 => HealthState::Warning,
                30..=35 => HealthState::Nominal,
                35..=37 => HealthState::Warning,
                _ => HealthState::Error,
            },
            None => return HealthState::Warning
        };

        status_array[2] = match data.pyro_sense {
            Some(x) => match x {                
                25..=30 => HealthState::Warning,
                30..=35 => HealthState::Nominal,
                35..=37 => HealthState::Warning,
                _ => HealthState::Error,
            },
            None => return HealthState::Warning
        };

        status_array[3] = match data.vcc_sense {
            Some(x) => match x {
                25..=30 => HealthState::Warning,
                30..=35 => HealthState::Nominal,
                35..=37 => HealthState::Warning,
                _ => HealthState::Error,
            },
            None => return HealthState::Warning
        };

        // There are more to add
        // int_v5, int_3v3, ext_v5, ext_3v3, failover_sense

        let mut warning_flag = false;
        for x in status_array {
            match x {
                HealthState::Warning => warning_flag = true,
                HealthState::Error => return HealthState::Error,
                HealthState::Nominal => continue,
            }
        }

        if warning_flag {
            return HealthState::Warning;
        }

        HealthState::Nominal
    }
}
