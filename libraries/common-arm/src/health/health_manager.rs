use crate::health::health_monitor::{HealthMonitor, HealthMonitorChannels};
use messages::health::HealthState;


// The idea was to not have to write the same code over and over again. 
// but I can't seem to create macros 
macro_rules! get_status {
    ($data:expr, $low:expr, $nominal:expr, $over:expr) => {
        match $data {
            Some(x) => match x {
                $low..=$nominal => {
                    warn!("Value under warning");
                    HealthState::Warning
                }
                $nominal => HealthState::Nominal,
                $nomial..=$over => {
                    warn!("Value over warning");
                    HealthState::Warning
                },
                _ => {
                    warn!("Value error");
                    HealthState::Error
                }
            },
            None => {
                warn!("No data");
                return HealthState::Warning
            }
        }
    };
}

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

        // These range values are not correct and need to be changed. 
        // status_array[0] = HealthManager::<T>::get_status(data.v3_3, 3, 4, 5); 
        // status_array[1] = HealthManager::<T>::get_status(data.pyro_sense, 3, 4, 5);
        // status_array[2] = HealthManager::<T>::get_status(data.vcc_sense, 3, 4, 5);
        // status_array[3] = HealthManager::<T>::get_status(data.failover_sense, 3, 4, 5);

        // status_array[0] = get_status!(data.v3_3, 3, 4, 5);

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

    // fn get_status(data: Option<u16>, low: u16, nominal: u16, over: u16) -> HealthState {
    //     match data {
    //         Some(x) => match x {
    //             low..=nominal => {
    //                 warn!("Value under warning");
    //                 HealthState::Warning
    //             }
    //             nominal => HealthState::Nominal,
    //             nominal..=over => {
    //                 warn!("Value over warning");
    //                 HealthState::Warning
    //             },
    //             _ => {
    //                 warn!("Value error");
    //                 HealthState::Error
    //             }
    //         },
    //         None => {
    //             warn!("No data");
    //             return HealthState::Warning
    //         }
    //     }
    // }

}
