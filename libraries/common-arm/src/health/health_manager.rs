use crate::health::health_monitor::{HealthMonitor, HealthMonitorChannels};
use core::ops::RangeInclusive;
use defmt::warn;
use messages::health::HealthState;

/// Manages and reports the health of the system.
/// I think OutputPin trait is the right one that should be good for atsame and stm32.
pub struct HealthManager<T: HealthMonitorChannels> {
    pub monitor: HealthMonitor<T>,
}

impl<T> HealthManager<T>
where
    T: HealthMonitorChannels,
{
    /// Create a new health manager.
    /// Divider is the ohmage of the resistor and resolution is the resolution of the ADC.
    pub fn new(monitor: HealthMonitor<T>) -> Self {
        Self { monitor }
    }
    /// !! Values are not correct !!
    /// They need to be updated with the proper dividers
    /// Evaluate the health of the system.
    /// All values must be nominal for the system to be in a nominal state.
    /// could be nice to send out a log message if something fails, tell the system what failed.
    pub fn evaluate(&mut self) -> HealthState {
        self.monitor.update(); // We need new values before we can evaluate the health status.
        let data = self.monitor.data.clone();
        // This is ugly...
        let v5_status = get_status(data.v5, &self.monitor.range_5v);
        let v3_3_status = get_status(data.v3_3, &self.monitor.range_3v3);
        let ext_3v3 = get_status(data.ext_3v3, &self.monitor.range_ext_3v3);
        let ext_v5_status = get_status(data.ext_v5, &self.monitor.range_ext_5v);
        let int_v3_3_status = get_status(data.int_v3_3, &self.monitor.range_int_3v3);
        let int_5v_status = get_status(data.int_v5, &self.monitor.range_int_5v);
        let pyro_status = get_status(data.pyro_sense, &self.monitor.range_pyro);
        let vcc_status = get_status(data.vcc_sense, &self.monitor.range_vcc);
        let failover_status = get_status(data.failover_sense, &self.monitor.range_failover);
        let drogue_sense = get_status(data.drogue_sense, &self.monitor.range_drogue_sense);
        let main_sense = get_status(data.main_sense, &self.monitor.range_main_sense);

        for status in [
            v5_status,
            v3_3_status,
            ext_3v3,
            ext_v5_status,
            int_v3_3_status,
            int_5v_status,
            pyro_status,
            vcc_status,
            failover_status,
            drogue_sense,
            main_sense,
        ] {
            match status {
                HealthState::Error => return HealthState::Error,
                HealthState::Warning => return HealthState::Warning,
                _ => continue,
            }
        }

        HealthState::Nominal
    }
}

fn get_status(data: Option<u16>, nominal: &RangeInclusive<u16>) -> HealthState {
    return match data {
        Some(x) => {
            if !nominal.contains(&x) {
                warn!("Unsafe Voltage");
                HealthState::Error
            }
            else {
                HealthState::Nominal
            }
        },
        None => {
            warn!("No data");
            HealthState::Warning
        }
    }
}
