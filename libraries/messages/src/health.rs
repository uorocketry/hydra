use derive_more::From;
use messages_proc_macros_lib::common_derives;

#[common_derives]
#[derive(From)]
pub struct Health {
    pub data: HealthData,
    pub status: HealthState,
}

/// The health status of a component.
/// Nomial: everything is fine  
/// Warning: something is wrong, but it's not critical
/// Error: something is wrong, and it's critical
/// We need some way to quantify this concept of good and bad health.
/// Could be current range or voltage range. If failover happens on the regulators that is warning.
#[common_derives]
#[derive(From)]
pub enum HealthState {
    Nominal,
    Warning,
    Error,
}

#[common_derives]
#[derive(From)]
pub enum HealthData {
    // RegulatorStatus(RegulatorStatus),
    HealthStatus(HealthStatus),
}

#[common_derives]
#[derive(From)]
pub struct HealthStatus {
    pub v5: Option<u16>,
    pub v3_3: Option<u16>,
    pub pyro_sense: Option<u16>,
    pub vcc_sense: Option<u16>,
    pub int_v5: Option<u16>,
    pub int_v3_3: Option<u16>,
    pub ext_v5: Option<u16>,
    pub ext_3v3: Option<u16>,
    pub failover_sense: Option<u16>,
}

impl Health {
    pub fn new(data: impl Into<HealthData>, status: impl Into<HealthState>) -> Self {
        Health {
            data: data.into(),
            status: status.into(),
        }
    }
}
