use defmt::Format;
use derive_more::From;
use serde::{Deserialize, Serialize};

#[cfg(test)]
use proptest_derive::Arbitrary;

#[cfg(feature = "ts")]
use ts_rs::TS;

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(test, derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
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
#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(test, derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum HealthState {
    Nominal,
    Warning,
    Error,
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(test, derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum HealthData {
    // RegulatorStatus(RegulatorStatus),
    HealthStatus(HealthStatus),
}

// #[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
// #[cfg_attr(test, derive(Arbitrary))]
// #[cfg_attr(feature = "ts", derive(TS))]
// #[cfg_attr(feature = "ts", ts(export))]
// pub struct RegulatorStatus {
//     pub v5_reg: bool,
//     pub v3_reg: bool,
// }

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(test, derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
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
