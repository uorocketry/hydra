use derive_more::From;
use defmt::Format;
use serde::{Deserialize, Serialize};
use crate::sender::Sender;

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
    pub v5: u16,
    pub v3: u16,
    pub pyro_sense: u16,
    pub vcc_sense: u16,
    pub int_v5: u16,
    pub int_v3: u16,
    pub ext_v5: u16,
    pub failover_sense: u16,
}