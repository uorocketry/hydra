use defmt::Format;
use serde::{Deserialize, Serialize};

#[cfg(feature = "ts")]
use ts_rs::TS;

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum Sender {
    GroundStation,
    MainBoard,
}
