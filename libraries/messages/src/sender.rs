use defmt::Format;
use serde::{Deserialize, Serialize};

#[cfg(feature = "ts")]
use ts_rs::TS;

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum Sender {
    GroundStation,
    LogicBoard,
    RecoveryBoard,
    PowerBoard,
}

impl From<Sender> for u16 {
    fn from(sender: Sender) -> Self {
        match sender {
            Sender::GroundStation => 0,
            Sender::LogicBoard => 1,
            Sender::RecoveryBoard => 2,
            Sender::PowerBoard => 3,
        }
    }
}