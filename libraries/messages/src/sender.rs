use defmt::Format;
use serde::{Deserialize, Serialize};

#[cfg(test)]
use proptest_derive::Arbitrary;

#[cfg(feature = "ts")]
use ts_rs::TS;

#[derive(Serialize, Deserialize, Clone, Debug, Format, Copy)]
#[cfg_attr(test, derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum Sender {
    GroundStation,
    LogicBoard,
    RecoveryBoard,
}

impl From<Sender> for u16 {
    fn from(sender: Sender) -> Self {
        match sender {
            Sender::GroundStation => 0,
            Sender::LogicBoard => 1,
            Sender::RecoveryBoard => 2,
        }
    }
}
