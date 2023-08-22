use defmt::Format;
use serde::{Deserialize, Serialize};

#[cfg(test)]
use proptest_derive::Arbitrary;

#[cfg(feature = "ts")]
use ts_rs::TS;

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(test, derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct State {
    pub data: StateData,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(test, derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum StateData {
    Initializing,
    WaitForTakeoff,
    Ascent,
    Descent,
    TerminalDescent,
    WaitForRecovery,
    Abort,
}

impl State {
    pub fn new(data: impl Into<StateData>) -> Self {
        State {
            data: data.into(),
        }
    }
}