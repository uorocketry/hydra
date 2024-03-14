use crate::sender::Sender;
use defmt::Format;
use derive_more::From;
use serde::{Deserialize, Serialize};

#[cfg(any(feature = "std", test))]
use proptest_derive::Arbitrary;

#[cfg(feature = "ts")]
use ts_rs::TS;

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct Command {
    pub data: CommandData,
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum CommandData {
    DeployDrogue(DeployDrogue),
    DeployMain(DeployMain),
    PowerDown(PowerDown),
    RadioRateChange(RadioRateChange),
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct DeployDrogue {
    pub val: bool,
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct DeployMain {
    pub val: bool,
    // Auth?
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct PowerDown {
    pub board: Sender, // This isn't proper naming !! This is the board to be powered down. Changes name of sender.rs to board.rs.
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct RadioRateChange {
    pub rate: RadioRate,
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum RadioRate {
    Fast,
    Slow,
}

impl Command {
    pub fn new(data: impl Into<CommandData>) -> Self {
        Command { data: data.into() }
    }
}
