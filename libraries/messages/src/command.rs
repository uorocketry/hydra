use derive_more::From;
use defmt::Format;
use serde::{Deserialize, Serialize};

#[cfg(feature = "ts")]
use ts_rs::TS;

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct Command {
    pub data: CommandData,
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum CommandData {
    DeployDrogue(DeployDrogue),
    DeployMain(DeployMain),
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct DeployDrogue {
    pub val: bool,
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct DeployMain {
    pub val: bool,
    // Auth? 
}

impl Command {
    pub fn new(data: impl Into<CommandData>) -> Self {
        Command {
            data: data.into(),
        }
    }
}