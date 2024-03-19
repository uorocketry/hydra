use crate::sender::Sender;
use derive_more::From;
use messages_proc_macros_lib::common_derives;

#[common_derives]
pub struct Command {
    pub data: CommandData,
}

#[common_derives]
#[derive(From)]
pub enum CommandData {
    DeployDrogue(DeployDrogue),
    DeployMain(DeployMain),
    PowerDown(PowerDown),
    RadioRateChange(RadioRateChange),
}

#[common_derives]
#[derive(From)]
pub struct DeployDrogue {
    pub val: bool,
}

#[common_derives]
#[derive(From)]
pub struct DeployMain {
    pub val: bool,
    // Auth?
}

#[common_derives]
#[derive(From)]
pub struct PowerDown {
    pub board: Sender, // This isn't proper naming !! This is the board to be powered down. Changes name of sender.rs to board.rs.
}

#[common_derives]
#[derive(From)]
pub struct RadioRateChange {
    pub rate: RadioRate,
}

#[common_derives]
#[derive(From)]
pub enum RadioRate {
    Fast,
    Slow,
}

impl Command {
    pub fn new(data: impl Into<CommandData>) -> Self {
        Command { data: data.into() }
    }
}
