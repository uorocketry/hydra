use crate::node::Node;
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
    Online(Online),
}


// not really a command but this decreases the complexity of the change on ground station. 
#[common_derives]
pub struct Online {
    pub online: bool, 
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
    pub board: Node,
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
