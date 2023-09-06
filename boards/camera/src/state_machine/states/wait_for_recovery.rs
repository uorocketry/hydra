use super::TerminalDescent;
use crate::app::monotonics;
use crate::state_machine::{RocketStates, State, StateMachineContext, TransitionInto};
use crate::types::COM_ID;
use crate::{no_transition, transition};
use rtic::mutex::Mutex;
use defmt::{write, Format, Formatter, info};
use messages::command::{Command, CommandData, PowerDown, RadioRateChange, RadioRate};
use messages::Message;
use messages::sender::Sender::SensorBoard;

#[derive(Debug, Clone)]
pub struct WaitForRecovery {}

impl State for WaitForRecovery {
    fn step(&mut self, context: &mut StateMachineContext) -> Option<RocketStates> {
        no_transition!() // this is our final resting place. We should also powerdown this board. 
    }
}

impl TransitionInto<WaitForRecovery> for TerminalDescent {
    fn transition(&self) -> WaitForRecovery {
        WaitForRecovery {}
    }
}

impl Format for WaitForRecovery {
    fn format(&self, f: Formatter) {
        write!(f, "Descent")
    }
}
