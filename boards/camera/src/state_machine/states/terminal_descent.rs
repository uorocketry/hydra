use defmt::{write, Format, Formatter};

use super::Descent;
use crate::state_machine::{
    RocketStates, State, StateMachineContext, TransitionInto, WaitForRecovery,
};
use crate::{no_transition, transition};
use rtic::mutex::Mutex;

#[derive(Debug, Clone)]
pub struct TerminalDescent {}

impl State for TerminalDescent {
    fn step(&mut self, context: &mut StateMachineContext) -> Option<RocketStates> {
        context.shared_resources.data_manager.lock(|data| {
            if data.is_landed() {
                transition!(self, WaitForRecovery)
            } else {
                no_transition!()
            }
        })
    }
}

impl TransitionInto<TerminalDescent> for Descent {
    fn transition(&self) -> TerminalDescent {
        TerminalDescent {}
    }
}

impl Format for TerminalDescent {
    fn format(&self, f: Formatter) {
        write!(f, "Terminal Descent")
    }
}
