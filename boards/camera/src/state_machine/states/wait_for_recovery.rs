use super::TerminalDescent;
use crate::no_transition;
use crate::state_machine::{RocketStates, State, StateMachineContext, TransitionInto};
use defmt::{write, Format, Formatter};

#[derive(Debug, Clone)]
pub struct WaitForRecovery {}

impl State for WaitForRecovery {
    fn step(&mut self, _context: &mut StateMachineContext) -> Option<RocketStates> {
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
