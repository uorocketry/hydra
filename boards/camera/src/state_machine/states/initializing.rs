use defmt::{write, Format, Formatter};

use crate::state_machine::states::wait_for_takeoff::WaitForTakeoff;
use crate::state_machine::{RocketStates, State, StateMachineContext, TransitionInto};
use crate::transition;

#[derive(Debug, Clone)]
pub struct Initializing {}

impl State for Initializing {
    fn step(&mut self, _context: &mut StateMachineContext) -> Option<RocketStates> {
        transition!(self, WaitForTakeoff)
    }
}

impl Format for Initializing {
    fn format(&self, f: Formatter) {
        write!(f, "Initializing")
    }
}
