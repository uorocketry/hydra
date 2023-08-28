use crate::state_machine::states::initializing::Initializing;
use crate::state_machine::{RocketStates, State, StateMachineContext, TransitionInto};
use defmt::{write, Format, Formatter};

#[derive(Debug, Clone)]
pub struct Abort {}

impl State for Abort {
    fn step(&mut self, _context: &mut StateMachineContext) -> Option<RocketStates> {
        todo!()
    }
}

impl TransitionInto<Abort> for Initializing {
    fn transition(&self) -> Abort {
        Abort {}
    }
}

impl Format for Abort {
    fn format(&self, f: Formatter) {
        write!(f, "Abort")
    }
}
