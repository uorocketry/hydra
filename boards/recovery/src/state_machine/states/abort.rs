use crate::state_machine::{StateMachineContext, RocketStates, State, TransitionInto};
use crate::state_machine::states::initializing::Initializing;
use defmt::{Format, write, Formatter};

#[derive(Debug)]
pub struct Abort {}

impl State for Abort {
    fn step(&mut self, data: &mut StateMachineContext) -> Option<RocketStates> {
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