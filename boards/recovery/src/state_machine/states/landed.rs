use defmt::{Format, write, Formatter};

use crate::state_machine::{StateMachineContext, RocketStates, State};

#[derive(Debug)]
pub struct Landed {}

impl State for Landed {
    fn step(&mut self, data: &mut StateMachineContext) -> Option<RocketStates> {
        todo!()
    }
}

impl Format for Landed {
    fn format(&self, f: Formatter) {
        write!(f, "Landed")
    }
}