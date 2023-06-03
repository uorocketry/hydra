use crate::state_machine::{StateMachineContext, RocketStates, State};
use defmt::{Format, write, Formatter};

#[derive(Debug)]
pub struct Apogee {}

impl State for Apogee {
    fn step(&mut self, data: &mut StateMachineContext) -> Option<RocketStates> {
        todo!()
    }
}

impl Format for Apogee {
    fn format(&self, f: Formatter) {
        write!(f, "Apogee")
    }
}