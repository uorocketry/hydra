use crate::state_machine::{StateMachineContext, RocketStates, State, TransitionInto};
use crate::state_machine::states::wait_for_takeoff::WaitForTakeoff;
use defmt::{Format, write, Formatter};

#[derive(Debug)]
pub struct Ascent {}

impl State for Ascent {
    fn step(&mut self, data: &mut StateMachineContext) -> Option<RocketStates> {
        todo!()
    }
}

impl TransitionInto<Ascent> for WaitForTakeoff {
    fn transition(&self) -> Ascent {
        Ascent {}
    }
}

impl Format for Ascent {
    fn format(&self, f: Formatter) {
        write!(f, "Ascent")
    }
}