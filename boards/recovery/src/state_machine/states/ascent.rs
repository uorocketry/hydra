use crate::state_machine::{StateMachineContext, RocketStates, State, TransitionInto};
use crate::state_machine::states::wait_for_takeoff::WaitForTakeoff;

#[derive(Debug)]
pub struct Ascent {}

impl State for Ascent {
    fn step(&mut self, data: &StateMachineContext) -> Option<RocketStates> {
        todo!()
    }
}

impl TransitionInto<Ascent> for WaitForTakeoff {
    fn transition(&self) -> Ascent {
        Ascent {}
    }
}