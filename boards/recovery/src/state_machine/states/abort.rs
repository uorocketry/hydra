use crate::state_machine::{StateMachineContext, RocketStates, State, TransitionInto};
use crate::state_machine::states::initializing::Initializing;

#[derive(Debug)]
pub struct Abort {}

impl State for Abort {
    fn step(&mut self, data: &StateMachineContext) -> Option<RocketStates> {
        todo!()
    }
}

impl TransitionInto<Abort> for Initializing {
    fn transition(&self) -> Abort {
        Abort {}
    }
}