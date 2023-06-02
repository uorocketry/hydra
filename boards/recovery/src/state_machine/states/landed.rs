use crate::state_machine::{StateMachineContext, RocketStates, State};

#[derive(Debug)]
pub struct Landed {}

impl State for Landed {
    fn step(&mut self, data: &StateMachineContext) -> Option<RocketStates> {
        todo!()
    }
}