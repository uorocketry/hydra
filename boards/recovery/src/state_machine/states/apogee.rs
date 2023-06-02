use crate::state_machine::{StateMachineContext, RocketStates, State};

#[derive(Debug)]
pub struct Apogee {}

impl State for Apogee {
    fn step(&mut self, data: &StateMachineContext) -> Option<RocketStates> {
        todo!()
    }
}