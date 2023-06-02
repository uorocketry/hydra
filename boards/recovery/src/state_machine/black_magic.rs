use enum_dispatch::enum_dispatch;
use core::fmt::Debug;
use defmt::println;

use crate::state_machine::{RocketEvents, RocketStates, StateMachineContext};

/// Trait that all states implement. Ignore this, not super important
#[enum_dispatch]
pub trait State: Debug {
    fn enter(&self) {
        // the trait bound `Self: Format` is not satisfied
        // println!("Enter {:?}", self)
    }
    fn exit(&self) {
        // the trait bound `Self: Format` is not satisfied
        // println!("Exit {:?}", self)
    }
    fn event(&mut self, _event: RocketEvents) -> Option<RocketStates> {
        None
    }
    fn step(&mut self, context: &StateMachineContext) -> Option<RocketStates>;

}


/// Transition Trait
pub trait TransitionInto<T> {
    fn transition(&self) -> T;
}

#[macro_export]
macro_rules! transition {
    ($self:ident, $i:ident) => {
        Some(TransitionInto::<$i>::transition($self).into())
    }
}

#[macro_export]
macro_rules! no_transition {
    () => {
        None
    }
}