use enum_dispatch::enum_dispatch;
use core::fmt::Debug;
use defmt::{info, Format};
use crate::state_machine::{RocketEvents, RocketStates, StateMachineContext};

/// Trait that all states implement. Ignore this, not super important
#[enum_dispatch]
pub trait State: Debug {
    fn enter(&self) where Self: Format {
        // the trait bound `Self: Format` is not satisfied
        info!("Enter {:?}", self)
    }
    fn exit(&self) where Self: Format {
        // the trait bound `Self: Format` is not satisfied
        info!("Exit {:?}", self)
    }
    fn event(&mut self, _event: RocketEvents) -> Option<RocketStates> {
        None
    }
    fn step(&mut self, context: &mut StateMachineContext) -> Option<RocketStates>;

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