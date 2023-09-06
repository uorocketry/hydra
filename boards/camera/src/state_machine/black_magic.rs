use crate::state_machine::{RocketEvents, RocketStates, StateMachineContext};
use core::fmt::Debug;
use defmt::{info, Format};
use enum_dispatch::enum_dispatch;

/// Trait that all states implement. Ignore this, not super important
#[enum_dispatch]
pub trait State: Debug {
    fn enter(&self, _context: &mut StateMachineContext)
    where
        Self: Format,
    {
        info!("Enter {:?}", self)
    }
    fn exit(&self)
    where
        Self: Format,
    {
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
    };
}

#[macro_export]
macro_rules! no_transition {
    () => {
        None
    };
}
