mod black_magic;
mod states;

use core::fmt::Debug;
use enum_dispatch::enum_dispatch;
use crate::{GPIOController, RocketData};
use crate::state_machine::states::*;
use crate::communication::CanDevice0;
use rtic::Mutex;
pub use black_magic::*;

trait StateMachineSharedResources {
    fn lock_can(&mut self, f: &dyn Fn(&mut CanDevice0));
}

impl<'a> StateMachineSharedResources for crate::app::run_sm::Context<'a> {
    fn lock_can(&mut self, fun: &dyn Fn(&mut CanDevice0)) {
        self.shared.can0.lock(fun)
    }
}

pub struct StateMachineContext<'a> {
    shared_resources: &'a mut dyn StateMachineSharedResources,
}

pub struct StateMachine {
    state: RocketStates,
}

// // The actual state machine that encapsulates a state
// pub struct RocketStateMachine {
//     state: RocketStates,
// }

// // A context that is passed around when executing the state machine
// pub struct RocketStateMachineContext<'a> {
//     pub(crate) data: &'a RocketData,
//     pub(crate) gpio: &'a GPIOController
// }

// Define some functions to interact with the state machine
impl StateMachine {
    pub fn new() -> Self {
        let state = Initializing {};
        state.enter();

        StateMachine {
            state: state.into(),
        }
    }

    pub fn run<T>(&mut self, data: &mut StateMachineContext) {
        if let Some(new_state) = self.state.step(data) {
            self.state.exit();
            new_state.enter();
            self.state = new_state;
        }
    }

    fn foo<T>(&self, context: &mut StateMachineContext) {
        context.shared_resources.lock_can(&|can| {

        });
    }
}

// All events are found here
pub enum RocketEvents {
    DeployDrogue,
    DeployMain,
}

// All states are defined here. Another struct must be defined for the actual state, and that struct
// must implement the State trait
#[enum_dispatch(State)]
#[derive(Debug)]
pub enum RocketStates {
    Initializing,
    WaitForTakeoff,
    Ascent,
    Apogee,
    Landed,
    Abort
}