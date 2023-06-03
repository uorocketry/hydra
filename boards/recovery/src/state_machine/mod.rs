mod black_magic;
mod states;

use core::fmt::Debug;
use defmt::Format;
use enum_dispatch::enum_dispatch;
use messages::sensor::SbgShort;
use crate::{GPIOController, RocketData};
use crate::state_machine::states::*;
use crate::communication::CanDevice0;
use rtic::Mutex;
pub use black_magic::*;

pub trait StateMachineSharedResources {
    fn lock_can(&mut self, f: &dyn Fn(&mut CanDevice0));
    fn lock_sbg_data(&mut self, f: &dyn Fn(&mut SbgShort));
    fn lock_gpio(&mut self, f: &dyn Fn(&mut GPIOController));
}

impl<'a> StateMachineSharedResources for crate::app::__rtic_internal_run_smSharedResources<'a> {
    fn lock_can(&mut self, fun: &dyn Fn(&mut CanDevice0)) {
        self.can0.lock(fun)
    }
    fn lock_sbg_data(&mut self, fun: &dyn Fn(&mut SbgShort)) {
        self.sbg_data.lock(fun)
    }
    fn lock_gpio(&mut self, fun: &dyn Fn(&mut GPIOController)) {
        self.gpio.lock(fun)
    }
}

pub struct StateMachineContext<'a> {
    pub shared_resources: &'a mut dyn StateMachineSharedResources,
}
pub struct StateMachine {
    state: RocketStates,
}

// Define some functions to interact with the state machine
impl StateMachine {
    pub fn new() -> Self {
        let state = Initializing {};
        state.enter();

        StateMachine {
            state: state.into(),
        }
    }

    pub fn run(&mut self, data: &mut StateMachineContext) {
        if let Some(new_state) = self.state.step(data) {
            self.state.exit();
            new_state.enter();
            self.state = new_state;
        }
    }

    // pub fn handle_event(&mut self) {
    //     self.state.event(event);
    // }
}

// All events are found here
pub enum RocketEvents {
    DeployDrogue,
    DeployMain,
}

// All states are defined here. Another struct must be defined for the actual state, and that struct
// must implement the State trait
#[enum_dispatch(State)]
#[derive(Debug, Format)]
pub enum RocketStates {
    Initializing,
    WaitForTakeoff,
    Ascent,
    Apogee,
    Landed,
    Abort
}

// impl Format for RocketStates {
//     fn format(&self, f: Formatter) {
//         match self {
//             RocketStates::Initializing(Initializing) => write!(f, "Initializing"),
//             RocketStates::WaitForTakeoff(WaitForTakeoff) => write!(f, "WaitForTakeoff"),
//             RocketStates::Ascent(Ascent) => write!("Ascent"),
//             RocketStates::Apogee(Apogee) => write!(f, "Apogee"),
//             RocketStates::Landed(Landed) => write!(f, "Landed"),
//             RocketStates::Abort(Abort) => write!(f, "Abort"),
//         }
//     }
// }