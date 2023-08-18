mod black_magic;
mod states;

use messages::state;
use crate::communication::CanDevice0;
use crate::data_manager::DataManager;
use crate::state_machine::states::*;
use crate::GPIOController;
pub use black_magic::*;
pub use states::Initializing;
use core::fmt::Debug;
use defmt::Format;
use enum_dispatch::enum_dispatch;
use rtic::Mutex;

pub trait StateMachineSharedResources {
    fn lock_can(&mut self, f: &dyn Fn(&mut CanDevice0));
    fn lock_data_manager(&mut self, f: &dyn Fn(&mut DataManager));
    fn lock_gpio(&mut self, f: &dyn Fn(&mut GPIOController));
}

impl<'a> StateMachineSharedResources for crate::app::__rtic_internal_run_smSharedResources<'a> {
    fn lock_can(&mut self, fun: &dyn Fn(&mut CanDevice0)) {
        self.can0.lock(fun)
    }
    fn lock_data_manager(&mut self, fun: &dyn Fn(&mut DataManager)) {
        self.data_manager.lock(fun)
    }
    fn lock_gpio(&mut self, fun: &dyn Fn(&mut GPIOController)) {
        self.gpio.lock(fun)
    }
}

pub struct StateMachineContext<'a, 'b> {
    pub shared_resources: &'b mut crate::app::__rtic_internal_run_smSharedResources<'a>
}
pub struct StateMachine {
    state: RocketStates,
}

// Define some functions to interact with the state machine
impl StateMachine {
    pub fn new() -> Self {
        let state = Ascent {};

        StateMachine {
            state: state.into(),
        }
    }

    pub fn run(&mut self, context: &mut StateMachineContext) {
        if let Some(new_state) = self.state.step(context) {
            self.state.exit();
            context.shared_resources.data_manager.lock(|data| {
                data.set_state(new_state.clone());
            });
            new_state.enter(context);
            self.state = new_state;
        }
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
#[derive(Debug, Format, Clone)]
pub enum RocketStates {
    Initializing,
    WaitForTakeoff,
    Ascent,
    Descent,
    TerminalDescent,
    Abort,
}

// Not ideal, but it works for now.
// Should be able to put this is a shared library.
impl From<state::StateData> for RocketStates {
    fn from(state: state::StateData) -> Self {
        match state {
            state::StateData::Initializing => RocketStates::Initializing(Initializing {}),
            state::StateData::WaitForTakeoff => RocketStates::WaitForTakeoff(WaitForTakeoff {}),
            state::StateData::Ascent => RocketStates::Ascent(Ascent {}),
            state::StateData::Descent => RocketStates::Descent(Descent {}),
            state::StateData::TerminalDescent => RocketStates::TerminalDescent(TerminalDescent {  } ),
            state::StateData::Abort => RocketStates::Abort(Abort {}),
        }
    }
}

impl Into<state::StateData> for RocketStates {
    fn into(self) -> state::StateData {
        match self {
            RocketStates::Initializing(_) => state::StateData::Initializing,
            RocketStates::WaitForTakeoff(_) => state::StateData::WaitForTakeoff,
            RocketStates::Ascent(_) => state::StateData::Ascent,
            RocketStates::Descent(_) => state::StateData::Descent,
            RocketStates::TerminalDescent(_) => state::StateData::TerminalDescent,
            RocketStates::Abort(_) => state::StateData::Abort,
        }
    }
}