mod states;

use crate::state_machine::states::{HydraError, RocketStates};
use crate::state_machine::states::{Initializing, State};

pub struct RocketStateMachine {
    state: RocketStates,
}

pub struct RocketStateMachineData {
    // Things here like sensor data, some command manager, etc.
}

impl RocketStateMachine {
    pub fn new() -> Self {
        let state = Initializing {};
        state.enter();

        RocketStateMachine {
            state: state.into(),
        }
    }

    pub fn run(&mut self, data: &RocketStateMachineData) -> Result<(), HydraError> {
        if let Some(new_state) = self.state.step(data)? {
            self.state.exit();
            new_state.enter();
            self.state = new_state;
        }

        Ok(())
    }
}
