use crate::state_machine::RocketStateMachineData;
use enum_dispatch::enum_dispatch;

// Use the one from `common-arm` once merged
pub struct HydraError {}

#[enum_dispatch]
pub trait State {
    fn enter(&self) {}
    fn exit(&self) {}
    fn event(&mut self, _event: RocketEvents) -> Result<Option<RocketStates>, HydraError> {
        Ok(None)
    }
    fn step(&mut self, data: &RocketStateMachineData) -> Result<Option<RocketStates>, HydraError>;
}

pub enum RocketEvents {
    DeployDrogue,
    DeployMain,
}

#[enum_dispatch(State)]
pub enum RocketStates {
    Initializing,
    WaitForTakeoff,
    Cruise,
    Coasting,
    Falling,
    Landed,
}

impl RocketStates {}

pub struct Initializing {}
pub struct WaitForTakeoff {}
pub struct Cruise {}
pub struct Coasting {}
pub struct Falling {}
pub struct Landed {}

impl State for Initializing {
    fn step(&mut self, data: &RocketStateMachineData) -> Result<Option<RocketStates>, HydraError> {
        todo!()
    }
}
impl State for WaitForTakeoff {
    fn step(&mut self, data: &RocketStateMachineData) -> Result<Option<RocketStates>, HydraError> {
        todo!()
    }
}
impl State for Cruise {
    fn step(&mut self, data: &RocketStateMachineData) -> Result<Option<RocketStates>, HydraError> {
        todo!()
    }
}
impl State for Coasting {
    fn step(&mut self, data: &RocketStateMachineData) -> Result<Option<RocketStates>, HydraError> {
        todo!()
    }
}
impl State for Falling {
    fn step(&mut self, data: &RocketStateMachineData) -> Result<Option<RocketStates>, HydraError> {
        todo!()
    }
}
impl State for Landed {
    fn step(&mut self, data: &RocketStateMachineData) -> Result<Option<RocketStates>, HydraError> {
        todo!()
    }
}
