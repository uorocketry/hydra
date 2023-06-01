use common_arm::sfsm::*;
use defmt::info;

pub struct WaitForLaunch {
    pub accel_y: f32, // When we pull an insane amount of Gs, it's pretty obvious that we're launching
    pub velocity_y: f32,
    pub altitude: f32,
    pub pressure: f32,
    pub previous_pressure: f32,
}
pub struct WaitForRecovery {
    pub velocity_y: f32,
}
pub struct Ascent {
    pub previous_pressure: f32,
    pub pressure: f32, // When pressure and or altitude do not increase, we're at apogee
    pub altitude: f32,
    pub velocity_y: f32,
}
pub struct Descent {
    pub velocity_y: f32, // When we are at a certain altitude, we need to deploy the main parachute
    pub altitude: f32,
}

pub struct Landing {
    pub velocity_y: f32, 
}

pub struct Abort {
}

struct Status {
    velocity_y: f32,
    altitude: f32,
    pressure: f32,
    accel_y: f32,
}

add_messages!(
    LogicBoard,
    [
        Status -> WaitForLaunch, // send message into the wait for launch state
    ]
);

add_state_machine!(
    pub LogicBoard, // Name of the state machine
    WaitForLaunch, // Initial state
    [WaitForLaunch, Ascent, Descent, Landing, WaitForRecovery, Abort], // States
    [ // Transitions
        WaitForLaunch => Ascent,
        Ascent => Descent,
        Descent => Landing,
        Landing => WaitForRecovery,
        WaitForLaunch => Abort, // We can only abort before the launch
    ]
);

/// Questionable From implementation.
/// This will be changed in the future so that messages gets the state definitions for each board.
impl From<&LogicBoardStates> for messages::Status {
    fn from(state: &LogicBoardStates) -> Self {
        match state {
            LogicBoardStates::WaitForLaunchState(Some(_)) => messages::Status::Initializing,
            LogicBoardStates::AscentState(Some(_)) => messages::Status::Running,
            LogicBoardStates::AbortState(Some(_)) => messages::Status::Uninitialized,
            _ => messages::Status::Uninitialized,
        }
    }
}

impl State for WaitForLaunch {
    fn entry(&mut self) {
        info!("Entering WaitForLaunch");
    }
    fn execute(&mut self) {
        todo!();
    }
}

impl State for Ascent {
    fn entry(&mut self) {
        info!("Entering Ascent");
    }
    fn execute(&mut self) {
        info!("Executing Ascent");
    }
}

impl State for Descent {
    fn entry(&mut self) {
        info!("Entering Descent");
    }
    fn execute(&mut self) {
        info!("Executing Descent");
    }
}

impl State for Landing {
    fn entry(&mut self) {
        info!("Entering Landing");
    }
    fn execute(&mut self) {
        info!("Executing Landing");
    }
}

impl State for WaitForRecovery {
    fn entry(&mut self) {
        info!("Entering Standby");
    }
    fn execute(&mut self) {
        info!("Executing Standby");
    }
}

impl State for Abort {
    fn entry(&mut self) {
        info!("Entering Abort");
    }
    fn execute(&mut self) {
        info!("Executing Abort");
    }
}

impl Into<Ascent> for WaitForLaunch {
    fn into(self) -> Ascent {
        Ascent {
            previous_pressure: self.previous_pressure,
            pressure: self.pressure,
            altitude: self.altitude,
            velocity_y: self.velocity_y,
        }
    }
}

impl Transition<Ascent> for WaitForLaunch {
    fn guard(&self) -> TransitGuard {
        // Ideally 10 m/s^2 should be enough to detect a launch
        if self.accel_y > 10.0 {
            return TransitGuard::Transit;
        }
        TransitGuard::Remain
    }
}

impl Into<Abort> for WaitForLaunch {
    fn into(self) -> Abort {
        Abort {}
    }
}

impl Transition<Abort> for WaitForLaunch {
    fn guard(&self) -> TransitGuard {
        todo!();
    }
}

impl Into<Descent> for Ascent {
    fn into(self) -> Descent {
        Descent {
            velocity_y: self.velocity_y,
            altitude: self.altitude,
        }
    }
}

impl Transition<Descent> for Ascent {
    fn guard(&self) -> TransitGuard {
        // Questionable implementation
        if self.pressure - self.previous_pressure < -1.0 {
            return TransitGuard::Transit;
        }
        TransitGuard::Remain
    }
}

impl Into<Landing> for Descent {
    fn into(self) -> Landing {
        Landing {
            velocity_y: self.velocity_y,
        }
    }
}

impl Transition<Landing> for Descent {
    fn guard(&self) -> TransitGuard {
        // Will this be 450m Above Ground Level? 
        if self.altitude < 450.0 {
            return TransitGuard::Transit;
        }
        TransitGuard::Remain
    }
}

impl Into<WaitForRecovery> for Landing {
    fn into(self) -> WaitForRecovery {
        WaitForRecovery {
            velocity_y: self.velocity_y,
        }
    }
}

impl Transition<WaitForRecovery> for Landing {
    fn guard(&self) -> TransitGuard {
        if self.velocity_y < 1.0 {
            return TransitGuard::Transit;
        }
        TransitGuard::Remain
    }
}

impl ReceiveMessage<Status> for WaitForLaunch {
    fn receive_message(&mut self, message: Status) {
        self.previous_pressure = self.pressure;
        self.accel_y = message.accel_y;
        self.altitude = message.altitude;
        self.pressure = message.pressure;
        self.velocity_y = message.velocity_y;
    }
}

#[sfsm_trace]
fn trace(log: &str) {
    info!("{}", log);
}