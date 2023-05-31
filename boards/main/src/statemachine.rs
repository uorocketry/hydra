use common_arm::sfsm::*;
use defmt::info;

pub struct WaitForLaunch {
    pub malfunction: bool,
    pub launch_status: bool,
    pub countdown: u32,
    pub tries: u32,
    pub accel_y: f32, 
}
pub struct Standby {}
pub struct Ascent {}

pub struct Descent {}

pub struct Landing {}

pub struct Launch {}

pub struct Abort {
    tries: u32,
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
        Status -> Launch, // send message into the launch state
    ]
);

add_state_machine!(
    pub LogicBoard, // Name of the state machine
    WaitForLaunch, // Initial state
    [WaitForLaunch, Launch, Ascent, Descent, Landing, Standby,  Abort], // States
    [ // Transitions
        WaitForLaunch => Launch,
        Launch => Ascent,
        Ascent => Descent,
        Descent => Landing,
        Landing => Standby,
        WaitForLaunch => Abort, // We can only abort before the launch
    ]
);

/// Questionable From implementation.
/// This will be changed in the future so that messages gets the state definitions for each board.
impl From<&LogicBoardStates> for messages::Status {
    fn from(state: &LogicBoardStates) -> Self {
        match state {
            LogicBoardStates::WaitForLaunchState(Some(_)) => messages::Status::Initializing,
            LogicBoardStates::LaunchState(Some(_)) => messages::Status::Running,
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

impl State for Launch {
    fn entry(&mut self) {
        info!("Entering Launch");
    }
    fn execute(&mut self) {
        info!("Executing Launch");
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

impl State for Standby {
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
        self.tries += 1;
    }
    fn execute(&mut self) {
        info!("Aborting... try {}", self.tries);
    }
}

impl Into<Abort> for WaitForLaunch {
    fn into(self) -> Abort {
        Abort { tries: self.tries }
    }
}

impl Transition<Abort> for WaitForLaunch {
    fn guard(&self) -> TransitGuard {
        return self.malfunction.into();
    }
}

derive_transition_into!(Descent, Landing);
impl Transition<Landing> for Descent {
    fn guard(&self) -> TransitGuard {
        todo!();
    }
}

derive_transition_into!(Ascent, Descent);
impl Transition<Descent> for Ascent {
    fn guard(&self) -> TransitGuard {
        todo!();
    }
}

derive_transition_into!(Launch, Ascent);
impl Transition<Ascent> for Launch {
    fn guard(&self) -> TransitGuard {
        todo!();
    }
}

derive_transition_into!(Landing, Standby);
impl Transition<Standby> for Landing {
    fn guard(&self) -> TransitGuard {
        todo!();
    }
}

derive_transition_into!(WaitForLaunch, Launch);
impl Transition<Launch> for WaitForLaunch {
    fn guard(&self) -> TransitGuard {
        if self.countdown == 0 {
            return TransitGuard::Transit;
        }
        return TransitGuard::Remain;
    }
}

impl Into<WaitForLaunch> for Abort {
    fn into(self) -> WaitForLaunch {
        WaitForLaunch {
            countdown: 0,
            malfunction: false,
            launch_status: false,
            tries: self.tries,
            accel_y: 0.0,
        }
    }
}
derive_transition!(Abort, WaitForLaunch, TransitGuard::Transit);

impl ReceiveMessage<Status> for Launch {
    fn receive_message(&mut self, message: Status) {
        todo!();
    }
}

#[sfsm_trace]
fn trace(log: &str) {
    info!("{}", log);
}
