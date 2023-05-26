use common_arm::sfsm::*;
use defmt::info;

pub struct WaitForLaunch {
    pub malfunction: bool,
    pub countdown: u32,
    pub tries: u32,
}

pub struct Launch {}

pub struct Abort {tries: u32}

add_state_machine!(
    pub LogicBoard,
    WaitForLaunch,
    [WaitForLaunch, Launch, Abort],
    [
        WaitForLaunch => Launch,
        WaitForLaunch => Abort,
        Abort => WaitForLaunch,
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
        self.countdown = 3;
    }
    fn execute(&mut self) {
        info!("Seconds to launch {}", self.countdown);
        self.countdown -= 1; // count down
    }
}

impl Into<Abort> for WaitForLaunch {
    fn into(self) -> Abort {Abort {tries: self.tries}}
}

impl Transition<Abort> for WaitForLaunch {
    fn guard(&self) -> TransitGuard {
        return self.malfunction.into();
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

impl State for Abort {
    fn entry(&mut self) {
        info!("Entering Abort");
        self.tries += 1;
    }
    fn execute(&mut self) {
        info!("Aborting... try {}", self.tries);
    }
}
impl Into<WaitForLaunch> for Abort {
    fn into(self) -> WaitForLaunch {
        WaitForLaunch {
            countdown: 0,
            malfunction: false,
            tries: self.tries,
        }
    }
}
derive_transition!(Abort, WaitForLaunch, TransitGuard::Transit);

impl State for Launch {
    fn entry(&mut self) {
        info!("Entering Launch");
    }
}

#[sfsm_trace]
fn trace(log: &str) {
    info!("{}", log);
}

fn run_launch_sequence() -> Result<(), SfsmError> {
    let mut rocket = LogicBoard::new();

    let wait_for_launch = WaitForLaunch {
        tries: 0,
        malfunction: false,
        countdown: 0,
    };
    rocket.start(wait_for_launch)?;
    assert!(IsState::<WaitForLaunch>::is_state(&rocket));
    rocket.step()?;
    assert!(IsState::<WaitForLaunch>::is_state(&rocket));
    rocket.step()?;
    assert!(IsState::<Abort>::is_state(&rocket));
    rocket.step()?;
    assert!(IsState::<WaitForLaunch>::is_state(&rocket));
    rocket.step()?;
    assert!(IsState::<WaitForLaunch>::is_state(&rocket));
    rocket.step()?;
    assert!(IsState::<WaitForLaunch>::is_state(&rocket));
    rocket.step()?;
    assert!(IsState::<Launch>::is_state(&rocket));
    rocket.step()?;
    Ok(())
}