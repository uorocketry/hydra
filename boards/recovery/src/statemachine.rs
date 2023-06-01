use common_arm::sfsm::*;
use defmt::info;
use crate::app::{fire_drogue, fire_main};
use common_arm::spawn;

pub struct WaitForLaunch {
    pub accel_y: f32,
    pub pressure: f32,
    pub altitude: f32,
}
pub struct WaitForDrogue {
    pub pressure: f32,
    pub previous_pressure: f32,
    pub altitude: f32,
}

pub struct WaitForMain {
    pub altitude: f32,
}

pub struct WaitForRecovery {}

struct Status {
    velocity_y: f32,
    altitude: f32,
    pressure: f32,
    accel_y: f32,
}

add_messages!(
    RecoveryBoard,
    [
        Status -> WaitForLaunch,
        Status -> WaitForDrogue,
        Status -> WaitForMain,
    ]
);

add_state_machine!(
    pub RecoveryBoard, // Name of the state machine
    WaitForLaunch, // Initial state
    [WaitForLaunch, WaitForDrogue, WaitForMain, WaitForRecovery], // States
    [ // Transitions
        WaitForLaunch => WaitForDrogue,
        WaitForDrogue => WaitForMain,
        WaitForMain => WaitForRecovery,
    ]
);

impl State for WaitForLaunch {
    fn entry(&mut self) {
        info!("Entering WaitForLaunch");
    }
    fn execute(&mut self) {
        todo!();
    }
}

impl State for WaitForDrogue {
    fn entry(&mut self) {
        info!("Entering WaitForDrogue");
    }
    fn execute(&mut self) {
        todo!();
    }
}

impl State for WaitForMain {
    fn entry(&mut self) {
        info!("Entering WaitForMain");
    }
    fn execute(&mut self) {
        todo!();
    }
}

impl State for WaitForRecovery {
    fn entry(&mut self) {
        info!("Entering WaitForRecovery");
    }
    fn execute(&mut self) {
        todo!();
    }
}

impl Into<WaitForDrogue> for WaitForLaunch {
    fn into(self) -> WaitForDrogue {
        WaitForDrogue {
            pressure: self.pressure,
            previous_pressure: self.pressure, // our previous is our new
            altitude: self.altitude,
        }
    }
}

impl Transition<WaitForDrogue> for WaitForLaunch {
    fn guard(&self) -> TransitGuard {
        // Ideally 10 m/s^2 should be enough to detect a launch
        if self.accel_y > 10.0 {
            return TransitGuard::Transit;
        }
        TransitGuard::Remain
    }
}

impl Into<WaitForMain> for WaitForDrogue {
    fn into(self) -> WaitForMain {
        WaitForMain {
            altitude: self.altitude,
        }
    }
}

impl Transition<WaitForMain> for WaitForDrogue {
    fn guard(&self) -> TransitGuard {
        if self.pressure - self.previous_pressure < -1.0 {
            spawn!(fire_drogue).ok();
            return TransitGuard::Transit;
        }
        TransitGuard::Remain
    }
}

impl Into<WaitForRecovery> for WaitForMain {
    fn into(self) -> WaitForRecovery {
        WaitForRecovery {}
    }
}

impl Transition<WaitForRecovery> for WaitForMain {
    fn guard(&self) -> TransitGuard {
        if self.altitude > 450.0 {
            spawn!(fire_main).ok();
            return TransitGuard::Transit;
        }
        TransitGuard::Remain
    }
}

impl ReceiveMessage<Status> for WaitForLaunch {
    fn receive_message(&mut self, message: Status) {
        self.accel_y = message.accel_y;
        self.pressure = message.pressure;
        self.altitude = message.altitude;
    }
}

impl ReceiveMessage<Status> for WaitForDrogue {
    fn receive_message(&mut self, message: Status) {
        self.pressure = message.pressure;
        self.previous_pressure = self.pressure;
        self.altitude = message.altitude;
    }
}

impl ReceiveMessage<Status> for WaitForMain {
    fn receive_message(&mut self, message: Status) {
        self.altitude = message.altitude;
    }
}

#[sfsm_trace]
fn trace(log: &str) {
    info!("{}", log);
}
