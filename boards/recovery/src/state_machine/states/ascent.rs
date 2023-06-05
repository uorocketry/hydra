use core::cell::RefCell;

use crate::state_machine::states::apogee::Apogee;
use crate::state_machine::states::wait_for_takeoff::WaitForTakeoff;
use crate::state_machine::{RocketStates, State, StateMachineContext, TransitionInto};
use crate::{no_transition, transition};
use defmt::{write, Format, Formatter};
use messages::sensor::SbgShort;
use rtic::mutex::Mutex;

#[derive(Debug)]
pub struct Ascent {}

impl State for Ascent {
    fn step(&mut self, context: &mut StateMachineContext) -> Option<RocketStates> {
        // Gravity has overcome the force of the rocket, so we are now falling.
        context
            .shared_resources
            .sbg_data
            .lock(&|data: &mut SbgShort| {
                if data.accel_y < -9.8 {
                    transition!(self, Apogee)
                } else {
                    no_transition!()
                }
            })
    }
}

impl TransitionInto<Ascent> for WaitForTakeoff {
    fn transition(&self) -> Ascent {
        Ascent {}
    }
}

impl Format for Ascent {
    fn format(&self, f: Formatter) {
        write!(f, "Ascent")
    }
}
