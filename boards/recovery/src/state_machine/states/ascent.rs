use crate::state_machine::states::apogee::Apogee;
use crate::state_machine::states::wait_for_takeoff::WaitForTakeoff;
use crate::state_machine::{RocketStates, State, StateMachineContext, TransitionInto};
use crate::{no_transition, transition};
use defmt::{write, Format, Formatter};
use rtic::mutex::Mutex;

#[derive(Debug, Clone)]
pub struct Ascent {}

impl State for Ascent {
    fn step(&mut self, context: &mut StateMachineContext) -> Option<RocketStates> {
        context
            .shared_resources
            .data_manager
            .lock(|data| {
                if data.is_falling() {
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
