use super::Ascent;
use crate::state_machine::{Landed, RocketStates, State, StateMachineContext, TransitionInto};
use crate::{no_transition, transition};
use rtic::mutex::Mutex;
use defmt::{write, Format, Formatter, info};

#[derive(Debug, Clone)]
pub struct Apogee {}

impl State for Apogee {
    fn enter(&self, context: &mut StateMachineContext) {
        info!("Apogee");
        context.shared_resources.gpio.lock(|gpio| gpio.fire_drogue());
    }
    fn step(&mut self, context: &mut StateMachineContext) -> Option<RocketStates> {
        // is this 450 AGL? I could put these types of values in a top file like
        // types.rs to make it easier to change.
        context.shared_resources.data_manager.lock(|data| {
            // Handle the case where we don't have any data yet
            if data.is_below_main() {
                transition!(self, Landed)
            } else {
                no_transition!()
            }
        })
    }
}

impl TransitionInto<Apogee> for Ascent {
    fn transition(&self) -> Apogee {
        Apogee {}
    }
}

impl Format for Apogee {
    fn format(&self, f: Formatter) {
        write!(f, "Apogee")
    }
}
