use defmt::{write, Format, Formatter, info};

use crate::no_transition;
use crate::state_machine::{RocketStates, State, StateMachineContext, TransitionInto};
use rtic::mutex::Mutex;
use super::Apogee;

#[derive(Debug, Clone)]
pub struct Landed {}

impl State for Landed {
    fn enter(&self,context: &mut StateMachineContext) {
        info!("Landed");
        context.shared_resources.gpio.lock(|gpio| gpio.fire_main());
    }
    fn step(&mut self, _context: &mut StateMachineContext) -> Option<RocketStates> {
        no_transition!()
    }
}

impl TransitionInto<Landed> for Apogee {
    fn transition(&self) -> Landed {
        Landed {}
    }
}

impl Format for Landed {
    fn format(&self, f: Formatter) {
        write!(f, "Landed")
    }
}
