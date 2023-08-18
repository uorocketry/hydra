use defmt::{write, Format, Formatter, info};

use crate::no_transition;
use crate::state_machine::{RocketStates, State, StateMachineContext, TransitionInto};
use rtic::mutex::Mutex;
use super::Descent;

#[derive(Debug, Clone)]
pub struct TerminalDescent {}

impl State for TerminalDescent {
    fn enter(&self,context: &mut StateMachineContext) {
        info!("Terminal Descent");
        context.shared_resources.gpio.lock(|gpio| gpio.fire_main());
    }
    fn step(&mut self, _context: &mut StateMachineContext) -> Option<RocketStates> {
        no_transition!()
    }
}

impl TransitionInto<TerminalDescent> for Descent {
    fn transition(&self) -> TerminalDescent {
        TerminalDescent {}
    }
}

impl Format for TerminalDescent {
    fn format(&self, f: Formatter) {
        write!(f, "Terminal Descent")
    }
}
