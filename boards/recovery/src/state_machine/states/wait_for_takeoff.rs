use crate::state_machine::states::ascent::Ascent;
use crate::state_machine::states::initializing::Initializing;
use crate::state_machine::{RocketStates, State, StateMachineContext, TransitionInto};
use crate::{no_transition, transition};
use defmt::{write, Format, Formatter};
use rtic::mutex::Mutex;

#[derive(Debug, Clone)]
pub struct WaitForTakeoff {}

impl State for WaitForTakeoff {
    fn step(&mut self, context: &mut StateMachineContext) -> Option<RocketStates> {
        context.shared_resources.data_manager.lock(|data| {
            if data.is_launched() {
                transition!(self, Ascent)
            } else {
                no_transition!()
            }
        })
    }
}

impl TransitionInto<WaitForTakeoff> for Initializing {
    fn transition(&self) -> WaitForTakeoff {
        WaitForTakeoff {}
    }
}

impl Format for WaitForTakeoff {
    fn format(&self, f: Formatter) {
        write!(f, "WaitForTakeoff")
    }
}
