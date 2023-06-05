



use crate::state_machine::states::ascent::Ascent;
use crate::state_machine::states::initializing::Initializing;
use crate::state_machine::{RocketStates, State, StateMachineContext, TransitionInto};
use crate::{no_transition, transition};
use rtic::mutex::Mutex;
use defmt::{write, Format, Formatter};

#[derive(Debug)]
pub struct WaitForTakeoff {}

impl State for WaitForTakeoff {
    // fn event(&mut self,_event:crate::state_machine::RocketEvents) -> Option<RocketStates> {

    // }

    fn step(&mut self, context: &mut StateMachineContext) -> Option<RocketStates> {
        // let flag = RefCell::new(false);
        context
            .shared_resources
            .data_manager
            .lock(|data| {
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
