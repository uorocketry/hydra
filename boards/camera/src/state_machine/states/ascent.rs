use crate::state_machine::states::descent::Descent;
use crate::state_machine::states::wait_for_takeoff::WaitForTakeoff;
use crate::state_machine::{RocketStates, State, StateMachineContext, TransitionInto};
use crate::{no_transition, transition};
use messages::command::{Command, CommandData, PowerDown, RadioRateChange, RadioRate};
use messages::Message;
use defmt::{write, Format, Formatter};
use rtic::mutex::Mutex;
use crate::types::COM_ID;
use crate::app::monotonics;
use systick_monotonic::ExtU64;
use crate::app::toggle_cams;
use common_arm::spawn;

#[derive(Debug, Clone)]
pub struct Ascent {}

impl State for Ascent {
    fn enter(&self, context: &mut StateMachineContext) {
        spawn!(toggle_cams);
    }
    fn step(&mut self, context: &mut StateMachineContext) -> Option<RocketStates> {
        context
            .shared_resources
            .data_manager
            .lock(|data| {
                if data.is_falling() {
                    transition!(self, Descent)
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
