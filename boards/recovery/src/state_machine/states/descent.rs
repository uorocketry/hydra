use super::Ascent;
use crate::app::fire_drogue;
use crate::state_machine::{
    RocketStates, State, StateMachineContext, TerminalDescent, TransitionInto,
};
use crate::{no_transition, transition};
use common_arm::spawn;
use defmt::{write, Format, Formatter};
use rtic::mutex::Mutex;

#[derive(Debug, Clone)]
pub struct Descent {}

impl State for Descent {
    fn enter(&self, context: &mut StateMachineContext) {
        context.shared_resources.em.run(|| {
            spawn!(fire_drogue)?;
            Ok(())
        });
    }
    fn step(&mut self, context: &mut StateMachineContext) -> Option<RocketStates> {
        context.shared_resources.data_manager.lock(|data| {
            if data.is_below_main() {
                transition!(self, TerminalDescent)
            } else {
                no_transition!()
            }
        })
    }
}

impl TransitionInto<Descent> for Ascent {
    fn transition(&self) -> Descent {
        Descent {}
    }
}

impl Format for Descent {
    fn format(&self, f: Formatter) {
        write!(f, "Descent")
    }
}
