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

#[derive(Debug, Clone)]
pub struct Ascent {}

impl State for Ascent {
    fn enter(&self, context: &mut StateMachineContext) {
        for i in 0..10 {
            let radio_rate_change = RadioRateChange {
                rate: RadioRate::Fast,
            };
            let message_com = Message::new(&monotonics::now(), COM_ID, Command::new(radio_rate_change));
            context.shared_resources.can0.lock(|can| {
                context.shared_resources.em.run(||{
                    can.send_message(message_com)?;
                    Ok(())
                })
            });  
        }
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
