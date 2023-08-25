use super::TerminalDescent;
use crate::app::monotonics;
use crate::state_machine::{RocketStates, State, StateMachineContext, TransitionInto};
use crate::types::COM_ID;
use crate::{no_transition, transition};
use rtic::mutex::Mutex;
use defmt::{write, Format, Formatter, info};
use messages::command::{Command, CommandData, PowerDown, RadioRateChange, RadioRate};
use messages::Message;
use messages::sender::Sender::SensorBoard;

#[derive(Debug, Clone)]
pub struct WaitForRecovery {}

impl State for WaitForRecovery {
    fn enter(&self, context: &mut StateMachineContext) {
        // send a command over CAN to shut down non-critical systems for recovery. 
        for i in 0..10 {
            let sensor_power_down = PowerDown {
                board: SensorBoard
            };
            let radio_rate_change = RadioRateChange {
                rate: RadioRate::Slow,
            };
            let message = Message::new(&monotonics::now(), COM_ID, Command::new(sensor_power_down));
            let message_com = Message::new(&monotonics::now(), COM_ID, Command::new(radio_rate_change));
            context.shared_resources.can0.lock(|can| {
                context.shared_resources.em.run(||{
                    can.send_message(message)?;
                    can.send_message(message_com)?;
                    Ok(())
                })
            });  
        }
    }
    fn step(&mut self, context: &mut StateMachineContext) -> Option<RocketStates> {
        no_transition!() // this is our final resting place. We should also powerdown this board. 
    }
}

impl TransitionInto<WaitForRecovery> for TerminalDescent {
    fn transition(&self) -> WaitForRecovery {
        WaitForRecovery {}
    }
}

impl Format for WaitForRecovery {
    fn format(&self, f: Formatter) {
        write!(f, "Descent")
    }
}
