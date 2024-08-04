use super::TerminalDescent;
use crate::app::monotonics;
use crate::no_transition;
use crate::state_machine::{RocketStates, State, StateMachineContext, TransitionInto};
use crate::types::COM_ID;
use defmt::{write, Format, Formatter};
use messages::command::{Command, PowerDown, RadioRate, RadioRateChange};
use messages::sender::Sender::SensorBoard;
use messages::Message;
use rtic::mutex::Mutex;
use crate::RTC;

#[derive(Debug, Clone)]
pub struct WaitForRecovery {}

impl State for WaitForRecovery {
    fn enter(&self, context: &mut StateMachineContext) {
        // This should change to a ack and not be sent 10 times
        // send a command over CAN to shut down non-critical systems for recovery.
        for _i in 0..10 {
            let sensor_power_down = PowerDown { board: SensorBoard };
            let radio_rate_change = RadioRateChange {
                rate: RadioRate::Slow,
            };
            let message = Message::new(cortex_m::interrupt::free(|cs| {
                let mut rc = RTC.borrow(cs).borrow_mut();
                let rtc = rc.as_mut().unwrap();
                rtc.count32()}), COM_ID, Command::new(sensor_power_down));
            let message_com =
                Message::new(cortex_m::interrupt::free(|cs| {
                    let mut rc = RTC.borrow(cs).borrow_mut();
                    let rtc = rc.as_mut().unwrap();
                    rtc.count32()}), COM_ID, Command::new(radio_rate_change));
            context.shared_resources.can0.lock(|can| {
                context.shared_resources.em.run(|| {
                    can.send_message(message)?;
                    can.send_message(message_com)?;
                    Ok(())
                })
            });
        }
    }
    fn step(&mut self, _context: &mut StateMachineContext) -> Option<RocketStates> {
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
