use super::Descent;
use crate::app::fire_main;
use crate::state_machine::{
    RocketStates, State, StateMachineContext, TransitionInto, WaitForRecovery,
};
use crate::{no_transition, transition};
use atsamd_hal::prelude::_embedded_hal_timer_CountDown;
use atsamd_hal::timer_traits::InterruptDrivenTimer;
use common_arm::spawn;
use defmt::{write, Format, Formatter};
use rtic::mutex::Mutex;

#[derive(Debug, Clone)]
pub struct TerminalDescent {}

impl State for TerminalDescent {
    fn enter(&self, context: &mut StateMachineContext) {
        context.shared_resources.em.run(|| {
            spawn!(fire_main)?;
            Ok(())
        });
        context.shared_resources.recovery_timer.lock(|timer| {
            timer.enable_interrupt();
            let duration_mins = atsamd_hal::fugit::MinutesDurationU32::minutes(1);
            // timer requires specific duration format
            let timer_duration: atsamd_hal::fugit::Duration<u32, 1, 1000000000> =
                duration_mins.convert();
            timer.start(timer_duration);
        });
    }
    fn step(&mut self, context: &mut StateMachineContext) -> Option<RocketStates> {
        context.shared_resources.data_manager.lock(|data| {
            if data.is_landed() {
                transition!(self, WaitForRecovery)
            } else {
                no_transition!()
            }
        })
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
