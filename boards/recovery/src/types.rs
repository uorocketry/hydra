use atsamd_hal::gpio::{PA18, PA19, Pin, PushPullOutput};
use atsamd_hal::prelude::*;
use messages::sender::Sender;
use messages::sender::Sender::RecoveryBoard;

// -------
// Sender ID
// -------
pub static COM_ID: Sender = RecoveryBoard;


pub struct GPIOController {
    drogue_ematch: Pin<PA18, PushPullOutput>,
    main_ematch: Pin<PA19, PushPullOutput>,
}

impl GPIOController {
    pub fn new(drogue_ematch: Pin<PA18, PushPullOutput>, main_ematch: Pin<PA19, PushPullOutput>) -> Self {
        Self {
            drogue_ematch,
            main_ematch,
        }
    }
    pub fn fire_drogue(&mut self) {
        self.drogue_ematch.set_high().ok();
    }
    pub fn fire_main(&mut self) {
        self.main_ematch.set_high().ok();
    }
}