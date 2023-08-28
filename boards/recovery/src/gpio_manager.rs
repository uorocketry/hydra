use atsamd_hal::gpio::{Pin, PushPullOutput, PA09, PA06};
use atsamd_hal::prelude::*;

pub struct GPIOManager {
    main_ematch: Pin<PA06, PushPullOutput>,
    drogue_ematch: Pin<PA09, PushPullOutput>,
}

impl GPIOManager {
    pub fn new(mut main_ematch: Pin<PA06, PushPullOutput>, mut drogue_ematch: Pin<PA09, PushPullOutput>) -> Self {
        drogue_ematch.set_low().ok();
        main_ematch.set_low().ok();
        Self {
            main_ematch,
            drogue_ematch,
        }
    }
    pub fn fire_drogue(&mut self) {
        self.drogue_ematch.set_high().ok();
    }
    pub fn fire_main(&mut self) {
        self.main_ematch.set_high().ok();
    }
    pub fn close_drogue(&mut self) {
        self.drogue_ematch.set_low().ok();
    }
    pub fn close_main(&mut self) {
        self.main_ematch.set_low().ok();
    }
}