use atsamd_hal::gpio::{Pin, PushPullOutput, PA09, PA06};
use atsamd_hal::prelude::*;

pub struct GPIOManager {
    drogue_ematch: Pin<PA09, PushPullOutput>,
    main_ematch: Pin<PA06, PushPullOutput>,
}

impl GPIOManager {
    pub fn new(mut drogue_ematch: Pin<PA09, PushPullOutput>, mut main_ematch: Pin<PA06, PushPullOutput>) -> Self {
        drogue_ematch.set_low().ok();
        main_ematch.set_low().ok();
        Self {
            drogue_ematch,
            main_ematch
        }
    }
    pub fn fire_drogue(&mut self) {
        self.drogue_ematch.set_high().ok();
    }
    pub fn fire_main(&mut self) {
        self.main_ematch.set_high().ok();
    }
    pub fn close_drouge(&mut self) {
        self.drogue_ematch.set_low().ok();
    }
    pub fn close_main(&mut self) {
        self.main_ematch.set_low().ok();
    }
}