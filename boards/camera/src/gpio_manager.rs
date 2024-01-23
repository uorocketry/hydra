use atsamd_hal::gpio::{Pin, PushPullOutput, PA06, PA09};
use atsamd_hal::prelude::*;

pub struct GPIOManager {
    cam1: Pin<PA09, PushPullOutput>,
    cam2: Pin<PA06, PushPullOutput>,
}

impl GPIOManager {
    pub fn new(cam1: Pin<PA09, PushPullOutput>, cam2: Pin<PA06, PushPullOutput>) -> Self {
        Self { cam1, cam2 }
    }
    pub fn toggle_cam1(&mut self) {
        self.cam1.toggle().ok();
    }
    pub fn toggle_cam2(&mut self) {
        self.cam2.toggle().ok();
    }
}
