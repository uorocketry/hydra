use atsamd_hal::gpio::{Pin, PushPullOutput, PB11, PB12};
use atsamd_hal::prelude::*;
use defmt::info;
pub struct GPIOManager {
    main_ematch: Pin<PB12, PushPullOutput>,
    drogue_ematch: Pin<PB11, PushPullOutput>,
}

impl GPIOManager {
    pub fn new(
        mut main_ematch: Pin<PB12, PushPullOutput>,
        mut drogue_ematch: Pin<PB11, PushPullOutput>,
    ) -> Self {
        drogue_ematch.set_low().ok();
        match drogue_ematch.set_low() {
            Ok(_) => {}
            Err(_) => {
                info!("Cannot set low");
            }
        }
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
