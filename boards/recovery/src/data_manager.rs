use messages::sensor::{SbgShort};
use heapless::Vec;

pub struct DataManager {
    pub sbg_short: Option<SbgShort>,
    pub historical_pressure: Vec<f32, 5>, // probably want to make this a ring buffer
}

impl DataManager {
    pub fn new() -> Self {
        let mut historical_pressure = Vec::new();
        Self {
            sbg_short: None,
            historical_pressure,
        }
    }
    pub fn get_accel_y(&self) -> f32 {
        self.sbg_short.as_ref().unwrap().accel_y
    }
    pub fn is_falling(&self) -> bool { 
        for i in 1..self.historical_pressure.len() {
            let slope = self.historical_pressure[i] - self.historical_pressure[i-1];
            if slope > 0.0 {
                return false;
            }
        }
        true
    }
    pub fn is_launched(&self) -> bool {
        self.sbg_short.as_ref().unwrap().height > 10.0
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
