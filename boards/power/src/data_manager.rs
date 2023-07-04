use messages::sensor::Power;
use messages::sensor::SensorData;

#[derive(Clone)]
pub struct DataManager {
    pub power_data: Option<Power>,
}

impl DataManager {
    pub fn new() -> Self {
        Self {
            power_data: None
        }
    }
    pub fn clone_sensors(&self) -> [Option<SensorData>; 1] {
        [
            self.power_data.clone().map(|x| x.into())
        ]
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}