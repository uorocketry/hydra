use messages::sensor::{Voltage, Current, Temperature, Regulator};
use messages::sensor::SensorData;

#[derive(Clone)]
pub struct DataManager {
    pub reg_data: [Option<Regulator>; 4],
    pub voltage_data: [Option<Voltage>; 4],
    pub current_data: Option<Current>,
    pub temperature_data: [Option<Temperature>; 4]
}

impl DataManager {
    pub fn new() -> Self {
        Self {
            reg_data: [None, None, None, None], // complains about copy when trying [None; 4], weird. 
            voltage_data: [None, None, None, None],
            current_data: None,
            temperature_data: [None, None, None, None]
        }
    }
    pub fn clone_sensors(&self) -> [Option<SensorData>; 10] {
        [
            self.reg_data[0].clone().map(|x| x.into()),
            self.reg_data[1].clone().map(|x| x.into()),
            self.reg_data[2].clone().map(|x| x.into()),
            self.reg_data[3].clone().map(|x| x.into()),
            self.voltage_data[0].clone().map(|x| x.into()),
            self.voltage_data[1].clone().map(|x| x.into()),
            self.voltage_data[2].clone().map(|x| x.into()),
            self.voltage_data[3].clone().map(|x| x.into()),
            self.current_data.clone().map(|x| x.into()),
            self.temperature_data[0].clone().map(|x| x.into()),
        ]
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}