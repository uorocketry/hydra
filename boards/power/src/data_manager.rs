// import messages
use messages::sensor::{Current, Voltage, Regulator, Temperature, SensorData};
use messages::Message;

pub struct DataManager {
    pub regulator: [Option<Regulator>; 4],
    pub voltage: [Option<Voltage>; 4],
    pub current: Option<Current>,
    pub temperature: [Option<Temperature>; 4],
}

impl DataManager {
    pub fn new() -> Self {
        Self { 
            regulator: [None, None, None, None],
            voltage: [None, None, None, None],
            current: None,
            temperature: [None, None, None, None],
        }
    }
    pub fn clone_power(&self) -> [Option<SensorData>; 13] {
        [
            self.regulator[0].clone().map(|x| x.into()),
            self.regulator[1].clone().map(|x| x.into()),
            self.regulator[2].clone().map(|x| x.into()),
            self.regulator[3].clone().map(|x| x.into()),
            self.voltage[0].clone().map(|x| x.into()),
            self.voltage[1].clone().map(|x| x.into()),
            self.voltage[2].clone().map(|x| x.into()),
            self.voltage[3].clone().map(|x| x.into()),
            self.current.clone().map(|x| x.into()),
            self.temperature[0].clone().map(|x| x.into()),
            self.temperature[1].clone().map(|x| x.into()),
            self.temperature[2].clone().map(|x| x.into()),
            self.temperature[3].clone().map(|x| x.into()),
        ]
    }
    pub fn handle_data(&mut self, data: Message) {
        // to do 
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}