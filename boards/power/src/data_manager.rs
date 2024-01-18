// import messages
use messages::sensor::{Current, Regulator, SensorData, Temperature, Voltage};
use messages::Message;

pub struct DataManager {
    // pub regulator: [Option<Regulator>; 4],
    // pub voltage: [Option<Voltage>; 4],
    // pub current: Option<Current>,
    // pub temperature: [Option<Temperature>; 4],
}

impl DataManager {
    pub fn new() -> Self {
        Self { 
            // regulator: [None, None, None, None],
            // voltage: [None, None, None, None],
            // current: None,
            // temperature: [None, None, None, None],
        }
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
