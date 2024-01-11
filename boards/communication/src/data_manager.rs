use defmt::info;
use messages::command::RadioRate;
use messages::state::StateData;
use messages::Message;
use heapless::HistoryBuffer;

// #[derive(Clone)]
pub struct DataManager {
    // I'd prefer a history buffer here, but clone trait not implemented. 
    pub sensors: HistoryBuffer<Option<Message>, 10>, // due the the match this will always be a sensor message. 
    pub state: Option<StateData>,
    pub logging_rate: Option<RadioRate>,
}

impl DataManager {
    pub fn new() -> Self {
        let mut sensors = HistoryBuffer::new();
        Self {
            // This will change to hold the last of 10 of a sensor message 
            sensors,
            state: None,
            logging_rate: Some(RadioRate::Slow), // start slow.
        }
    }

    pub fn get_logging_rate(&mut self) -> RadioRate {
        if let Some(rate) = self.logging_rate.take() {
            let rate_cln = rate.clone();
            self.logging_rate = Some(rate);
            return rate_cln;
        }
        self.logging_rate = Some(RadioRate::Slow);
        return RadioRate::Slow;
    }

    pub fn clone_states(&self) -> [Option<StateData>; 1] {
        [self.state.clone()]
    }
    pub fn handle_data(&mut self, data: Message) {
        match data.data {
            messages::Data::Sensor(_) => {
                // we will capture this in the error manager. 
                self.sensors.write(Some(data.clone()));
            },
            messages::Data::State(state) => {
                self.state = Some(state.data);
            }
            messages::Data::Command(command) => match command.data {
                messages::command::CommandData::RadioRateChange(command_data) => {
                    self.logging_rate = Some(command_data.rate);
                }
                _ => {}
            },
            _ => {
                info!("unkown");
            }
        }
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
