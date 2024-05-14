use heapless::{HistoryBuffer, Vec};
use messages::command::RadioRate;
use messages::state::StateData;
use messages::Message;
use messages::{
    MAX_COMMAND_SIZE, MAX_HEALTH_SIZE, MAX_LOG_SIZE, MAX_SENSOR_SIZE, MAX_SIZE, MAX_STATE_SIZE,
};
use postcard;

const MAX_RADIO_MSG: u8 = 255;

pub struct DataManager {
    pub message_queue: HistoryBuffer<Message, 32>,
    pub logging_rate: Option<RadioRate>,
    pub state: Option<StateData>,
}

impl DataManager {
    pub fn new() -> Self {
        Self {
            message_queue: HistoryBuffer::new(),
            logging_rate: Some(RadioRate::Slow), // start slow.
            state: None,
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

    pub fn stuff_messages(&mut self) -> Result<Vec<u8, 255>, postcard::Error> {
        let mut bytes: Vec<u8, 255> = Vec::new();
        for el in self.message_queue.oldest_ordered() {
            match el.data {
                messages::Data::Command(_) => {
                    if bytes.len() + MAX_COMMAND_SIZE <= MAX_SIZE {
                        bytes.extend(postcard::to_vec::<messages::Message, MAX_COMMAND_SIZE>(el)?);
                    } else {
                        break;
                    }
                }
                messages::Data::Health(_) => {
                    if bytes.len() + MAX_HEALTH_SIZE <= MAX_SIZE {
                        bytes.extend(postcard::to_vec::<messages::Message, MAX_HEALTH_SIZE>(el)?);
                    } else {
                        break;
                    }
                }
                messages::Data::Sensor(_) => {
                    if bytes.len() + MAX_SENSOR_SIZE <= MAX_SIZE {
                        bytes.extend(postcard::to_vec::<messages::Message, MAX_SENSOR_SIZE>(el)?);
                    } else {
                        break;
                    }
                }
                messages::Data::State(_) => {
                    if bytes.len() + MAX_STATE_SIZE <= MAX_SIZE {
                        bytes.extend(postcard::to_vec::<messages::Message, MAX_STATE_SIZE>(el)?);
                    } else {
                        break;
                    }
                }
                messages::Data::Log(_) => {
                    if bytes.len() + MAX_LOG_SIZE <= MAX_SIZE {
                        bytes.extend(postcard::to_vec::<messages::Message, MAX_LOG_SIZE>(el)?);
                    } else {
                        break;
                    }
                }
            }
        }
        if bytes.len() > 0 {
            return Ok(bytes);
        }
        return Err(postcard::Error::WontImplement);
    }

    pub fn handle_data(&mut self, data: Message) {
        match data.data {
            messages::Data::Command(command) => match command.data {
                messages::command::CommandData::RadioRateChange(command_data) => {
                    self.logging_rate = Some(command_data.rate);
                }
                messages::command::CommandData::DeployDrogue(_) => {}
                messages::command::CommandData::DeployMain(_) => {}
                messages::command::CommandData::PowerDown(_) => {}
            },
            _ => {
                self.message_queue.write(data);
            }
        }
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
