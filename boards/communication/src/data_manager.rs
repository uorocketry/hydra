use heapless::{HistoryBuffer, Vec};
use messages::command::RadioRate;
use messages::state::StateData;
use messages::Message;

const MAX_RADIO_MSG: u8 = 255;

#[derive(Clone)]
pub struct DataManager {
    pub message_queue: HistoryBuffer<Message, 32>,
    pub logging_rate: Option<RadioRate>,
}

impl DataManager {
    pub fn new() -> Self {
        Self {
            message_queue: HistoryBuffer::new(),
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

    pub fn stuff_messages(&mut self) -> Option<Vec<u8, 255>> {
        let mut bytes: Vec<u8, 255> = Vec::new();
        for el in self.message_queue.oldest_ordered() {
            bytes.extend_from_slice(el.to_bytes())
        }
        if bytes.len() > 0 {
            return Some(bytes);
        }
        None
    }

    pub fn clone_states(&self) -> [Option<StateData>; 1] {
        [self.state.clone()]
    }
    pub fn handle_data(&mut self, data: Message) {
        self.message_queue.write(data);
        match data.data {
            messages::Data::Command(command) => match command.data {
                messages::command::CommandData::RadioRateChange(command_data) => {
                    self.logging_rate = Some(command_data.rate);
                }
                messages::command::CommandData::DeployDrogue(_) => {}
                messages::command::CommandData::DeployMain(_) => {}
                messages::command::CommandData::PowerDown(_) => {}
            },
            _ => {}
        }
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
