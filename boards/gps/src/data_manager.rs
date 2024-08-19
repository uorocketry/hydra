use defmt::info;
use messages::command::RadioRate;
use messages::state::StateData;
use messages::Message;

#[derive(Clone)]
pub struct DataManager {
    pub state: Option<StateData>,
}

impl DataManager {
    pub fn new() -> Self {
        Self {
            state: None,
        }
    }

    pub fn handle_command(&mut self, command: Message) {
        info!("Handling command");
        match command.data {
            messages::Data::Command(command) => match command.data {
                messages::command::CommandData::RadioRateChange(command_data) => {
                }
                messages::command::CommandData::DeployDrogue(_) => {}
                messages::command::CommandData::DeployMain(_) => {}
                messages::command::CommandData::PowerDown(_) => {}
                messages::command::CommandData::Online(_) => {}
            },
            _ => {}
        }
    }
    pub fn handle_data(&mut self, data: Message) {
        match data.data {
            messages::Data::State(state) => {
                self.state = Some(state.data);
            }
            _ => {}
        }
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
