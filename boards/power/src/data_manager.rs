// import messages
use messages::Message;

pub struct DataManager {}

impl DataManager {
    pub fn new() -> Self {
        Self {}
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
