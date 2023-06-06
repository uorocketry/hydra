use messages::sensor::{Power};

pub struct DataManager {
    pub power: Option<Power>,
}

impl DataManager {
    pub fn new() -> Self {
        Self {
            power: None,
        }
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}