use messages::sensor::{Sbg, SbgShort};

pub struct DataManager {
    pub sbg: Option<Sbg>,
    pub sbg_short: Option<SbgShort>,
}

impl DataManager {
    pub fn new() -> Self {
        Self {
            sbg: None,
            sbg_short: None,
        }
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
