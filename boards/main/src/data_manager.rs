use messages::sensor::{Sbg, SbgEkf, SbgNav};

pub struct DataManager {
    pub sbg: Option<Sbg>,
    pub sbg_nav: Option<SbgNav>,
    pub sbg_ekf: Option<SbgEkf>,
}

impl DataManager {
    pub fn new() -> Self {
        Self {
            sbg: None,
            sbg_nav: None,
            sbg_ekf: None,
        }
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
