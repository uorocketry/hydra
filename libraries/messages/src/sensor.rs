use defmt::Format;
use derive_more::From;
use serde::{Deserialize, Serialize};

#[cfg(feature = "ts")]
use ts_rs::TS;

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct Sensor {
    pub component_id: u8,
    pub data: SensorData,
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum SensorData {
    Sbg(Sbg),
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct Sbg {
    pub accel: f32,
    pub speed: f32,
    pub pressure: f32,
    pub height: f32,
}

impl Sensor {
    pub fn new(component_id: u8, data: impl Into<SensorData>) -> Self {
        Sensor {
            component_id,
            data: data.into(),
        }
    }
}
