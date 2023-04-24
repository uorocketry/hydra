use defmt::Format;
use derive_more::From;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
pub struct Sensor {
    pub component_id: u8,
    pub data: SensorData,
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
pub enum SensorData {
    Sbg(Sbg),
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
pub struct Sbg {
    pub accel: f32,
    pub speed: f32,
    pub pressure: f32,
    pub height: f32,
    pub roll: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub latitude: f64,
    pub longitude: f64,
}

impl Sensor {
    pub fn new(component_id: u8, data: impl Into<SensorData>) -> Self {
        Sensor {
            component_id,
            data: data.into(),
        }
    }
}
