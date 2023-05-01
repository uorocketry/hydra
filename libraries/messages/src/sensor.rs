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
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
    pub velocity_n: f32,
    pub velocity_e: f32,
    pub velocity_d: f32,
    pub pressure: f32,
    pub height: f64,
    pub roll: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub latitude: f64,
    pub longitude: f64,
    pub quant_w: f32,
    pub quant_x: f32,
    pub quant_y: f32,
    pub quant_z: f32,
}

impl Sensor {
    pub fn new(component_id: u8, data: impl Into<SensorData>) -> Self {
        Sensor {
            component_id,
            data: data.into(),
        }
    }
}
