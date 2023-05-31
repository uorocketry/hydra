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
    SbgShort(SbgShort),
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct SbgShort {
    pub accel_y: f32,
    pub pressure: f32,
    pub height: f64,
    pub quant_w: f32,
    pub quant_x: f32,
    pub quant_y: f32,
    pub quant_z: f32,
}

impl From<Sbg> for SbgShort {
    fn from(sbg: Sbg) -> Self {
        SbgShort {
            accel_y: sbg.accel_y,
            pressure: sbg.pressure,
            height: sbg.height,
            quant_w: sbg.quant_w,
            quant_x: sbg.quant_x,
            quant_y: sbg.quant_y,
            quant_z: sbg.quant_z,
        }
    }
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

#[cfg(test)]
mod test {
    use crate::sender::Sender::LogicBoard;
    use crate::sensor::{Sbg, Sensor};
    use crate::Message;
    use crate::MAX_SIZE;
    use fugit::Instant;
    use quickcheck::{Arbitrary, Gen};
    use quickcheck_macros::quickcheck;

    impl Arbitrary for Sbg {
        fn arbitrary(g: &mut Gen) -> Self {
            Sbg {
                accel_x: f32::arbitrary(g),
                accel_y: f32::arbitrary(g),
                accel_z: f32::arbitrary(g),
                velocity_n: f32::arbitrary(g),
                velocity_e: f32::arbitrary(g),
                pressure: f32::arbitrary(g),
                height: f64::arbitrary(g),
                roll: f32::arbitrary(g),
                yaw: f32::arbitrary(g),
                pitch: f32::arbitrary(g),
                latitude: f64::arbitrary(g),
                longitude: f64::arbitrary(g),
                quant_w: f32::arbitrary(g),
                quant_x: f32::arbitrary(g),
                quant_y: f32::arbitrary(g),
                velocity_d: f32::arbitrary(g),
                quant_z: f32::arbitrary(g),
            }
        }
    }

    #[quickcheck]
    fn sbg_size(component_id: u8, sbg: Sbg) -> postcard::Result<()> {
        let msg = Message::new(
            &Instant::<u64, 1, 1000>::from_ticks(0),
            MainBoard,
            Sensor::new(component_id, sbg),
        );
        let bytes = postcard::to_allocvec(&msg)?;
        assert!(dbg!(bytes.len()) <= MAX_SIZE);
        Ok(())
    }
}
