use crate::app::sleep_system;
use common_arm::{spawn, HydraError};
use messages::sensor::{
    Air, EkfNav1, EkfNav2, EkfQuat, GpsPos1, GpsPos2, GpsVel, Imu1, Imu2, SensorData, UtcTime, EkfNavAcc, GpsPosAcc, GpsVelAcc,
};
use messages::Message;

#[derive(Clone)]
pub struct DataManager {
    pub air: Option<Air>,
    pub ekf_nav: Option<(EkfNav1, EkfNav2, EkfNavAcc)>,
    pub ekf_quat: Option<EkfQuat>,
    pub imu: Option<(Imu1, Imu2)>,
    pub utc_time: Option<UtcTime>,
    pub gps_vel: Option<(GpsVel, GpsVelAcc)>,
    pub gps_pos: Option<(GpsPos1, GpsPos2, GpsPosAcc)>,
}

impl DataManager {
    pub fn new() -> Self {
        Self {
            air: None,
            ekf_nav: None,
            ekf_quat: None,
            imu: None,
            utc_time: None,
            gps_vel: None,
            gps_pos: None,
        }
    }

    pub fn clone_sensors(&self) -> [Option<SensorData>; 13] {
        [
            self.air.clone().map(|x| x.into()),
            self.ekf_nav.clone().map(|x| x.0.into()),
            self.ekf_nav.clone().map(|x| x.1.into()),
            self.ekf_nav.clone().map(|x| x.2.into()),
            self.ekf_quat.clone().map(|x| x.into()),
            self.imu.clone().map(|x| x.0.into()),
            self.imu.clone().map(|x| x.1.into()),
            self.utc_time.clone().map(|x| x.into()),
            self.gps_vel.clone().map(|x| x.0.into()),
            self.gps_vel.clone().map(|x| x.1.into()),
            self.gps_pos.clone().map(|x| x.0.into()),
            self.gps_pos.clone().map(|x| x.1.into()),
            self.gps_pos.clone().map(|x| x.2.into()),
        ]
    }

    pub fn handle_data(&mut self, data: Message) -> Result<(), HydraError> {
        match data.data {
            messages::Data::Command(command) => match command.data {
                messages::command::CommandData::PowerDown(_) => {
                    spawn!(sleep_system)?; // need proper error handling. could just expect, but that is mal practice.
                }
                _ => {
                    // We don't care atm about these other commands.
                }
            },
            _ => {
                // we can disregard all other messages for now.
            }
        }
        Ok(())
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
