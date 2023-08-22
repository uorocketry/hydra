use common_arm::spawn;
use messages::sender::Sender;
use messages::sensor::{Air, EkfNav1, EkfNav2, EkfQuat, GpsVel, Imu1, Imu2, SensorData, UtcTime};
use messages::Message;
use crate::app::sleep_system;

#[derive(Clone)]
pub struct DataManager {
    pub air: Option<Air>,
    pub ekf_nav: Option<(EkfNav1, EkfNav2)>,
    pub ekf_quat: Option<EkfQuat>,
    pub imu: Option<(Imu1, Imu2)>,
    pub utc_time: Option<UtcTime>,
    pub gps_vel: Option<GpsVel>,
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
        }
    }

    pub fn clone_sensors(&self) -> [Option<SensorData>; 8] {
        [
            self.air.clone().map(|x| x.into()),
            self.ekf_nav.clone().map(|x| x.0.into()),
            self.ekf_nav.clone().map(|x| x.1.into()),
            self.ekf_quat.clone().map(|x| x.into()),
            self.imu.clone().map(|x| x.0.into()),
            self.imu.clone().map(|x| x.1.into()),
            self.utc_time.clone().map(|x| x.into()),
            self.gps_vel.clone().map(|x| x.into()),
        ]
    }

    pub fn handle_data(&mut self, data: Message) {
        match data.data {
            messages::Data::Command(command) => match command.data {
                messages::command::CommandData::PowerDown(info) => {
                    match info.board {
                        Sender::SensorBoard => {
                        // sleep the system.
                        spawn!(sleep_system);
                        }
                        _ => {
                            // don't care 
                        }
                    }
                }
                _ => {
                    // We don't care atm about these other commands. 
                }
            }
            _ => {
                // we can disregard all other messages for now. 
            }
        }
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
