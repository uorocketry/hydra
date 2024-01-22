use defmt::info;
use messages::command::RadioRate;
use messages::sensor::{
    Air, EkfNav1, EkfNav2, EkfQuat, GpsPos1, GpsPos2, GpsVel, Imu1, Imu2, SensorData, UtcTime,
};
use messages::state::StateData;
use messages::Message;

#[derive(Clone)]
pub struct DataManager {
    pub air: Option<Air>,
    pub ekf_nav: (Option<EkfNav1>, Option<EkfNav2>),
    pub ekf_quat: Option<EkfQuat>,
    pub imu: (Option<Imu1>, Option<Imu2>),
    pub utc_time: Option<UtcTime>,
    pub gps_vel: Option<GpsVel>,
    pub gps_pos: (Option<GpsPos1>, Option<GpsPos2>),
    pub state: Option<StateData>,
    pub logging_rate: Option<RadioRate>,
}

impl DataManager {
    pub fn new() -> Self {
        Self {
            air: None,
            ekf_nav: (None, None),
            ekf_quat: None,
            imu: (None, None),
            utc_time: None,
            gps_vel: None,
            gps_pos: (None, None),
            state: None,
            logging_rate: Some(RadioRate::Slow), // start slow.
        }
    }

    pub fn get_logging_rate(&mut self) -> RadioRate {
        if let Some(rate) = self.logging_rate.take() {
            let rate_cln = rate.clone();
            self.logging_rate = Some(rate);
            return rate_cln;
        }
        self.logging_rate = Some(RadioRate::Slow);
        return RadioRate::Slow;
    }

    pub fn clone_sensors(&self) -> [Option<SensorData>; 10] {
        [
            self.air.clone().map(|x| x.into()),
            self.ekf_nav.0.clone().map(|x| x.into()),
            self.ekf_nav.1.clone().map(|x| x.into()),
            self.ekf_quat.clone().map(|x| x.into()),
            self.imu.0.clone().map(|x| x.into()),
            self.imu.1.clone().map(|x| x.into()),
            self.utc_time.clone().map(|x| x.into()),
            self.gps_vel.clone().map(|x| x.into()),
            self.gps_pos.0.clone().map(|x| x.into()),
            self.gps_pos.1.clone().map(|x| x.into()),
        ]
    }
    pub fn clone_states(&self) -> [Option<StateData>; 1] {
        [self.state.clone()]
    }
    pub fn handle_data(&mut self, data: Message) {
        match data.data {
            messages::Data::Sensor(sensor) => match sensor.data {
                messages::sensor::SensorData::Air(air_data) => {
                    self.air = Some(air_data);
                }
                messages::sensor::SensorData::EkfNav1(nav1_data) => {
                    self.ekf_nav.0 = Some(nav1_data);
                }
                messages::sensor::SensorData::EkfNav2(nav2_data) => {
                    self.ekf_nav.1 = Some(nav2_data);
                }
                messages::sensor::SensorData::EkfQuat(quat_data) => {
                    self.ekf_quat = Some(quat_data);
                }
                messages::sensor::SensorData::GpsVel(gps_vel_data) => {
                    self.gps_vel = Some(gps_vel_data);
                }
                messages::sensor::SensorData::Imu1(imu1_data) => {
                    self.imu.0 = Some(imu1_data);
                }
                messages::sensor::SensorData::Imu2(imu2_data) => {
                    self.imu.1 = Some(imu2_data);
                }
                messages::sensor::SensorData::UtcTime(utc_time_data) => {
                    self.utc_time = Some(utc_time_data);
                }
                messages::sensor::SensorData::GpsPos1(gps) => {
                    self.gps_pos.0 = Some(gps);
                }
                messages::sensor::SensorData::GpsPos2(gps) => {
                    self.gps_pos.1 = Some(gps);
                }
                _ => {
                    info!("impl power related");
                }
            },
            messages::Data::State(state) => {
                self.state = Some(state.data);
            }
            messages::Data::Command(command) => match command.data {
                messages::command::CommandData::RadioRateChange(command_data) => {
                    self.logging_rate = Some(command_data.rate);
                }
                _ => {}
            },
            _ => {
                info!("unkown");
            }
        }
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
