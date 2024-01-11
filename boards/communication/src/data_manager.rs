use defmt::info;
use messages::command::RadioRate;
use messages::state::StateData;
use messages::Message;
use messages::sensor::{
    Air, EkfNav1, EkfNav2, EkfQuat, GpsPos1, GpsPos2, GpsVel, Imu1, Imu2, SensorData, UtcTime,
};

#[derive(Clone)]
pub struct DataManager {
    // I'd prefer a history buffer here, but clone trait not implemented. 
    // pub sensors: HistoryBuffer<Option<Message>, 10>, // due the the match this will always be a sensor message. 
    pub air: Option<Message>,
    pub ekf_nav_1: Option<Message>,
    pub ekf_nav_2: Option<Message>,
    pub ekf_quat: Option<Message>,
    pub imu_1: Option<Message>, 
    pub imu_2: Option<Message>,
    pub utc_time: Option<Message>,
    pub gps_vel: Option<Message>,
    pub gps_pos_1: Option<Message>, 
    pub gps_pos_2: Option<Message>,
    pub state: Option<StateData>,
    pub logging_rate: Option<RadioRate>,
}

impl DataManager {
    pub fn new() -> Self {
        Self {
            air: None,
            ekf_nav_1: None,
            ekf_nav_2: None,
            ekf_quat: None,
            imu_1: None,
            imu_2: None,
            utc_time: None,
            gps_vel: None,
            gps_pos_1: None,
            gps_pos_2: None,
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

    pub fn clone_sensors(&self) -> [Option<Message>; 10] {
        [
            self.air.clone(),
            self.ekf_nav_1.clone(),
            self.ekf_nav_2.clone(),
            self.ekf_quat.clone(),
            self.imu_1.clone(),
            self.imu_2.clone(),
            self.utc_time.clone(),
            self.gps_vel.clone(),
            self.gps_pos_1.clone(),
            self.gps_pos_2.clone(),
        ]
    }

    pub fn clone_states(&self) -> [Option<StateData>; 1] {
        [self.state.clone()]
    }
    pub fn handle_data(&mut self, data: Message) {
        match data.data {
            messages::Data::Sensor(ref sensor) => match sensor.data {
                messages::sensor::SensorData::Air(_) => {
                    self.air = Some(data);
                }
                messages::sensor::SensorData::EkfNav1(_) => {
                    self.ekf_nav_1 = Some(data);
                }
                messages::sensor::SensorData::EkfNav2(_) => {
                    self.ekf_nav_2 = Some(data);
                }
                messages::sensor::SensorData::EkfQuat(_) => {
                    self.ekf_quat = Some(data);
                }
                messages::sensor::SensorData::GpsVel(_) => {
                    self.gps_vel = Some(data);
                }
                messages::sensor::SensorData::Imu1(_) => {
                    self.imu_1 = Some(data);
                }
                messages::sensor::SensorData::Imu2(_) => {
                    self.imu_2 = Some(data);
                }
                messages::sensor::SensorData::UtcTime(_) => {
                    self.utc_time = Some(data);
                }
                messages::sensor::SensorData::GpsPos1(_) => {
                    self.gps_pos_1 = Some(data);
                }
                messages::sensor::SensorData::GpsPos2(_) => {
                    self.gps_pos_2 = Some(data);
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
