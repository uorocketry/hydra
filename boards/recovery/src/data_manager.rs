use crate::state_machine::RocketStates;
use crate::app::{fire_drogue, fire_main};
use common_arm::spawn;
use heapless::HistoryBuffer;
use messages::sensor::{Air, EkfNav1, EkfNav2, EkfQuat, GpsVel, Imu1, Imu2, UtcTime};
use messages::Message;
use defmt::{info};

const MAIN_HEIGHT: f32 = 500.0;
const HEIGHT_MIN: f32 = 300.0;

pub struct DataManager {
    pub air: Option<Air>,
    pub ekf_nav: (Option<EkfNav1>, Option<EkfNav2>),
    pub ekf_quat: Option<EkfQuat>,
    pub imu: (Option<Imu1>, Option<Imu2>),
    pub utc_time: Option<UtcTime>,
    pub gps_vel: Option<GpsVel>,
    pub historical_barometer_altitude: HistoryBuffer<(f32, u32), 8>,
    pub current_state: Option<RocketStates>,
}

impl DataManager {
    pub fn new() -> Self {
        let historical_barometer_altitude = HistoryBuffer::new();
        Self {
            air: None,
            ekf_nav: (None, None),
            ekf_quat: None,
            imu: (None, None),
            utc_time: None,
            gps_vel: None,
            historical_barometer_altitude,
            current_state: None,
        }
    }
    /// Returns true if the rocket is descending 
    pub fn is_falling(&self) -> bool {
        // if let Some(air) = &self.air {
        //     if air.altitude < HEIGHT_MIN {
        //         return false 
        //     }
        // } else {
        //     return false;
        // }
        if self.historical_barometer_altitude.len() < 8 {
            return false;
        }
        let mut buf = self.historical_barometer_altitude.oldest_ordered();
        match buf.next() {
            Some(last) => {
                let mut avg_sum: f32 = 0.0;
                let mut prev = last;
                for i in buf {
                    let time_diff: f32 = (i.1 - prev.1) as f32 / 1_000_000.0;
                    if time_diff == 0.0 {
                        continue;
                    }
                    avg_sum += (i.0 - prev.0)/time_diff; 
                    prev = i;
                }
                match avg_sum / 7.0 { // 7 because we have 8 points.   
                    x if !(-100.0..=-2.0).contains(&x) => { 
                        info!("avg: {}", avg_sum / 7.0);
                        return false;
                    }
                    _ => {
                        info!("avg: {}", avg_sum / 7.0);
                    }
                }
            }
            None => {
                return false;
            }
        }
        true
    }
    pub fn is_launched(&self) -> bool {
        match self.air.as_ref() {
            Some(air) => air.altitude > HEIGHT_MIN,
            None => false,
        }
    }
    pub fn is_below_main(&self) -> bool {
        match self.air.as_ref() {
            Some(air) => air.altitude < MAIN_HEIGHT,
            None => false,
        }
    }
    pub fn get_alt(&self) -> f32 {
        match self.air.as_ref() {
            Some(air) => air.altitude,
            None => 0.0,
        }
    }
    pub fn set_state(&mut self, state: RocketStates) {
        self.current_state = Some(state);
    }
    pub fn handle_data(&mut self, data: Message) {
        match data.data {
            messages::Data::Sensor(sensor) => match sensor.data {
                messages::sensor::SensorData::Air(air_data) => {
                    let tup_data = (air_data.altitude, air_data.time_stamp);
                    self.air = Some(air_data);
                    if let Some(recent) = self.historical_barometer_altitude.recent() {
                        if recent.1 != tup_data.1 {
                            self.historical_barometer_altitude.write(tup_data);
                        } else {
                            info!("duplicate data {}", tup_data.1);
                        }
                    } else {
                        self.historical_barometer_altitude.write(tup_data);
                    }
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
            },
            messages::Data::Command(command) => match command.data {
                messages::command::CommandData::DeployDrogue(drogue) => {
                    spawn!(fire_drogue);
                },
                messages::command::CommandData::DeployMain(main) => {
                    spawn!(fire_main);
                }
            }
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
