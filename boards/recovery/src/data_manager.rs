use crate::app::{fire_drogue, fire_main};
use crate::state_machine::RocketStates;
use common_arm::{spawn, HydraError};
use defmt::info;
use heapless::HistoryBuffer;
use messages::sensor::{Air, EkfNav1, EkfNav2, EkfQuat, GpsVel, Imu1, Imu2, UtcTime};
use messages::Message;

const MAIN_HEIGHT: f32 = 876.0; // meters ASL
const HEIGHT_MIN: f32 = 600.0; // meters ASL
const RECOVERY_DATA_POINTS: u8 = 8; // number of barometric altitude readings held by the recovery
                                    // algorithm
const RECOVERY_TIMER_TIMEOUT: u8 = 15; // minutes

pub struct DataManager {
    pub air: Option<Air>,
    pub ekf_nav: (Option<EkfNav1>, Option<EkfNav2>),
    pub ekf_quat: Option<EkfQuat>,
    pub imu: (Option<Imu1>, Option<Imu2>),
    pub utc_time: Option<UtcTime>,
    pub gps_vel: Option<GpsVel>,
    pub historical_barometer_altitude: HistoryBuffer<(f32, u32), 8>, // RECOVERY_DATA_POINTS (issue
    // when putting as const)
    pub current_state: Option<RocketStates>,
    // each tick represents a minute that passed
    pub recovery_counter: u8,
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
            recovery_counter: 0,
        }
    }
    /// Returns true if the rocket is descending
    pub fn is_falling(&self) -> bool {
        if (self.historical_barometer_altitude.len() as u8) < RECOVERY_DATA_POINTS {
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
                    let slope = (i.0 - prev.0) / time_diff;
                    if slope < -100.0 {
                        return false;
                    }
                    avg_sum += slope;
                    prev = i;
                }
                match avg_sum / (RECOVERY_DATA_POINTS as f32 - 1.0) {
                    // 7 because we have 8 points.
                    // exclusive range
                    x if !(-100.0..=-5.0).contains(&x) => {
                        return false;
                    }
                    _ => {
                        info!("avg: {}", avg_sum / (RECOVERY_DATA_POINTS as f32 - 1.0));
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
            Some(air) => match air.altitude {
                Some(altitude) => altitude > HEIGHT_MIN,
                None => false,
            },
            None => false,
        }
    }
    pub fn is_landed(&mut self) -> bool {
        if self.historical_barometer_altitude.len() < RECOVERY_DATA_POINTS.into() {
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
                    avg_sum += (i.0 - prev.0) / time_diff;
                    prev = i;
                }
                match avg_sum / (RECOVERY_DATA_POINTS as f32 - 1.0) {
                    // inclusive range
                    x if (-0.25..=0.25).contains(&x) => {
                        if self.recovery_counter >= RECOVERY_TIMER_TIMEOUT {
                            return true;
                        }
                    }
                    _ => {
                        self.recovery_counter = 0;
                    }
                }
            }
            None => {
                return false;
            }
        }
        false
    }
    pub fn is_below_main(&self) -> bool {
        match self.air.as_ref() {
            Some(air) => match air.altitude {
                Some(altitude) => altitude < MAIN_HEIGHT,
                None => false,
            },
            None => false,
        }
    }
    pub fn set_state(&mut self, state: RocketStates) {
        self.current_state = Some(state);
    }
    pub fn handle_data(&mut self, data: Message) -> Result<(), HydraError> {
        match data.data {
            messages::Data::Sensor(sensor) => match sensor.data {
                messages::sensor::SensorData::Air(air_data) => {
                    /*
                       NOTE!!!
                       There should be added a counter to check how many times
                       the alt is dropped, if the number is high switch to
                       the on board barometer.
                    */

                    if let Some(alt) = air_data.altitude {
                        let tup_data: (f32, u32) = (alt, air_data.time_stamp);
                        self.air = Some(air_data);
                        if let Some(recent) = self.historical_barometer_altitude.recent() {
                            if recent.1 != tup_data.1 {
                                self.historical_barometer_altitude.write(tup_data);
                            }
                        } else {
                            self.historical_barometer_altitude.write(tup_data);
                        }
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
                _ => {}
            },
            messages::Data::Command(command) => match command.data {
                messages::command::CommandData::DeployDrogue(_) => {
                    spawn!(fire_drogue)?; // need someway to capture this error.
                }
                messages::command::CommandData::DeployMain(_) => {
                    spawn!(fire_main)?;
                }
                messages::command::CommandData::PowerDown(_) => {
                    // don't handle for now.
                }
                _ => {}
            },
            _ => {}
        }
        Ok(())
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
