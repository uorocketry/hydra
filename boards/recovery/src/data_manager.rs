use crate::state_machine::RocketStates;
use crate::app::{fire_drogue, fire_main};
use common_arm::spawn;
use heapless::HistoryBuffer;
use messages::sensor::{Air, EkfNav1, EkfNav2, EkfQuat, GpsVel, Imu1, Imu2, UtcTime};
use messages::Message;
use defmt::{info};

const MAIN_HEIGHT: f32 = 500.0; // meters 
const HEIGHT_MIN: f32 = 300.0; // meters 

pub struct DataManager {
    pub air: Option<Air>,
    pub ekf_nav: (Option<EkfNav1>, Option<EkfNav2>),
    pub ekf_quat: Option<EkfQuat>,
    pub imu: (Option<Imu1>, Option<Imu2>),
    pub utc_time: Option<UtcTime>,
    pub gps_vel: Option<GpsVel>,
    pub historical_barometer_altitude: HistoryBuffer<(f32, u32), 8>,
    pub historical_pressure: HistoryBuffer<(f32, u32), 8>,
    pub current_state: Option<RocketStates>,
    pub rolling_avg_baro_alt: f32,
    pub rolling_avg_pres: f32,
    pub rolling_avg_baro_alt_n: f32,
    pub rolling_avg_pres_n: f32, 
}

impl DataManager {
    pub fn new() -> Self {
        let historical_barometer_altitude = HistoryBuffer::new();
        let historical_pressure = HistoryBuffer::new();
        Self {
            air: None,
            ekf_nav: (None, None),
            ekf_quat: None,
            imu: (None, None),
            utc_time: None,
            gps_vel: None,
            historical_barometer_altitude,
            historical_pressure,
            current_state: None,
            rolling_avg_baro_alt: 0.0,
            rolling_avg_pres: 0.0,
            rolling_avg_baro_alt_n: 0.0,
            rolling_avg_pres_n: 0.0,
        }
    }
    /// Returns true if the rocket is descending 
    pub fn is_falling(&mut self) -> bool {
        if self.historical_barometer_altitude.len() < 8 {
            return false;
        }
        let mut buf_baro = self.historical_barometer_altitude.oldest_ordered();
        match buf_baro.next() {
            Some(last) => {
                let mut avg_sum: f32 = 0.0;
                let mut prev = last;
                for i in buf_baro {
                    let time_diff: f32 = (i.1 - prev.1) as f32 / 1_000_000.0;
                    avg_sum += (i.0 - prev.0)/time_diff; 
                    prev = i;
                }
                self.rolling_avg_baro_alt_n = self.rolling_avg_baro_alt_n + 7.0; // 7 slopes 
                self.rolling_avg_baro_alt = self.rolling_avg_baro_alt + avg_sum;

                match self.rolling_avg_baro_alt/self.rolling_avg_baro_alt_n {
                    // exclusive range    
                    x if !(-100.0..=-5.0).contains(&x)  => { 
                        info!("avg baro: {}", self.rolling_avg_baro_alt/self.rolling_avg_baro_alt_n);
                        return false;
                    }
                    _ => {
                        info!("avg baro: {}", self.rolling_avg_baro_alt/self.rolling_avg_baro_alt_n);
                    }
                }
            }
            None => {
                return false;
            }
        }
        let mut buf_pres = self.historical_pressure.oldest_ordered();
        match buf_pres.next() {
            Some(last) => {
                let mut avg_sum: f32 = 0.0;
                let mut prev = last;
                for i in buf_pres {
                    let time_diff: f32 = (i.1 - prev.1) as f32 / 1_000_000.0;
                    avg_sum += (i.0 - prev.0)/time_diff; 
                    prev = i;
                }
                self.rolling_avg_pres_n = self.rolling_avg_pres_n + 7.0; // 7 slopes 
                self.rolling_avg_pres = self.rolling_avg_pres + avg_sum;
                match self.rolling_avg_pres / self.rolling_avg_pres_n {
                    // exclusive range  
                    x if x < 0.0  => { 
                        info!("avg pressure: {}", self.rolling_avg_pres / self.rolling_avg_pres_n);
                        return false;
                    }
                    _ => {
                        info!("avg pressure: {}", self.rolling_avg_pres / self.rolling_avg_pres_n);
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
    pub fn is_landed(&mut self) -> bool {
        if self.historical_barometer_altitude.len() < 8 {
            return false;
        }
        let mut buf_baro = self.historical_barometer_altitude.oldest_ordered();
        match buf_baro.next() {
            Some(last) => {
                let mut avg_sum: f32 = 0.0;
                let mut prev = last;
                for i in buf_baro {
                    let time_diff: f32 = (i.1 - prev.1) as f32 / 1_000_000.0;
                    avg_sum += (i.0 - prev.0)/time_diff; 
                    prev = i;
                }
                self.rolling_avg_baro_alt_n = self.rolling_avg_baro_alt_n + 7.0; // 7 slopes 
                self.rolling_avg_baro_alt = self.rolling_avg_baro_alt + avg_sum;

                match self.rolling_avg_baro_alt/self.rolling_avg_baro_alt_n {
                    // inclusive range    
                    x if (-4.0..=4.0).contains(&x)  => { 
                        info!("avg baro: {}", self.rolling_avg_baro_alt/self.rolling_avg_baro_alt_n);
                        return true;
                    }
                    _ => {
                        info!("avg baro: {}", self.rolling_avg_baro_alt/self.rolling_avg_baro_alt_n);
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
            Some(air) => air.altitude < MAIN_HEIGHT,
            None => false,
        }
    }
    pub fn set_state(&mut self, state: RocketStates) {
        self.current_state = Some(state);
    }
    pub fn handle_data(&mut self, data: Message) {
        match data.data {
            messages::Data::Sensor(sensor) => match sensor.data {
                messages::sensor::SensorData::Air(air_data) => {
                    let tup_data_alt = (air_data.altitude, air_data.time_stamp.clone());
                    let tup_data_pres = (air_data.pressure_abs, air_data.time_stamp);
                    self.air = Some(air_data);
                    if let Some(recent) = self.historical_barometer_altitude.recent() {
                        if recent.1 != tup_data_alt.1 {
                            self.historical_barometer_altitude.write(tup_data_alt);
                        }
                    } else { // first element
                        self.historical_barometer_altitude.write(tup_data_alt);
                    }
                    if let Some(recent) = self.historical_pressure.recent() {
                        if recent.1 != tup_data_pres.1 {
                            self.historical_pressure.write(tup_data_pres);
                        }
                    } else { // first element
                        self.historical_pressure.write(tup_data_pres);
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
                },
                _ => {
                    info!("impl power");
                }
            },
            messages::Data::Command(command) => match command.data {
                messages::command::CommandData::DeployDrogue(drogue) => {
                    spawn!(fire_drogue);
                },
                messages::command::CommandData::DeployMain(main) => {
                    spawn!(fire_main);
                },
                messages::command::CommandData::PowerDown(_) => {
                    // don't handle for now.
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
