use crate::state_machine::RocketStates;
use heapless::HistoryBuffer;
use messages::sensor::{Air, EkfNav1, EkfNav2, EkfQuat, GpsVel, Imu1, Imu2, UtcTime};
use messages::Message;
use defmt::info;

const MAIN_HEIGHT: f32 = 450.0;
const VELOCITY_MIN: f32 = 20.0;
const HEIGHT_MIN: f32 = 300.0;

pub struct DataManager {
    pub air: Option<Air>,
    pub ekf_nav: (Option<EkfNav1>, Option<EkfNav2>),
    pub ekf_quat: Option<EkfQuat>,
    pub imu: (Option<Imu1>, Option<Imu2>),
    pub utc_time: Option<UtcTime>,
    pub gps_vel: Option<GpsVel>,
    pub historical_pressure: HistoryBuffer<f32, 20>,
    pub current_state: Option<RocketStates>,
}

impl DataManager {
    pub fn new() -> Self {
        let historical_pressure = HistoryBuffer::new();
        Self {
            air: None,
            ekf_nav: (None, None),
            ekf_quat: None,
            imu: (None, None),
            utc_time: None,
            gps_vel: None,
            historical_pressure,
            current_state: None,
        }
    }
    /// Returns true if the rocket has passed apogee.
    /// This is determined by looking at the historical pressure data.
    /// Furthermore, we only start checking pressure data when velocity is less than 20m/s
    /// because we want to avoid the complexities of pressure during transonic flight.
    pub fn is_falling(&self) -> bool {
        let ekf_nav1 = self.ekf_nav.0.as_ref();
        if let Some(ekf_nav1) = ekf_nav1 {
            if ekf_nav1.velocity[2] > VELOCITY_MIN {
                info!("fail");
                return false;
            }
        }
        info!("points {}", self.historical_pressure.len());
        if self.historical_pressure.len() < 20 {
            info!("fail");
            return false;
        }
        match self.historical_pressure.last() {
            Some(last) => {
                let mut avg = 0.0;
                let mut prev = last;
                for i in self.historical_pressure.oldest_ordered() {
                    avg += (i - prev)/0.25;
                    prev = i;
                }
                match avg / 20.0 {
                    x if x < 10.0 => {
                        return false;
                    }
                    _ => {
                        info!("avg: {}", avg / 20.0);
                    }
                }
                // if (avg / 20.0 < ) { // 4 is the number of slopes. 
                //     return false; 
                // }
            }
            None => {
                return false;
            }
        }
        // match self.historical_pressure.recent() {
        //     Some(pressure) => {
        //         match self.historical_pressure.last() {
        //             Some(last) => {
        //             if (pressure - last) / 1.25 < 100.0 {
        //                 return false;
        //             }
        //         }
        //             None => {
        //                 return false;
        //             }
        //         }
        //     }
        //     None => {
        //         return false;
        //     }
        // }
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
    pub fn handle_data(&mut self, data: Message) {
        match data.data {
            messages::Data::Sensor(sensor) => match sensor.data {
                messages::sensor::SensorData::Air(air_data) => {
                    let pressure = air_data.pressure_abs;
                    self.air = Some(air_data);
                    self.historical_pressure.write(pressure);
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
            _ => {}
        }
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
