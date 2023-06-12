use heapless::spsc::Queue;
use messages::Message;
use messages::sensor::{Air, EkfNav1, EkfNav2, EkfQuat, GpsVel, Imu1, Imu2, UtcTime};
use crate::state_machine::RocketStates;

const MAIN_HEIGHT: f32 = 450.0;
const ACCEL_MIN: f32 = 20.0;

pub struct DataManager {
    pub sbg_air: Option<Air>,
    pub sbg_nav1: Option<EkfNav1>,
    pub sbg_nav2: Option<EkfNav2>,
    pub sbg_quat: Option<EkfQuat>,
    pub sbg_gps_vel: Option<GpsVel>,
    pub sbg_imu1: Option<Imu1>,
    pub sbg_imu2: Option<Imu2>,
    pub sbg_utc_time: Option<UtcTime>,
    pub historical_pressure: Queue<f32, 8>, // probably want to make this a ring buffer
    pub current_state: Option<RocketStates>,
}

impl DataManager {
    pub fn new() -> Self {
        let historical_pressure = Queue::new();
        Self {
            sbg_air: None,
            sbg_nav1: None,
            sbg_nav2: None,
            sbg_quat: None,
            sbg_gps_vel: None,
            sbg_imu1: None,
            sbg_imu2: None,
            sbg_utc_time: None,
            historical_pressure,
            current_state: None,
        }
    }
    /// Returns true if the rocket has passed apogee.
    /// This is determined by looking at the historical pressure data.
    /// Furthermore, we only start checking pressure data when velocity is less than 20m/s 
    /// because we want to avoid the complexities of pressure during transonic flight. 
    pub fn is_falling(&self) -> bool {
        let ekf_nav1 = self.sbg_nav1.as_ref();
        if let Some(ekf_nav1) = ekf_nav1 {
            if ekf_nav1.velocity[2] < 20.0 {
                return false;
            }
        } 
        match self.historical_pressure.peek() {
            Some(mut point_previous) => {
                for i in self.historical_pressure.iter() {
                    let slope = i - point_previous;
                    if slope > 0.0 {
                        return false;
                    }
                    point_previous = i;
                }
            },
            None => {
                return false;
            }
        }
        true
    }
    pub fn is_launched(&self) -> bool {
        match self.sbg_imu1.as_ref() {
            Some(imu1) => {
                imu1.accelerometers[1] > ACCEL_MIN
            },
            None => {
                false
            }
        }
    }
    pub fn is_below_main(&self) -> bool {
        match self.sbg_air.as_ref() {
            Some(air) => {
                air.altitude < MAIN_HEIGHT
            },
            None => {
                false
            }
        }
    }
    pub fn handle_data(&mut self, data: Message) {
        match data.data {
            messages::Data::Sensor(sensor) => match sensor.data {
                messages::sensor::SensorData::Air(air_data) => {
                    let pressure = air_data.pressure_abs;
                    self.sbg_air = Some(air_data);
                    if self.historical_pressure.is_full() {
                        self.historical_pressure.dequeue();
                    }
                    self.historical_pressure.enqueue(pressure);
                },
                messages::sensor::SensorData::EkfNav1(nav1_data) => {
                    self.sbg_nav1 = Some(nav1_data);
                },
                messages::sensor::SensorData::EkfNav2(nav2_data) => {
                    self.sbg_nav2 = Some(nav2_data);
                },
                messages::sensor::SensorData::EkfQuat(quat_data) => {
                    self.sbg_quat = Some(quat_data);
                },
                messages::sensor::SensorData::GpsVel(gps_vel_data) => {
                    self.sbg_gps_vel = Some(gps_vel_data);
                },
                messages::sensor::SensorData::Imu1(imu1_data) => {
                    self.sbg_imu1 = Some(imu1_data);
                },
                messages::sensor::SensorData::Imu2(imu2_data) => {
                    self.sbg_imu2 = Some(imu2_data);
                },
                messages::sensor::SensorData::UtcTime(utc_time_data) => {
                    self.sbg_utc_time = Some(utc_time_data);
                },
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
