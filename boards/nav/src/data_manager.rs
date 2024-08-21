use common_arm::HydraError;
use messages::sensor::{
    Air, EkfNav1, EkfNav2, EkfNavAcc, EkfQuat, GpsPos1, GpsPos2, GpsPosAcc, GpsVel, GpsVelAcc,
    Imu1, Imu2, SensorData, UtcTime,
};
use messages::Message;
use heapless::Vec;

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

    pub fn take_sensors(&mut self) -> Vec<SensorData, 13> {
        let mut sensors = Vec::new();
    
        if let Some(data) = self.air.take().map(|x| x.into()) {
            sensors.push(data);
        }
    
        if let Some((data1, data2, data3)) = self.ekf_nav.take().map(|x| (x.0.into(), x.1.into(), x.2.into())) {
            sensors.push(data1);
            sensors.push(data2);
            sensors.push(data3);
        }
    
        if let Some(data) = self.ekf_quat.take().map(|x| x.into()) {
            sensors.push(data);
        }
    
        if let Some((data1, data2)) = self.imu.take().map(|x| (x.0.into(), x.1.into())) {
            sensors.push(data1);
            sensors.push(data2);
        }
    
        if let Some(data) = self.utc_time.take().map(|x| x.into()) {
            sensors.push(data);
        }
    
        if let Some((data1, data2)) = self.gps_vel.take().map(|x| (x.0.into(), x.1.into())) {
            sensors.push(data1);
            sensors.push(data2);
        }
    
        if let Some((data1, data2, data3)) = self.gps_pos.take().map(|x| (x.0.into(), x.1.into(), x.2.into())) {
            sensors.push(data1);
            sensors.push(data2);
            sensors.push(data3);
        }
    
        sensors
    }

    pub fn handle_data(&mut self, data: Message) -> Result<(), HydraError> {
        match data.data {
            messages::Data::Sensor(sensor) => match sensor.data {
                _ => {
                }
            },
            _ => {
                // we can disregard all other messages for now.
            }
        }
        Ok(())
    }

    pub fn handle_command(&mut self, data: Message) -> Result<(), HydraError> {
        match data.data {
            messages::Data::Command(command) => match command.data {
                messages::command::CommandData::PowerDown(_) => {
                    crate::app::sleep_system::spawn().ok();
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
