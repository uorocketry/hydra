use messages::sensor::{Air};
use messages::Message;
use messages::state::{State, StateData};
use defmt::info;
use heapless::HistoryBuffer;

const MAIN_HEIGHT: f32 = 876.0; // meters 
const HEIGHT_MIN: f32 = 600.0; // meters 

pub struct DataManager {
    pub air: Option<Air>,
    pub historical_barometer_altitude: HistoryBuffer<(f32, u32), 8>,
}

impl DataManager {
    pub fn new() -> Self {
        let historical_barometer_altitude = HistoryBuffer::new();
        Self {
            air: None,
            historical_barometer_altitude,
        }
    }
    /// Returns true if the rocket is descending 
    pub fn is_falling(&self) -> bool {
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
                    let slope = (i.0 - prev.0)/time_diff; 
                    if slope < -100.0 {
                        return false; 
                    }
                    avg_sum += slope; 
                    prev = i;
                }
                match avg_sum / 7.0 { // 7 because we have 8 points.  
                    // exclusive range  
                    x if !(-100.0..=-5.0).contains(&x) => { 
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
    pub fn is_landed(&self) -> bool {
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
                match avg_sum / 7.0 {
                    // inclusive range    
                    x if (-0.25..=0.25).contains(&x)  => { 
                        return true;
                    }
                    _ => {
                        // continue 
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
    pub fn handle_data(&mut self, data: Message) {
        match data.data {
            messages::Data::Sensor(sensor) => match sensor.data {
                messages::sensor::SensorData::Air(air_data) => {
                    self.air = Some(air_data);
                }
                _ => {
                }
            },
            _ => {
            }
        }
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
