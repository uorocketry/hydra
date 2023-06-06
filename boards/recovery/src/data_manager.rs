use heapless::spsc::Queue;
use messages::sensor::SbgShort;
use messages::Message;

const MAIN_HEIGHT: f64 = 450.0;
const ACCEL_MIN: f64 = 20.0;

pub struct DataManager {
    pub sbg_short: Option<SbgShort>,
    pub historical_pressure: Queue<f32, 8>, // probably want to make this a ring buffer
}

impl DataManager {
    pub fn new() -> Self {
        let historical_pressure = Queue::new();
        Self {
            sbg_short: None,
            historical_pressure,
        }
    }
    pub fn get_accel_y(&self) -> f32 {
        self.sbg_short.as_ref().unwrap().accel_y
    }
    // Questions, questions, questions!!??? does it work!!!!!
    // Probably not, but it's a start
    pub fn is_falling(&self) -> bool {
        let mut point_previous = self.historical_pressure.peek().unwrap();
        for i in self.historical_pressure.iter() {
            let slope = i - point_previous;
            if slope > 0.0 {
                return false;
            }
            point_previous = i;
        }
        true
    }
    pub fn is_launched(&self) -> bool {
        self.sbg_short.as_ref().unwrap().height > ACCEL_MIN
    }
    pub fn is_below_main(&self) -> bool {
        self.sbg_short.as_ref().unwrap().height < MAIN_HEIGHT
    }
    pub fn handle_data(&mut self, data: Message) {
        match data.data {
            messages::Data::Sensor(sensor) => match sensor.data {
                messages::sensor::SensorData::SbgShort(sbg_short) => {
                    let pressure = sbg_short.pressure; // satisfy borrow checker
                    self.sbg_short = Some(sbg_short);
                    if self.historical_pressure.is_full() {
                        self.historical_pressure.dequeue().unwrap();
                    }
                    self.historical_pressure.enqueue(pressure).unwrap();
                }
                _ => {}
            },
            messages::Data::State(state) => {
                // handle state logic
            }
        }
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
