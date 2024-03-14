use crate::sensor_status::{AirStatus, EkfStatus, GpsVelStatus, ImuStatus, UtcTimeStatus, GpsPositionStatus};
use defmt::Format;
use derive_more::From;
use serde::{Deserialize, Serialize};

#[cfg(any(feature = "std", test))]
use proptest_derive::Arbitrary;

#[cfg(feature = "ts")]
use ts_rs::TS;

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct Sensor {
    /// Used to differentiate between multiple components on the same sender. Unused right now.
    pub component_id: u8,
    pub data: SensorData,
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum SensorData {
    UtcTime(UtcTime),
    Air(Air),
    EkfQuat(EkfQuat),
    EkfNav1(EkfNav1),
    EkfNav2(EkfNav2),
    Imu1(Imu1),
    Imu2(Imu2),
    GpsVel(GpsVel),
    GpsPos1(GpsPos1),
    GpsPos2(GpsPos2),
}

/* Replace with new health monitor */

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(test, derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct Regulator {
    pub status: bool,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(test, derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct Voltage {
    pub voltage: f32,
    pub rolling_avg: f32,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(test, derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct Current {
    pub current: f32,
    pub rolling_avg: f32,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(test, derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct Temperature {
    pub temperature: f32,
    pub rolling_avg: f32,
}

/* Replace with new health monitor */


#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(test, derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct GpsPos1 {
    #[doc = "< Time in us since the sensor power up."]
    pub time_stamp: u32,
    #[doc = "< GPS position status, type and bitmask."]
    pub status: GpsPositionStatus,
    #[doc = "< GPS time of week in ms."]
    pub time_of_week: Option<u32>,
    #[doc = "< Latitude in degrees, positive north."]
    pub latitude: Option<f64>,
    #[doc = "< Longitude in degrees, positive east."]
    pub longitude: Option<f64>,
    #[doc = "< Altitude above Mean Sea Level in meters."]
    pub altitude: Option<f64>,
    #[doc = "< Altitude difference between the geoid and the Ellipsoid in meters (Height above Ellipsoid = altitude + undulation)."]
    pub undulation: Option<f32>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(test, derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct GpsPos2 {
    #[doc = "< 1 sigma latitude accuracy in meters."]
    pub latitude_accuracy: Option<f32>,
    #[doc = "< 1 sigma longitude accuracy in meters."]
    pub longitude_accuracy: Option<f32>,
    #[doc = "< 1 sigma altitude accuracy in meters."]
    pub altitude_accuracy: Option<f32>,
    #[doc = "< Number of space vehicles used to compute the solution (since version 1.4)."]
    pub num_sv_used: Option<u8>,
    #[doc = "< Base station id for differential corrections (0-4095). Set to 0xFFFF if differential corrections are not used (since version 1.4)."]
    pub base_station_id: Option<u16>,
    #[doc = "< Differential correction age in 0.01 seconds. Set to 0XFFFF if differential corrections are not used (since version 1.4)."]
    pub differential_age: Option<u16>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct UtcTime {
    #[doc = "< Time in us since the sensor power up."]
    pub time_stamp: u32, 
    #[doc = "< UTC time and clock status information"]
    pub status: UtcTimeStatus,
    #[doc = "< Year for example: 2013."]
    pub year: Option<u16>,
    #[doc = "< Month in year [1 .. 12]."]
    pub month: Option<i8>,
    #[doc = "< Day in month [1 .. 31]."]
    pub day: Option<i8>,
    #[doc = "< Hour in day [0 .. 23]."]
    pub hour: Option<i8>,
    #[doc = "< Minute in hour [0 .. 59]."]
    pub minute: Option<i8>,
    #[doc = "< Second in minute [0 .. 60]. (60 is used only when a leap second is added)"]
    pub second: Option<i8>,
    #[doc = "< Nanosecond of current second in ns."]
    pub nano_second: Option<i32>,
    #[doc = "< GPS time of week in ms."]
    pub gps_time_of_week: Option<u32>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct Air {
    #[doc = "< Time in us since the sensor power up."]
    pub time_stamp: u32,
    #[doc = "< Airdata sensor status bitmask."]
    pub status: AirStatus,
    #[doc = "< Raw absolute pressure measured by the barometer sensor in Pascals."]
    pub pressure_abs: Option<f32>,
    #[doc = "< Altitude computed from barometric altimeter in meters and positive upward."]
    pub altitude: Option<f32>,
    #[doc = "< Raw differential pressure measured by the pitot tube in Pascal."]
    pub pressure_diff: Option<f32>,
    #[doc = "< True airspeed measured by a pitot tube in m.s^-1 and positive forward."]
    pub true_airspeed: Option<f32>,
    #[doc = "< Outside air temperature in °C that could be used to compute true airspeed from differential pressure."]
    pub air_temperature: Option<f32>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct EkfQuat {
    #[doc = "< Time in us since the sensor power up."]
    pub time_stamp: u32,
    #[doc = "< Orientation quaternion stored in W, X, Y, Z form."]
    pub quaternion: Option<[f32; 4usize]>,
    #[doc = "< Roll, Pitch and Yaw angles 1 sigma standard deviation in rad."]
    pub euler_std_dev: Option<[f32; 3usize]>,
    #[doc = "< EKF solution status bitmask and enum."]
    pub status: EkfStatus,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct EkfNav1 {
    #[doc = "< Time in us since the sensor power up."]
    pub time_stamp: u32,
    #[doc = "< North, East, Down velocity in m.s^-1."]
    pub velocity: Option<[f32; 3usize]>,
    #[doc = "< North, East, Down velocity 1 sigma standard deviation in m.s^-1."]
    pub velocity_std_dev: Option<[f32; 3usize]>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct EkfNav2 {
    #[doc = "< Latitude, Longitude in degrees positive North and East.\nAltitude above Mean Sea Level in meters."]
    pub position: Option<[f64; 3usize]>,
    #[doc = "< Altitude difference between the geoid and the Ellipsoid in meters (Height above Ellipsoid = altitude + undulation)."]
    pub undulation: Option<f32>,
    #[doc = "< Latitude, longitude and altitude 1 sigma standard deviation in meters."]
    pub position_std_dev: Option<[f32; 3usize]>,
    #[doc = "< EKF solution status bitmask and enum."]
    pub status: EkfStatus,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct Imu1 {
    #[doc = "< Time in us since the sensor power up."]
    pub time_stamp: u32,
    #[doc = "< IMU status bitmask."]
    pub status: ImuStatus,
    #[doc = "< X, Y, Z accelerometers in m.s^-2."]
    pub accelerometers: Option<[f32; 3usize]>,
    #[doc = "< X, Y, Z gyroscopes in rad.s^-1."]
    pub gyroscopes: Option<[f32; 3usize]>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct Imu2 {
    #[doc = "< Internal temperature in °C."]
    pub temperature: Option<f32>,
    #[doc = "< X, Y, Z delta velocity in m.s^-2."]
    pub delta_velocity: Option<[f32; 3usize]>,
    #[doc = "< X, Y, Z delta angle in rad.s^-1."]
    pub delta_angle: Option<[f32; 3usize]>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct GpsVel {
    #[doc = "< GPS time of week in ms."]
    pub time_of_week: Option<u32>,
    #[doc = "< Time in us since the sensor power up."]
    pub time_stamp: u32,
    #[doc = "< GPS velocity status, type and bitmask."]
    pub status: GpsVelStatus,
    #[doc = "< GPS North, East, Down velocity in m.s^-1."]
    pub velocity: Option<[f32; 3usize]>,
    #[doc = "< GPS North, East, Down velocity 1 sigma accuracy in m.s^-1."]
    pub velocity_acc: Option<[f32; 3usize]>,
    #[doc = "< Track ground course in degrees."]
    pub course: Option<f32>,
    #[doc = "< Course accuracy in degrees."]
    pub course_acc: Option<f32>,
}

impl Sensor {
    pub fn new(data: impl Into<SensorData>) -> Self {
        Sensor {
            component_id: 0,
            data: data.into(),
        }
    }
}
