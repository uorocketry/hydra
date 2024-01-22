use crate::bindings::{
    SbgLogAirData, SbgLogEkfNavData, SbgLogEkfQuatData, SbgLogGpsPos, SbgLogGpsVel, SbgLogImuData,
    SbgLogUtcData,
};
use messages::sensor::{
    Air, EkfNav1, EkfNav2, EkfQuat, GpsPos1, GpsPos2, GpsVel, Imu1, Imu2, UtcTime,
};

impl From<SbgLogGpsPos> for (GpsPos1, GpsPos2) {
    fn from(value: SbgLogGpsPos) -> Self {
        (
            GpsPos1 {
                time_stamp: value.timeStamp,
                status: value.status,
                time_of_week: value.timeOfWeek,
                latitude: value.latitude,
                longitude: value.longitude,
                altitude: value.altitude,
                undulation: value.undulation,
            },
            GpsPos2 {
                latitude_accuracy: value.latitudeAccuracy,
                longitude_accuracy: value.longitudeAccuracy,
                altitude_accuracy: value.altitudeAccuracy,
                num_sv_used: value.numSvUsed,
                base_station_id: value.baseStationId,
                differential_age: value.differentialAge,
            },
        )
    }
}

impl From<SbgLogUtcData> for UtcTime {
    fn from(value: SbgLogUtcData) -> Self {
        Self {
            time_stamp: value.timeStamp,
            status: value.status,
            year: value.year,
            month: value.month,
            day: value.day,
            hour: value.hour,
            minute: value.minute,
            second: value.second,
            nano_second: value.nanoSecond,
            gps_time_of_week: value.gpsTimeOfWeek,
        }
    }
}

impl From<SbgLogAirData> for Air {
    fn from(value: SbgLogAirData) -> Self {
        Self {
            time_stamp: value.timeStamp,
            status: value.status,
            pressure_abs: value.pressureAbs,
            altitude: value.altitude,
            pressure_diff: value.pressureDiff,
            true_airspeed: value.trueAirspeed,
            air_temperature: value.airTemperature,
        }
    }
}

impl From<SbgLogEkfQuatData> for EkfQuat {
    fn from(value: SbgLogEkfQuatData) -> Self {
        Self {
            time_stamp: value.timeStamp,
            quaternion: value.quaternion,
            euler_std_dev: value.eulerStdDev,
            status: value.status,
        }
    }
}

impl From<SbgLogEkfNavData> for (EkfNav1, EkfNav2) {
    fn from(value: SbgLogEkfNavData) -> Self {
        (
            EkfNav1 {
                time_stamp: value.timeStamp,
                velocity: value.velocity,
                velocity_std_dev: value.velocityStdDev,
            },
            EkfNav2 {
                position: value.position,
                undulation: value.undulation,
                position_std_dev: value.positionStdDev,
                status: value.status,
            },
        )
    }
}

impl From<SbgLogImuData> for (Imu1, Imu2) {
    fn from(value: SbgLogImuData) -> Self {
        (
            Imu1 {
                time_stamp: value.timeStamp,
                status: value.status,
                accelerometers: value.accelerometers,
                gyroscopes: value.gyroscopes,
            },
            Imu2 {
                temperature: value.temperature,
                delta_velocity: value.deltaVelocity,
                delta_angle: value.deltaAngle,
            },
        )
    }
}

impl From<SbgLogGpsVel> for GpsVel {
    fn from(value: SbgLogGpsVel) -> Self {
        Self {
            time_stamp: value.timeStamp,
            status: value.status,
            time_of_week: value.timeOfWeek,
            velocity: value.velocity,
            velocity_acc: value.velocityAcc,
            course: value.course,
            course_acc: value.courseAcc,
        }
    }
}
