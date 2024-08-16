use crate::bindings::{
    SbgLogAirData, SbgLogEkfNavData, SbgLogEkfQuatData, SbgLogGpsPos, SbgLogGpsVel, SbgLogImuData,
    SbgLogUtcData,
};
use bitflags::Flags;
use messages::sensor::{
    Air, EkfNav1, EkfNav2, EkfNavAcc, EkfQuat, GpsPos1, GpsPos2, GpsPosAcc, GpsVel, GpsVelAcc,
    Imu1, Imu2, UtcTime,
};
use messages::sensor_status::{
    AirFlags, AirStatus, EkfFlags, EkfStatus, GpsPositionStatus, GpsPositionStatusE, GpsVelStatus,
    GpsVelStatusE, ImuFlags, ImuStatus, UtcStatus, UtcTimeStatus,
};

/// Simple helper function to work with the flags structure and set the fields as needed.
#[inline]
fn check<F, T>(flags: &Option<F>, test: F, value: T) -> Option<T>
where
    F: Flags,
{
    match flags {
        Some(x) if x.contains(test) => Some(value),
        _ => None,
    }
}

impl From<SbgLogGpsPos> for (GpsPos1, GpsPos2, GpsPosAcc) {
    fn from(value: SbgLogGpsPos) -> Self {
        let status = GpsPositionStatus::new(value.status);

        let valid = matches!(status.get_status(), Some(GpsPositionStatusE::SolComputed));
        (
            GpsPos1 {
                latitude: if valid { Some(value.latitude) } else { None },
                longitude: if valid { Some(value.longitude) } else { None },
            },
            GpsPos2 {
                altitude: if valid { Some(value.altitude) } else { None },
                undulation: if valid { Some(value.undulation) } else { None },
                time_of_week: if valid { Some(value.timeOfWeek) } else { None },
            },
            GpsPosAcc {
                status,
                time_stamp: value.timeStamp,
                latitude_accuracy: if valid {
                    Some(value.latitudeAccuracy)
                } else {
                    None
                },
                longitude_accuracy: if valid {
                    Some(value.longitudeAccuracy)
                } else {
                    None
                },
                altitude_accuracy: if valid {
                    Some(value.altitudeAccuracy)
                } else {
                    None
                },
                num_sv_used: if valid { Some(value.numSvUsed) } else { None },
                base_station_id: if valid {
                    Some(value.baseStationId)
                } else {
                    None
                },
                differential_age: if valid {
                    Some(value.differentialAge)
                } else {
                    None
                },
            },
        )
    }
}

impl From<SbgLogUtcData> for UtcTime {
    fn from(value: SbgLogUtcData) -> Self {
        let status = UtcTimeStatus::new(value.status);
        let valid = matches!(
            status.get_utc_status(),
            Some(UtcStatus::Valid | UtcStatus::NoLeapSec)
        );

        Self {
            time_stamp: value.timeStamp, // not convinced this is matched valid to the Utc Status bitmask.
            status,
            year: if valid { Some(value.year) } else { None },
            month: if valid { Some(value.month) } else { None },
            day: if valid { Some(value.day) } else { None },
            hour: if valid { Some(value.hour) } else { None },
            minute: if valid { Some(value.minute) } else { None },
            second: if valid { Some(value.second) } else { None },
            nano_second: if valid { Some(value.nanoSecond) } else { None },
            gps_time_of_week: if valid {
                Some(value.gpsTimeOfWeek)
            } else {
                None
            },
        }
    }
}

impl From<SbgLogAirData> for Air {
    fn from(value: SbgLogAirData) -> Self {
        let status = AirStatus::new(value.status);
        let flags = status.get_flags();

        Self {
            time_stamp: value.timeStamp, // TODO: check if valid.
            status,
            pressure_abs: check(&flags, AirFlags::PressureAbsValid, value.pressureAbs),
            altitude: check(&flags, AirFlags::AltitudeValid, value.altitude),
            pressure_diff: check(&flags, AirFlags::PressureDiffValid, value.pressureDiff),
            true_airspeed: check(&flags, AirFlags::AirpseedValid, value.trueAirspeed),
            air_temperature: check(&flags, AirFlags::TemperatureValid, value.airTemperature),
        }
    }
}

impl From<SbgLogEkfQuatData> for EkfQuat {
    fn from(value: SbgLogEkfQuatData) -> Self {
        let status = EkfStatus::new(value.status);
        let flags = status.get_flags();

        Self {
            time_stamp: value.timeStamp,
            quaternion: check(&flags, EkfFlags::HeadingValid, value.quaternion),
            euler_std_dev: check(&flags, EkfFlags::HeadingValid, value.eulerStdDev),
            status,
        }
    }
}

impl From<SbgLogEkfNavData> for (EkfNav1, EkfNav2, EkfNavAcc) {
    fn from(value: SbgLogEkfNavData) -> Self {
        let status = EkfStatus::new(value.status);
        let flags = status.get_flags();

        (
            EkfNav1 {
                time_stamp: value.timeStamp,
                velocity: check(&flags, EkfFlags::VelocityValid, value.velocity),
            },
            EkfNav2 {
                undulation: check(&flags, EkfFlags::AttitudeValid, value.undulation),
                position: check(&flags, EkfFlags::PositionValid, value.position),
            },
            EkfNavAcc {
                velocity_std_dev: check(&flags, EkfFlags::VelocityValid, value.velocityStdDev),
                position_std_dev: check(&flags, EkfFlags::PositionValid, value.positionStdDev),
                status,
            },
        )
    }
}

impl From<SbgLogImuData> for (Imu1, Imu2) {
    fn from(value: SbgLogImuData) -> Self {
        let status = ImuStatus::new(value.status);
        let flags = status.get_flags();

        (
            Imu1 {
                time_stamp: value.timeStamp,
                gyroscopes: check(&flags, ImuFlags::GyrosInRange, value.gyroscopes),
                status,
                accelerometers: check(&flags, ImuFlags::AccelsInRange, value.accelerometers),
            },
            Imu2 {
                temperature: Some(value.temperature), // we cannot check since no flag exists. Keep in option for uniformity.
                delta_velocity: check(&flags, ImuFlags::AccelsInRange, value.deltaVelocity),
                delta_angle: check(&flags, ImuFlags::GyrosInRange, value.deltaAngle),
            },
        )
    }
}

impl From<SbgLogGpsVel> for (GpsVel, GpsVelAcc) {
    fn from(value: SbgLogGpsVel) -> Self {
        let status = GpsVelStatus::new(value.status);

        let valid = matches!(status.get_status(), Some(GpsVelStatusE::SolComputed));

        (
            GpsVel {
                time_stamp: value.timeStamp,
                time_of_week: if valid { Some(value.timeOfWeek) } else { None },
                status,
                velocity: if valid { Some(value.velocity) } else { None },
                course: if valid { Some(value.course) } else { None },
            },
            GpsVelAcc {
                velocity_acc: if valid { Some(value.velocityAcc) } else { None },
                course_acc: if valid { Some(value.courseAcc) } else { None },
            },
        )
    }
}
