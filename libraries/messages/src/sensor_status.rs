use bitflags::bitflags;
use messages_proc_macros_lib::common_derives;
use serde::{Deserialize, Serialize};

// ----------
// EKF Status
// ----------

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EkfSolutionMode {
    #[doc = "The Kalman filter is not initialized and the returned data are all invalid."]
    Uninitialized,
    #[doc = "The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely."]
    VerticalGyro,
    #[doc = "A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely."]
    Ahrs,
    #[doc = "The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation."]
    NavVelocity,
    #[doc = "Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided."]
    NavPosition,
}

bitflags! {
    #[derive(Serialize, Deserialize, Debug, PartialEq, Eq, Hash)]
    pub struct EkfFlags: u32 {
        const AttitudeValid			= (0x00000001 << 4);
        const HeadingValid			= (0x00000001 << 5);
        const VelocityValid			= (0x00000001 << 6);
        const PositionValid			= (0x00000001 << 7);
        const VertRefUsed			= (0x00000001 << 8);
        const MagRefUsed			= (0x00000001 << 9);
        const Gps1VelUsed			= (0x00000001 << 10);
        const Gps1PosUsed			= (0x00000001 << 11);
        const Gps1HdtUsed			= (0x00000001 << 13);
        const Gps2VelUsed			= (0x00000001 << 14);
        const Gps2PosUsed			= (0x00000001 << 15);
        const Gps2HdtUsed			= (0x00000001 << 17);
        const OdoUsed				= (0x00000001 << 18);
        const DvlBtUsed			    = (0x00000001 << 19);
        const DvlWtUsed			    = (0x00000001 << 20);
        const UserPosUsed			= (0x00000001 << 21);
        const UserVelUsed			= (0x00000001 << 22);
        const UserHeadingUsed		= (0x00000001 << 23);
        const UsblUsed				= (0x00000001 << 24);
        const AirDataUsed			= (0x00000001 << 25);
        const ZuptUsed				= (0x00000001 << 26);
        const AlignValid			= (0x00000001 << 27);
        const DepthUsed			    = (0x00000001 << 28);
    }
}

#[common_derives]
#[derive(Copy)]
pub struct EkfStatus {
    status: u32,
}

impl EkfStatus {
    pub fn new(val: u32) -> Self {
        Self { status: val }
    }

    pub fn get_solution_mode(&self) -> Option<EkfSolutionMode> {
        match self.status & 0x0000000F {
            0 => Some(EkfSolutionMode::Uninitialized),
            1 => Some(EkfSolutionMode::VerticalGyro),
            2 => Some(EkfSolutionMode::Ahrs),
            3 => Some(EkfSolutionMode::NavVelocity),
            4 => Some(EkfSolutionMode::NavPosition),
            _ => None,
        }
    }

    pub fn get_flags(&self) -> Option<EkfFlags> {
        EkfFlags::from_bits(self.status & !(0x0000000F)) // check this line
    }
}

// ----------
// UTC Time Status
// ----------

pub enum ClockStatus {
    #[doc = "< An error has occurred on the clock estimation."]
    Error,
    #[doc = "< The clock is only based on the internal crystal."]
    FreeRunning,
    #[doc = "< A PPS has been detected and the clock is converging to it."]
    Steering,
    #[doc = "< The clock has converged to the PPS and is within 500ns."]
    Valid,
}

pub enum UtcStatus {
    #[doc = "< The UTC time is not known, we are just propagating the UTC time internally."]
    Invalid,
    #[doc = "< We have received valid UTC time information but we don't have the leap seconds information."]
    NoLeapSec,
    #[doc = "< We have received valid UTC time data with valid leap seconds."]
    Valid,
}

#[common_derives]
#[derive(Copy)]
pub struct UtcTimeStatus {
    status: u16,
}

impl UtcTimeStatus {
    pub fn new(status: u16) -> Self {
        Self { status }
    }

    pub fn get_clock_status(&self) -> Option<ClockStatus> {
        match (self.status >> 1) & 0x000F {
            0 => Some(ClockStatus::Error),
            1 => Some(ClockStatus::FreeRunning),
            2 => Some(ClockStatus::Steering),
            3 => Some(ClockStatus::Valid),
            _ => None,
        }
    }

    pub fn get_utc_status(&self) -> Option<UtcStatus> {
        match (self.status >> 6) & 0x000F {
            0 => Some(UtcStatus::Invalid),
            1 => Some(UtcStatus::NoLeapSec),
            2 => Some(UtcStatus::Valid),
            _ => None,
        }
    }
}

// ----------
// Air Status
// ----------

bitflags! {
    #[derive(Serialize, Deserialize, Debug, PartialEq, Eq, Hash)]
    pub struct AirFlags: u16 {
        const TimeIsDelay			= (0x0001 << 0);
        const PressureAbsValid		= (0x0001 << 1);
        const AltitudeValid			= (0x0001 << 2);
        const PressureDiffValid		= (0x0001 << 3);
        const AirpseedValid			= (0x0001 << 4);
        const TemperatureValid		= (0x0001 << 5);
    }
}

#[common_derives]
#[derive(Copy)]
pub struct AirStatus {
    status: u16,
}

impl AirStatus {
    pub fn new(status: u16) -> Self {
        Self { status }
    }

    pub fn get_flags(&self) -> Option<AirFlags> {
        AirFlags::from_bits(self.status)
    }
}

// ----------
// IMU Status
// ----------

bitflags! {
    #[derive(Serialize, Deserialize, Debug, PartialEq, Eq, Hash)]
    pub struct ImuFlags: u16 {
        const ComOk			    = (0x00000001 << 0);
        const StatusTestPass	= (0x00000001 << 1);
        const AccelXTestPass	= (0x00000001 << 2);
        const AccelYTestPass	= (0x00000001 << 3);
        const AccelZTestPass	= (0x00000001 << 4);
        const GyroXTestPass		= (0x00000001 << 5);
        const GyroYTestPass		= (0x00000001 << 6);
        const GyroZTestPass		= (0x00000001 << 7);
        const AccelsInRange	    = (0x00000001 << 8);
        const GyrosInRange	    = (0x00000001 << 9);
    }
}

#[common_derives]
#[derive(Copy)]
pub struct ImuStatus {
    status: u16,
}

impl ImuStatus {
    pub fn new(status: u16) -> Self {
        Self { status }
    }

    pub fn get_flags(&self) -> Option<ImuFlags> {
        ImuFlags::from_bits(self.status)
    }
}

// ----------
// GPS Status
// ----------

pub enum GpsPositionStatusE {
    #[doc = "< A valid solution has been computed."]
    SolComputed,
    #[doc = "< Not enough valid SV to compute a solution."]
    InsufficientObs,
    #[doc = "< An internal error has occurred."]
    InternalError,
    #[doc = "< The height limit has been exceeded."]
    HeightLimit,
}

pub enum GpsPositionType {
    #[doc = "< No valid position solution available."]
    NoSolution,
    #[doc = "< An unknown solution type has been computed."]
    UnknownType,
    #[doc = "< Single point solution position."]
    Single,
    #[doc = "< Standard Pseudorange Differential Solution (DGPS)."]
    PseudoRangeDiff,
    #[doc = "< SBAS satellite used for differential corrections."]
    Sbas,
    #[doc = "< Omnistar VBS Position (L1 sub-meter)."]
    OmniStar,
    #[doc = "< Floating RTK ambiguity solution (20 cms RTK)."]
    RtkFloat,
    #[doc = "< Integer RTK ambiguity solution (2 cms RTK)."]
    RtkInt,
    #[doc = "< Precise Point Positioning with float ambiguities."]
    PppFloat,
    #[doc = "< Precise Point Positioning with fixed ambiguities."]
    PppInt,
    #[doc = "< Fixed location solution position."]
    Fixed,
}

pub enum GpsSatelliteUsed {
    L1,
    L2,
    L5,
    GloL1,
    GloL2,
    GloL3,
    GalE1,
    GalE5A,
    GalE5B,
    GalE5Alt,
    GalE6,
    BdsB1,
    BdsB2,
    BdsB3,
    QzssL1,
    QzssL2,
    QzssL5,
}

#[common_derives]
#[derive(Copy)]
pub struct GpsPositionStatus {
    status: u32,
}

impl GpsPositionStatus {
    pub fn new(status: u32) -> Self {
        Self { status }
    }

    pub fn get_satellites_used(&self) -> Option<[GpsSatelliteUsed; 17]> {
        unimplemented!("Need to implement this");
    }

    pub fn get_status(&self) -> Option<GpsPositionStatusE> {
        match self.status & 0x0000003F {
            0 => Some(GpsPositionStatusE::SolComputed),
            1 => Some(GpsPositionStatusE::InsufficientObs),
            2 => Some(GpsPositionStatusE::InternalError),
            3 => Some(GpsPositionStatusE::HeightLimit),
            _ => None,
        }
    }

    pub fn get_type(&self) -> Option<GpsPositionType> {
        match (self.status >> 6) & 0x0000003F {
            0 => Some(GpsPositionType::NoSolution),
            1 => Some(GpsPositionType::UnknownType),
            2 => Some(GpsPositionType::Single),
            3 => Some(GpsPositionType::PseudoRangeDiff),
            4 => Some(GpsPositionType::Sbas),
            5 => Some(GpsPositionType::OmniStar),
            6 => Some(GpsPositionType::RtkFloat),
            7 => Some(GpsPositionType::RtkInt),
            8 => Some(GpsPositionType::PppFloat),
            9 => Some(GpsPositionType::PppInt),
            10 => Some(GpsPositionType::Fixed),
            _ => None,
        }
    }
}

// ----------
// GPS Vel Status
// ----------

pub enum GpsVelStatusE {
    #[doc = "< A valid solution has been computed."]
    SolComputed,
    #[doc = "< Not enough valid SV to compute a solution."]
    InsufficientObs,
    #[doc = "< An internal error has occurred."]
    InternalError,
    #[doc = "< Velocity limit exceeded."]
    VelLimit,
}

pub enum GpsVelType {
    #[doc = "< No valid velocity solution available."]
    NoSolution,
    #[doc = "< An unknown solution type has been computed."]
    UnknownType,
    #[doc = "< A Doppler velocity has been computed."]
    Doppler,
    #[doc = "< A differential velocity has been computed between two positions."]
    Differential,
}

#[common_derives]
#[derive(Copy)]
pub struct GpsVelStatus {
    status: u32,
}

impl GpsVelStatus {
    pub fn new(status: u32) -> Self {
        Self { status }
    }

    pub fn get_status(&self) -> Option<GpsVelStatusE> {
        match self.status & 0x0000003F {
            0 => Some(GpsVelStatusE::SolComputed),
            1 => Some(GpsVelStatusE::InsufficientObs),
            2 => Some(GpsVelStatusE::InternalError),
            3 => Some(GpsVelStatusE::VelLimit),
            _ => None,
        }
    }

    pub fn get_type(&self) -> Option<GpsVelType> {
        match (self.status >> 6) & 0x0000003F {
            0 => Some(GpsVelType::NoSolution),
            1 => Some(GpsVelType::UnknownType),
            2 => Some(GpsVelType::Doppler),
            3 => Some(GpsVelType::Differential),
            _ => None,
        }
    }
}

// ----------
// Tests
// ----------

#[cfg(test)]
mod tests {
    use crate::sensor_status::{EkfFlags, EkfSolutionMode, EkfStatus};

    #[test]
    fn ekf_status_valid_enum_success() {
        let status = EkfStatus::new(0b011);

        assert_eq!(
            status.get_solution_mode(),
            Some(EkfSolutionMode::NavVelocity)
        )
    }

    #[test]
    fn ekf_status_invalid_enum_expect_none() {
        let status = EkfStatus::new(0b111);

        assert_eq!(status.get_solution_mode(), None)
    }

    #[test]
    fn ekf_status_mixed_enum_flags_success() {
        let status = EkfStatus::new(0b00110010010);

        let expected_flags =
            EkfFlags::AttitudeValid | EkfFlags::PositionValid | EkfFlags::VertRefUsed;

        assert_eq!(status.get_solution_mode(), Some(EkfSolutionMode::Ahrs));

        assert_eq!(status.get_flags(), Some(expected_flags));
    }
}
