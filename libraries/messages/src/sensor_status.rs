use bitflags::bitflags;
use defmt::Format;
use serde::{Deserialize, Serialize};

#[cfg(test)]
use proptest_derive::Arbitrary;

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

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(test, derive(Arbitrary))]
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
        EkfFlags::from_bits(self.status & !(0x0000000F))
    }
}
