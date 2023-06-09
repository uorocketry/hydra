use defmt::Format;
use serde::{Deserialize, Serialize};

#[cfg(any(feature = "std", test))]
use proptest_derive::Arbitrary;

#[cfg(feature = "ts")]
use ts_rs::TS;

// I don't agree with the naming, We can use these as Ids to sent commands to that specific board.
#[derive(Serialize, Deserialize, Clone, Debug, Format, Copy)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum Sender {
    GroundStation,
    SensorBoard,
    RecoveryBoard,
    CommunicationBoard,
    PowerBoard,
    CameraBoard,
    BeaconBoard,
}

impl From<Sender> for u16 {
    fn from(sender: Sender) -> Self {
        match sender {
            Sender::GroundStation => 0,
            Sender::SensorBoard => 1,
            Sender::RecoveryBoard => 2,
            Sender::CommunicationBoard => 3,
            Sender::PowerBoard => 4,
            Sender::CameraBoard => 5,
            Sender::BeaconBoard => 6,
        }
    }
}
