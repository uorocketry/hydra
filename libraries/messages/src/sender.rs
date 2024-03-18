use messages_proc_macros_lib::common_derives;

// I don't agree with the naming, We can use these as Ids to sent commands to that specific board.
#[common_derives]
#[derive(Copy)]
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
