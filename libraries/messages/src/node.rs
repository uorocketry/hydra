use messages_proc_macros_lib::common_derives;

#[common_derives]
#[derive(Copy)]
pub enum Node {
    GroundStation,
    SensorBoard,
    RecoveryBoard,
    CommunicationBoard,
    PowerBoard,
    CameraBoard,
    BeaconBoard,
}

impl From<Node> for u16 {
    fn from(node: Node) -> Self {
        match node {
            Node::GroundStation => 0,
            Node::SensorBoard => 1,
            Node::RecoveryBoard => 2,
            Node::CommunicationBoard => 3,
            Node::PowerBoard => 4,
            Node::CameraBoard => 5,
            Node::BeaconBoard => 6,
        }
    }
}
