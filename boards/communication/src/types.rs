use atsamd_hal::gpio::*;
use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::{uart, IoSet1, Sercom5};
use messages::node::Node;
use messages::node::Node::CommunicationBoard;

// -------
// Node ID
// -------
pub static COM_ID: Node = CommunicationBoard;

// -------
// Ground Station
// -------
pub type GroundStationPads = uart::PadsFromIds<Sercom5, IoSet1, PB17, PB16>;
pub type GroundStationUartConfig = uart::Config<GroundStationPads, EightBit>;
