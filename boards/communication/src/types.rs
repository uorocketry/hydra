use atsamd_hal::gpio::*;
use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::{uart, IoSet1, IoSet3, Sercom0, Sercom2, Sercom4, Sercom5};
use messages::node::Node;
use messages::node::Node::CommunicationBoard;

// -------
// Node ID
// -------
pub static COM_ID: Node = CommunicationBoard;

// -------
// Ground Station
// -------
pub type GroundStationPads = uart::PadsFromIds<Sercom0, IoSet1, PA09, PA08>;
pub type GroundStationUartConfig = uart::Config<GroundStationPads, EightBit>;

// -------
// GPS
// -------
pub type GpsPads = uart::PadsFromIds<Sercom2, IoSet1, PA13, PA12>;
pub type GpsUartConfig = uart::Config<GpsPads, EightBit>;
