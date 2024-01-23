use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::{uart, IoSet1, Sercom5};
use atsamd_hal::gpio::*;
use messages::sender::Sender;
use messages::sender::Sender::CommunicationBoard;

// -------
// Sender ID
// -------
pub static COM_ID: Sender = CommunicationBoard;

// -------
// Ground Station
// -------
pub type GroundStationPads = uart::PadsFromIds<Sercom5, IoSet1, PB17, PB16>;
pub type GroundStationUartConfig = uart::Config<GroundStationPads, EightBit>;