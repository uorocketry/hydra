use atsamd_hal::gpio::*;
use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::{uart, uart::Duplex, IoSet1, Sercom2, IoSet3};
use messages::sender::Sender;
use messages::sender::Sender::CommunicationBoard;

// -------
// Sender ID
// -------
pub static COM_ID: Sender = CommunicationBoard;

// -------
// GPS UART
// -------
pub type GpsPads = uart::PadsFromIds<Sercom2, IoSet1, PA13, PA12>;
pub type GpsUartConfig = uart::Config<GpsPads, EightBit>;
pub type GpsUart = uart::Uart<GpsUartConfig, Duplex>;
// -------
// Ground Station
// -------
pub type GroundStationPads = uart::PadsFromIds<Sercom2, IoSet3, PA08, PA09>;
pub type GroundStationUartConfig = uart::Config<GroundStationPads, EightBit>;
