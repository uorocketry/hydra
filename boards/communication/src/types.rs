use atsamd_hal::gpio::*;
use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::uart::Uart;
use atsamd_hal::sercom::{uart, IoSet1, Sercom5};

// -------
// Ground Station
// -------
pub type GroundStationPads = uart::PadsFromIds<Sercom5, IoSet1, PB17, PB16>;
pub type GroundStationUartConfig = uart::Config<GroundStationPads, EightBit>;