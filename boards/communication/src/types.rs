use crate::sd_manager::TimeSink;
use atsamd_hal::gpio::*;
use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::spi;
use atsamd_hal::sercom::{Sercom5, Sercom1, uart, IoSet1};
use messages::sender::Sender;
use messages::sender::Sender::CommunicationBoard;
use embedded_sdmmc as sd;

// -------
// Sender ID
// -------
pub static COM_ID: Sender = CommunicationBoard;

// -------
// Ground Station
// -------
pub type GroundStationPads = uart::PadsFromIds<Sercom5, IoSet1, PB17, PB16>;
pub type GroundStationUartConfig = uart::Config<GroundStationPads, EightBit>;

// -------
// SD Card
// -------
pub type SdPads = spi::Pads<
    Sercom1,
    IoSet1,
    Pin<PA19, Alternate<C>>,
    Pin<PA16, Alternate<C>>,
    Pin<PA17, Alternate<C>>,
>;
pub type SdController = sd::Controller<
    sd::SdMmcSpi<spi::Spi<spi::Config<SdPads>, spi::Duplex>, Pin<PA18, Output<PushPull>>>,
    TimeSink,
>;
