use crate::sd_manager::TimeSink;
use atsamd_hal as hal;
use atsamd_hal::gpio::*;
use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::uart::Uart;
use atsamd_hal::sercom::{spi, uart, IoSet1, Sercom1};
use embedded_sdmmc as sd;
use hal::dmac;
use hal::dmac::BufferPair;
use hal::sercom::Sercom0;
use hal::sercom::Sercom5;
use messages::sender::Sender;
use messages::sender::Sender::LogicBoard;
use sbg_rs::sbg::SBG_BUFFER_SIZE;

// -------
// Sender ID
// -------
pub static COM_ID: Sender = LogicBoard;

// -------
// Ground Station
// -------
pub type GroundStationPads = uart::PadsFromIds<Sercom5, IoSet1, PB17, PB16>;
pub type GroundStationUartConfig = uart::Config<GroundStationPads, EightBit>;

// -------
// SBG
// -------
pub type PadsSBG = uart::PadsFromIds<Sercom0, IoSet1, PA09, PA08>;
pub type ConfigSBG = uart::Config<PadsSBG, EightBit>;
pub type SBGTransfer = dmac::Transfer<
    dmac::Channel<dmac::Ch0, dmac::Busy>,
    BufferPair<Uart<ConfigSBG, uart::RxDuplex>, SBGBuffer>,
>;
pub type SBGBuffer = &'static mut [u8; SBG_BUFFER_SIZE];

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