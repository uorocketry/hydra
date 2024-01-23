use atsamd_hal as hal;
use atsamd_hal::gpio::*;
use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::uart::Uart;
use atsamd_hal::sercom::{spi, uart, IoSet6, Sercom5};
use hal::dmac;
use hal::dmac::BufferPair;
use hal::sercom::IoSet2;

use hal::sercom::Sercom4;
use messages::sender::Sender;
use messages::sender::Sender::SensorBoard;
use sbg_rs::sbg::SBG_BUFFER_SIZE;

// -------
// Sender ID
// -------
pub static COM_ID: Sender = SensorBoard;

// -------
// SBG
// -------
pub type PadsSBG = uart::PadsFromIds<Sercom5, IoSet6, PB03, PB02>;
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
    Sercom4,
    IoSet2,
    Pin<PB11, Alternate<D>>,
    Pin<PB08, Alternate<D>>,
    Pin<PB09, Alternate<D>>,
>;

pub type SdSpi = spi::Spi<spi::Config<SdPads>, spi::Duplex>;
