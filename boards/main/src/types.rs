use atsamd_hal as hal;
use atsamd_hal::gpio::*;

use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::uart::Uart;
use atsamd_hal::sercom::{uart, IoSet1};
use hal::dmac;
use hal::dmac::BufferPair;

use hal::sercom::Sercom0;
use hal::sercom::Sercom5;

use panic_halt as _;
use sbg_rs::sbg::SBG_BUFFER_SIZE;

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
