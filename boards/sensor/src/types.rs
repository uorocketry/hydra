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
pub type PadsSBG = uart::PadsFromIds<Sercom0, IoSet1, PA09, PA08>;
pub type ConfigSBG = uart::Config<PadsSBG, EightBit>;
pub type SBGTransfer = dmac::Transfer<
    dmac::Channel<dmac::Ch0, dmac::Busy>,
    BufferPair<Uart<ConfigSBG, uart::RxDuplex>, SBGBufferWrapper>,
>;

pub type SBGBuffer = [u8; SBG_BUFFER_SIZE];
pub struct SBGBufferWrapper(pub SBGBuffer);
/// SAFETY: We are implementing an unsafe trait 
unsafe impl dmac::Buffer for SBGBufferWrapper {
    type Beat = u8;
    fn dma_ptr(&mut self) -> *mut Self::Beat {
        self.0.as_mut_ptr()
    }
    /// is this really not incrementing? 
    fn incrementing(&self) -> bool {
        false
    }
    fn buffer_len(&self) -> usize {
        SBG_BUFFER_SIZE
    }
}

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
