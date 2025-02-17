use atsamd_hal::gpio::*;
use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::{uart, uart::Duplex, IoSet1, Sercom2, IoSet3, uart::TxDuplex};
use messages::sender::Sender;
use messages::sender::Sender::GpsBoard;
use atsamd_hal::dmac;
use atsamd_hal::dmac::BufferPair;

// -------
// Sender ID
// -------
pub static COM_ID: Sender = GpsBoard;

// -------
// GPS UART
// -------
pub type GpsPads = uart::PadsFromIds<Sercom2, IoSet1, PA13, PA12>;
pub type GpsUartConfig = uart::Config<GpsPads, EightBit>;
pub type GpsUart = uart::Uart<GpsUartConfig, Duplex>;
pub type GpsUartTx = uart::Uart<GpsUartConfig, TxDuplex>;
pub type GPSTransfer = dmac::Transfer<
    dmac::Channel<dmac::Ch0, dmac::Busy>,
    BufferPair<uart::Uart<GpsUartConfig, uart::RxDuplex>, GPSBUFFER>,
>;
pub type GPSBUFFER = &'static mut [u8; 256];

// -------
// Ground Station
// -------
pub type GroundStationPads = uart::PadsFromIds<Sercom2, IoSet3, PA08, PA09>;
pub type GroundStationUartConfig = uart::Config<GroundStationPads, EightBit>;
