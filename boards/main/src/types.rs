use atsamd_hal as hal;
use atsamd_hal::gpio::*;

use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::uart::Uart;
use atsamd_hal::sercom::{uart, IoSet1};
use common_arm::mcan;
use hal::dmac;
use hal::dmac::BufferPair;

use hal::sercom::Sercom0;
use hal::sercom::Sercom5;
use mcan::generic_array::typenum::consts::*;
use mcan::message::rx;
use mcan::message::tx;

use panic_halt as _;
use sbg_rs::sbg::SBG_BUFFER_SIZE;

pub type Pads = uart::PadsFromIds<Sercom5, IoSet1, PB17, PB16>;
pub type Config = uart::Config<Pads, EightBit>;

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

pub struct Capacities;

impl mcan::messageram::Capacities for Capacities {
    type StandardFilters = U128;
    type ExtendedFilters = U64;
    type RxBufferMessage = rx::Message<64>;
    type DedicatedRxBuffers = U64;
    type RxFifo0Message = rx::Message<64>;
    type RxFifo0 = U64;
    type RxFifo1Message = rx::Message<64>;
    type RxFifo1 = U64;
    type TxMessage = tx::Message<64>;
    type TxBuffers = U32;
    type DedicatedTxBuffers = U0;
    type TxEventFifo = U32;
}
