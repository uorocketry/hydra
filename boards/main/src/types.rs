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

const SBG_BUFFER_SIZE: usize = 4096;
pub static mut BUF_DST: SBGBuffer = &mut [0; SBG_BUFFER_SIZE];
pub static mut BUF_DST2: SBGBuffer = &mut [0; SBG_BUFFER_SIZE];

pub type Pads = uart::PadsFromIds<Sercom5, IoSet1, PB17, PB16>;
pub type PadsSBG = uart::PadsFromIds<Sercom0, IoSet1, PA09, PA08>;
pub type Config = uart::Config<Pads, EightBit>;
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

// pub type RxFifo0 = RxFifo<
//     'static,
//     Fifo0,
//     hal::clock::v2::types::Can0,
//     <Capacities as mcan::messageram::Capacities>::RxFifo0Message,
// >;

// pub type RxFifo1 = RxFifo<
//     'static,
//     Fifo1,
//     hal::clock::v2::types::Can0,
//     <Capacities as mcan::messageram::Capacities>::RxFifo1Message,
// >;

// pub type Tx = mcan::tx_buffers::Tx<'static, hal::clock::v2::types::Can0, Capacities>;
// pub type TxEventFifo = mcan::tx_event_fifo::TxEventFifo<'static, hal::clock::v2::types::Can0>;
// pub type Aux0 = mcan::bus::Aux<
//     'static,
//     hal::clock::v2::types::Can0,
//     hal::can::Dependencies<
//         hal::clock::v2::types::Can0,
//         hal::clock::v2::gclk::Gclk0Id,
//         Pin<PA23, AlternateI>,
//         Pin<PA22, AlternateI>,
//         pac::CAN0,
//     >,
// >;
