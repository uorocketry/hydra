use common_arm::mcan;
use mcan::generic_array::typenum::consts::*;
use mcan::message::rx;
use mcan::message::tx;

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
