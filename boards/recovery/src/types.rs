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