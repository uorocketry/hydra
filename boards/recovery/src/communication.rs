/// Encapsulates all communication logic.
use crate::types::*;
use atsamd_hal::can::Dependencies;
use atsamd_hal::clock::v2::ahb::AhbClk;
use atsamd_hal::clock::v2::gclk::Gclk0Id;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::clock::v2::types::Can0;
use atsamd_hal::clock::v2::Source;
use atsamd_hal::gpio::{Alternate, AlternateI, Pin, I, PA22, PA23};
use atsamd_hal::pac::CAN0;
use mcan::bus::Can;
use atsamd_hal::typelevel::Increment;
use common_arm::mcan;
use common_arm::mcan::tx_buffers::DynTx;
use common_arm::HydraError;
use heapless::Vec;
use mcan::embedded_can as ecan;
use mcan::interrupt::state::EnabledLine0;
use mcan::interrupt::{Interrupt, OwnedInterruptSet};
use mcan::message::tx;
use mcan::messageram::SharedMemory;
use mcan::{
    config::{BitTiming, Mode},
    filter::{Action, ExtFilter, Filter},
};
use messages::Message;

use common_arm::mcan::message::Raw;
use defmt::info;
use postcard::from_bytes;
use systick_monotonic::fugit::RateExtU32;

pub struct CanDevice0 {
    pub can: Can<
        'static,
        Can0,
        Dependencies<Can0, Gclk0Id, Pin<PA23, Alternate<I>>, Pin<PA22, Alternate<I>>, CAN0>,
        Capacities,
    >,
    line_interrupts: OwnedInterruptSet<Can0, EnabledLine0>,
}

impl CanDevice0 {
    // we need to give back gclk0
    pub fn new<S>(
        can_rx: Pin<PA23, AlternateI>,
        can_tx: Pin<PA22, AlternateI>,
        pclk_can: Pclk<Can0, Gclk0Id>,
        ahb_clock: AhbClk<Can0>,
        peripheral: CAN0,
        gclk0: S,
        can_memory: &'static mut SharedMemory<Capacities>,
    ) -> (Self, S::Inc)
    where
        S: Source<Id = Gclk0Id> + Increment,
    {
        let (can_dependencies, gclk0) =
            Dependencies::new(gclk0, pclk_can, ahb_clock, can_rx, can_tx, peripheral);

        let mut can =
            mcan::bus::CanConfigurable::new(200.kHz(), can_dependencies, can_memory).unwrap();
        can.config().mode = Mode::Fd {
            allow_bit_rate_switching: false,
            data_phase_timing: BitTiming::new(500.kHz()),
        };

        // Only for testing purposes
        can.config().loopback = true;

        let interrupts_to_be_enabled = can
            .interrupts()
            .split(
                [
                    Interrupt::RxFifo0NewMessage,
                    Interrupt::RxFifo0Full,
                    Interrupt::RxFifo0MessageLost,
                    Interrupt::RxFifo1NewMessage,
                    Interrupt::RxFifo1Full,
                    Interrupt::RxFifo1MessageLost,
                ]
                .into_iter()
                .collect(),
            )
            .unwrap();

        // Line 0 and 1 are connected to the same interrupt line
        let line_interrupts = can
            .interrupt_configuration()
            .enable_line_0(interrupts_to_be_enabled);

        can.filters_standard()
            .push(Filter::Classic {
                action: Action::StoreFifo0,
                filter: ecan::StandardId::ZERO,
                mask: ecan::StandardId::MAX,
            })
            .unwrap_or_else(|_| panic!("Standard filter"));

        can.filters_extended()
            .push(ExtFilter::Classic {
                action: Action::StoreFifo1,
                filter: ecan::ExtendedId::ZERO,
                mask: ecan::ExtendedId::MAX,
            })
            .unwrap_or_else(|_| panic!("Extended filter"));

        let can = can.finalize().unwrap();
        (
            CanDevice0 {
                can,
                line_interrupts,
            },
            gclk0,
        )
    }
    pub fn send_message(&mut self, m: Message) -> Result<(), HydraError> {
        let payload: Vec<u8, 64> = postcard::to_vec(&m)?;
        // let test: [u8; 64] = [0_u8;64];
        self.can.tx.transmit_queued(
            tx::MessageBuilder {
                // send a zero id for now, need to change filter
                // Change to m.sender.into().
                id: ecan::Id::Standard(ecan::StandardId::new(0).unwrap()),
                frame_type: tx::FrameType::FlexibleDatarate {
                    payload: &payload[..],
                    bit_rate_switching: false,
                    force_error_state_indicator: false,
                },
                store_tx_event: None,
            }
            .build()?,
        )?;
        Ok(())
    }
    pub fn process_data(&mut self) {
        let line_interrupts = &self.line_interrupts;
        for interrupt in line_interrupts.iter_flagged() {
            match interrupt {
                Interrupt::RxFifo0NewMessage => {
                    for message in &mut self.can.rx_fifo_0 {
                        let data: Message = from_bytes(message.data()).unwrap(); // handle the error properly
                        info!("Message: {:?}", data)
                    }
                }
                Interrupt::RxFifo1NewMessage => {
                    for message in &mut self.can.rx_fifo_1 {
                        let data: Message = from_bytes(message.data()).unwrap(); // handle the error properly
                        info!("Message: {:?}", data)
                    }
                }
                _ => (),
            }
        }
    }
}
