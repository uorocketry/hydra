
use crate::data_manager::DataManager;
use atsamd_hal::can::Dependencies;
use atsamd_hal::clock::v2::ahb::AhbClk;
use atsamd_hal::clock::v2::gclk::{Gclk0Id};
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::clock::v2::types::{Can0, Can1};
use atsamd_hal::clock::v2::Source;
use atsamd_hal::gpio::{
    Alternate, AlternateI, Pin, I, PA22, PA23,
};
use atsamd_hal::gpio::{AlternateH, H, PB14, PB15};
use atsamd_hal::pac::{CAN0, CAN1};
use atsamd_hal::prelude::*;
use atsamd_hal::typelevel::Increment;
use common_arm::HydraError;
use common_arm_atsame::mcan;
use common_arm_atsame::mcan::message::{rx, Raw};
use common_arm_atsame::mcan::tx_buffers::DynTx;
use defmt::error;
use defmt::info;
use heapless::Vec;
use mcan::bus::Can;
use mcan::embedded_can as ecan;
use mcan::interrupt::state::EnabledLine0;
use mcan::interrupt::{Interrupt, OwnedInterruptSet};
use mcan::message::tx;
use mcan::messageram::SharedMemory;
use mcan::{
    config::{BitTiming, Mode},
    filter::{Action, Filter},
};
use messages::mavlink::{self};
use messages::Message;
use postcard::from_bytes;
use systick_monotonic::*;
use typenum::{U0, U128, U32, U64};
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

// macro_rules! create_filter {
//     ($can:expr, $action:expr, $sender:expr) => {
//         $can.filters_standard()
//             .push(Filter::Classic {
//                 action: $action,
//                 filter: ecan::StandardId::new($sender.into()).unwrap(),
//                 mask: ecan::StandardId::ZERO,
//             })
//             .unwrap_or_else(|_| panic!("Filter Error"));
//     };
// }

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
    pub fn new<S>(
        can_rx: Pin<PA23, AlternateI>,
        can_tx: Pin<PA22, AlternateI>,
        pclk_can: Pclk<Can0, Gclk0Id>,
        ahb_clock: AhbClk<Can0>,
        peripheral: CAN0,
        gclk0: S,
        can_memory: &'static mut SharedMemory<Capacities>,
        loopback: bool,
    ) -> (Self, S::Inc)
    where
        S: Source<Id = Gclk0Id> + Increment,
    {
        let (can_dependencies, gclk0) =
            Dependencies::new(gclk0, pclk_can, ahb_clock, can_rx, can_tx, peripheral);

        let mut can = mcan::bus::CanConfigurable::new(
            200.kHz(),
            can_dependencies,
            can_memory,
        )
        .unwrap();
        can.config().mode = Mode::Fd {
            allow_bit_rate_switching: true,
            data_phase_timing: BitTiming::new(fugit::RateExtU32::kHz(500)),
        };

        if loopback {
            can.config().loopback = true;
        }

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
                filter: ecan::StandardId::new(messages::sender::Sender::RecoveryBoard.into())
                    .unwrap(),
                mask: ecan::StandardId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Recovery filter"));

        can.filters_standard()
            .push(Filter::Classic {
                action: Action::StoreFifo1,
                filter: ecan::StandardId::new(messages::sender::Sender::SensorBoard.into())
                    .unwrap(),
                mask: ecan::StandardId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Sensor filter"));

        can.filters_standard()
            .push(Filter::Classic {
                action: Action::StoreFifo0,
                filter: ecan::StandardId::new(messages::sender::Sender::PowerBoard.into()).unwrap(),
                mask: ecan::StandardId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Power filter"));

        can.filters_standard()
            .push(Filter::Classic {
                action: Action::StoreFifo0,
                filter: ecan::StandardId::new(messages::sender::Sender::GroundStation.into())
                    .unwrap(),
                mask: ecan::StandardId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Ground Station filter"));

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
        self.can.tx.transmit_queued(
            tx::MessageBuilder {
                id: ecan::Id::Standard(ecan::StandardId::new(m.sender.into()).unwrap()),
                frame_type: tx::FrameType::FlexibleDatarate {
                    payload: &payload[..],
                    bit_rate_switching: true,
                    force_error_state_indicator: false,
                },
                store_tx_event: None,
            }
            .build()?,
        )?;
        Ok(())
    }
    pub fn process_data(&mut self, data_manager: &mut DataManager) {
        let line_interrupts = &self.line_interrupts;
        for interrupt in line_interrupts.iter_flagged() {
            match interrupt {
                Interrupt::RxFifo0NewMessage => {
                    for message in &mut self.can.rx_fifo_0 {
                        match from_bytes::<Message>(message.data()) {
                            Ok(data) => {
                                // info!("Received message {}", data.clone());

                                data_manager.handle_data(data);
                            }
                            Err(_) => {
                                // error!("Error, ErrorContext::UnkownCanMessage");
                            }
                        }
                    }
                }
                Interrupt::RxFifo1NewMessage => {
                    for message in &mut self.can.rx_fifo_1 {
                        match from_bytes::<Message>(message.data()) {
                            Ok(data) => {
                                info!("Received message {}", data.clone());

                                data_manager.handle_data(data);
                            }
                            Err(_) => {
                                // error!("Error, ErrorContext::UnkownCanMessage");
                            }
                        }
                    }
                }
                _ => (),
            }
        }
    }
}

// So I really am not a fan of this can device 0 and can device 1, I think it would be better to have a single generic can manager
// that can also take a list of filters and apply them.

pub struct CanCommandManager {
    pub can: Can<
        'static,
        Can1,
        Dependencies<Can1, Gclk0Id, Pin<PB15, Alternate<H>>, Pin<PB14, Alternate<H>>, CAN1>,
        Capacities,
    >,
    line_interrupts: OwnedInterruptSet<Can1, EnabledLine0>,
}

impl CanCommandManager {
    pub fn new<S>(
        can_rx: Pin<PB15, AlternateH>,
        can_tx: Pin<PB14, AlternateH>,
        pclk_can: Pclk<Can1, Gclk0Id>,
        ahb_clock: AhbClk<Can1>,
        peripheral: CAN1,
        gclk0: S,
        can_memory: &'static mut SharedMemory<Capacities>,
        loopback: bool,
    ) -> (Self, S::Inc)
    where
        S: Source<Id = Gclk0Id> + Increment,
    {
        let (can_dependencies, gclk0) =
            Dependencies::new(gclk0, pclk_can, ahb_clock, can_rx, can_tx, peripheral);

        let mut can = mcan::bus::CanConfigurable::new(
            200.kHz(), // needs a prescaler of 6 to be changed in the mcan source because mcan is meh. 
            can_dependencies,
            can_memory,
        )
        .unwrap();
        can.config().mode = Mode::Fd {
            allow_bit_rate_switching: true,
            data_phase_timing: BitTiming::new(fugit::RateExtU32::kHz(500)),
        };

        if loopback {
            can.config().loopback = true;
        }

        let interrupts_to_be_enabled = can
            .interrupts()
            .split(
                [
                    Interrupt::RxFifo0NewMessage,
                    Interrupt::RxFifo0Full,
                    // Interrupt::RxFifo0MessageLost,
                    Interrupt::RxFifo1NewMessage,
                    Interrupt::RxFifo1Full,
                    // Interrupt::RxFifo1MessageLost,
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
                filter: ecan::StandardId::new(messages::sender::Sender::RecoveryBoard.into())
                    .unwrap(),
                mask: ecan::StandardId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Recovery filter"));

        can.filters_standard()
            .push(Filter::Classic {
                action: Action::StoreFifo1,
                filter: ecan::StandardId::new(messages::sender::Sender::SensorBoard.into())
                    .unwrap(),
                mask: ecan::StandardId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Sensor filter"));

        can.filters_standard()
            .push(Filter::Classic {
                action: Action::StoreFifo0,
                filter: ecan::StandardId::new(messages::sender::Sender::PowerBoard.into()).unwrap(),
                mask: ecan::StandardId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Power filter"));

        can.filters_standard()
            .push(Filter::Classic {
                action: Action::StoreFifo0,
                filter: ecan::StandardId::new(messages::sender::Sender::GroundStation.into())
                    .unwrap(),
                mask: ecan::StandardId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Ground Station filter"));

        let can = can.finalize().unwrap();
        (
            CanCommandManager {
                can,
                line_interrupts,
            },
            gclk0,
        )
    }
    pub fn send_message(&mut self, m: Message) -> Result<(), HydraError> {
        let payload: Vec<u8, 64> = postcard::to_vec(&m)?;
        self.can.tx.transmit_queued(
            tx::MessageBuilder {
                id: ecan::Id::Standard(ecan::StandardId::new(m.sender.into()).unwrap()),
                frame_type: tx::FrameType::FlexibleDatarate {
                    payload: &payload[..],
                    bit_rate_switching: true,
                    force_error_state_indicator: false,
                },
                store_tx_event: None,
            }
            .build()?,
        )?;
        Ok(())
    }
    pub fn process_data(&mut self, data_manager: &mut DataManager) {
        let line_interrupts = &self.line_interrupts;
        for interrupt in line_interrupts.iter_flagged() {
            match interrupt {
                Interrupt::RxFifo0NewMessage => {
                    for message in &mut self.can.rx_fifo_0 {
                        match from_bytes::<Message>(message.data()) {
                            Ok(data) => {
                                info!("Received message {}", data.clone());
                                data_manager.handle_command(data);
                            }
                            Err(_) => {
                                error!("Error, ErrorContext::UnkownCanMessage");
                            }
                        }
                    }
                }
                Interrupt::RxFifo1NewMessage => {
                    for message in &mut self.can.rx_fifo_1 {
                        match from_bytes::<Message>(message.data()) {
                            Ok(data) => {
                                info!("Received message {}", data.clone());
                                data_manager.handle_command(data);
                            }
                            Err(_) => {
                                error!("Error, ErrorContext::UnkownCanMessage");
                            }
                        }
                    }
                }
                _ => (),
            }
        }
    }
}