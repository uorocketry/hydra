use crate::data_manager::DataManager;
use crate::types::*;
use atsamd_hal::can::Dependencies;
use atsamd_hal::clock::v2::ahb::AhbClk;
use atsamd_hal::clock::v2::gclk::Gclk0Id;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::clock::v2::pclk::PclkToken;
use atsamd_hal::clock::v2::types::Can0;
use atsamd_hal::clock::v2::Source;
use atsamd_hal::gpio::{Alternate, AlternateI, Disabled, Floating, Pin, I, PA22, PA23, PB16, PB17};
use atsamd_hal::pac::CAN0;
use atsamd_hal::pac::MCLK;
use atsamd_hal::pac::SERCOM5;
use atsamd_hal::prelude::_embedded_hal_serial_Read;
use atsamd_hal::sercom;
use atsamd_hal::sercom::uart;
use atsamd_hal::sercom::uart::Uart;
use atsamd_hal::sercom::uart::{RxDuplex, TxDuplex};
use atsamd_hal::typelevel::Increment;
use common_arm::mcan;
use common_arm::mcan::message::{rx, Raw};
use common_arm::mcan::tx_buffers::DynTx;
use common_arm::{herror, HydraError};
use heapless::HistoryBuffer;
use heapless::Vec;
use mavlink::embedded::Read;
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
use messages::mavlink;
use messages::ErrorContext;
use messages::Message;
use postcard::from_bytes;
use systick_monotonic::fugit::RateExtU32;
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

        let mut can =
            mcan::bus::CanConfigurable::new(200.kHz(), can_dependencies, can_memory).unwrap();
        can.config().mode = Mode::Fd {
            allow_bit_rate_switching: false,
            data_phase_timing: BitTiming::new(500.kHz()),
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
                filter: ecan::StandardId::new(messages::node::Node::RecoveryBoard.into()).unwrap(),
                mask: ecan::StandardId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Recovery filter"));

        can.filters_standard()
            .push(Filter::Classic {
                action: Action::StoreFifo1,
                filter: ecan::StandardId::new(messages::node::Node::SensorBoard.into()).unwrap(),
                mask: ecan::StandardId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Sensor filter"));

        can.filters_standard()
            .push(Filter::Classic {
                action: Action::StoreFifo0,
                filter: ecan::StandardId::new(messages::node::Node::PowerBoard.into()).unwrap(),
                mask: ecan::StandardId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Power filter"));

        can.filters_standard()
            .push(Filter::Classic {
                action: Action::StoreFifo0,
                filter: ecan::StandardId::new(messages::node::Node::GroundStation.into()).unwrap(),
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
                    bit_rate_switching: false,
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
                                data_manager.handle_data(data);
                            }
                            Err(_) => {
                                herror!(Error, ErrorContext::UnkownCanMessage);
                            }
                        }
                    }
                }
                Interrupt::RxFifo1NewMessage => {
                    for message in &mut self.can.rx_fifo_1 {
                        match from_bytes::<Message>(message.data()) {
                            Ok(data) => {
                                data_manager.handle_data(data);
                            }
                            Err(_) => {
                                herror!(Error, ErrorContext::UnkownCanMessage);
                            }
                        }
                    }
                }
                _ => (),
            }
        }
    }
}

pub struct RadioDevice {
    transmitter: Uart<GroundStationUartConfig, TxDuplex>,
    receiver: Uart<GroundStationUartConfig, RxDuplex>,
}

impl RadioDevice {
    pub fn new<S>(
        radio_token: PclkToken<SERCOM5>,
        mclk: &MCLK,
        sercom: SERCOM5,
        rx_pin: Pin<PB17, Disabled<Floating>>,
        tx_pin: Pin<PB16, Disabled<Floating>>,
        gclk0: S,
    ) -> (Self, S::Inc)
    where
        S: Source<Id = Gclk0Id> + Increment,
    {
        let (pclk_radio, gclk0) = Pclk::enable(radio_token, gclk0);
        let pads = uart::Pads::<sercom::Sercom5, _>::default()
            .rx(rx_pin)
            .tx(tx_pin);
        let uart = GroundStationUartConfig::new(mclk, sercom, pads, pclk_radio.freq())
            .baud(
                57600.Hz(),
                uart::BaudMode::Fractional(uart::Oversampling::Bits16),
            )
            .enable();
        let (mut rx, tx) = uart.split();
        // setup interrupts
        rx.enable_interrupts(uart::Flags::RXC);
        (
            RadioDevice {
                transmitter: tx,
                receiver: rx,
            },
            gclk0,
        )
    }
}

pub struct RadioManager {
    buf: HistoryBuffer<u8, 280>,
    radio: RadioDevice,
    mav_sequence: u8,
}

impl RadioManager {
    pub fn new(radio: RadioDevice) -> Self {
        let buf = HistoryBuffer::new();
        RadioManager {
            buf,
            radio,
            mav_sequence: 0,
        }
    }
    pub fn send_message(&mut self, m: Message) -> Result<(), HydraError> {
        let payload: Vec<u8, 255> = postcard::to_vec(&m)?;

        let mav_header = mavlink::MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: self.increment_mav_sequence(),
        };

        let mav_message = mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(
            mavlink::uorocketry::POSTCARD_MESSAGE_DATA { message: payload },
        );
        mavlink::write_versioned_msg(
            &mut self.radio.transmitter,
            mavlink::MavlinkVersion::V2,
            mav_header,
            &mav_message,
        )?;
        Ok(())
    }
    pub fn increment_mav_sequence(&mut self) -> u8 {
        self.mav_sequence = self.mav_sequence.wrapping_add(1);
        self.mav_sequence
    }
    pub fn receive_message(&mut self) -> Result<Message, HydraError> {
        if let Ok(data) = self.radio.receiver.read() {
            // lets add this data to the buffer and see if we can parse it
            self.buf.write(data);
            let (_header, msg) =
                mavlink::read_versioned_msg(&mut self.radio.receiver, mavlink::MavlinkVersion::V2)?;
            // Do we need the header?
            match msg {
                mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(msg) => {
                    return Ok(postcard::from_bytes::<Message>(&msg.message)?);
                    // weird Ok syntax to coerce to hydra error type.
                }
                _ => {
                    herror!(Error, ErrorContext::UnkownPostcardMessage);
                    return Err(mavlink::error::MessageReadError::Io.into());
                }
            }
        } else {
            return Err(mavlink::error::MessageReadError::Io.into());
        }
    }
}

impl Read for RadioManager {
    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), mavlink::error::MessageReadError> {
        let len = buf.len().min(self.buf.len());
        buf[..len].copy_from_slice(&self.buf[..len]);
        Ok(())
    }
    fn read_u8(&mut self) -> Result<u8, mavlink::error::MessageReadError> {
        if !self.buf.is_empty() {
            Ok(self.buf[0])
        } else {
            Err(mavlink::error::MessageReadError::Io)
        }
    }
}
