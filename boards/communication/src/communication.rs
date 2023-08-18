/// Encapsulates all communication logic.
use crate::data_manager::DataManager;
use crate::types::*;
use atsamd_hal::can::Dependencies;
use atsamd_hal::clock::v2::ahb::AhbClk;
use atsamd_hal::clock::v2::gclk::Gclk0Id;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::clock::v2::pclk::PclkToken;
use atsamd_hal::clock::v2::types::Can0;
use atsamd_hal::clock::v2::Source;
use atsamd_hal::dmac;
use atsamd_hal::gpio::{Alternate, AlternateI, Disabled, Floating, Pin, I, PA22, PA23, PB16, PB17};
use atsamd_hal::pac::CAN0;
use atsamd_hal::pac::MCLK;
use atsamd_hal::pac::SERCOM5;
use atsamd_hal::sercom::Sercom;
use atsamd_hal::sercom::{uart, Sercom5};
use atsamd_hal::sercom;
use atsamd_hal::sercom::uart::{ TxDuplex, RxDuplex};
use atsamd_hal::sercom::uart::{Duplex, Uart};
use atsamd_hal::time::*;
use atsamd_hal::typelevel::Increment;
use common_arm::mcan;
use common_arm::mcan::message::{rx, Raw};
use common_arm::mcan::tx_buffers::DynTx;
use common_arm::HydraError;
use defmt::info;
use crate::app::send_internal;
use common_arm::spawn;
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
use messages::mavlink;
use messages::Message;
use postcard::from_bytes;
use systick_monotonic::fugit::RateExtU32;
use typenum::{U0, U128, U32, U64};

use atsamd_hal::dmac::Transfer;
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
                            Err(e) => {
                                info!("Error: {:?}", e)
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
                            Err(e) => {
                                info!("Error: {:?}", e)
                            }
                        }
                    }
                }
                _ => (),
            }
        }
    }
}

pub static mut BUF_DST: RadioBuffer = &mut [0; 255];
pub static mut BUF_DST2: RadioBuffer = &mut [0; 255];

pub struct RadioDevice {
    uart: Uart<GroundStationUartConfig, TxDuplex>,
    mav_sequence: u8,
}

impl RadioDevice {
    pub fn new<S>(
        radio_token: PclkToken<SERCOM5>,
        mclk: &MCLK,
        sercom: SERCOM5,
        rx_pin: Pin<PB17, Disabled<Floating>>,
        tx_pin: Pin<PB16, Disabled<Floating>>,
        gclk0: S,
        mut dma_channel: dmac::Channel<dmac::Ch0, dmac::Ready>,
    ) -> (Self, S::Inc, RadioTransfer)
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
        let (rx, tx) = uart.split(); // tx sends rx uses dma to receive so we need to setup dma here. 
        dma_channel.as_mut().enable_interrupts(dmac::InterruptFlags::new().with_tcmpl(true));
        let xfer = Transfer::new(dma_channel, rx, unsafe {&mut *BUF_DST}, false).expect("DMA Radio RX").begin(Sercom5::DMA_RX_TRIGGER, dmac::TriggerAction::BURST);
        (
            RadioDevice {
                uart: tx,
                mav_sequence: 0,
            },
            gclk0,
            xfer
        )
    }
    pub fn send_message(&mut self, m: Message) -> Result<(), HydraError> {
        let payload: Vec<u8, 255> = postcard::to_vec(&m)?;

        let mav_header = mavlink::MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: self.mav_sequence.wrapping_add(1),
        };

        let mav_message = mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(
            mavlink::uorocketry::POSTCARD_MESSAGE_DATA { message: payload },
        );

        mavlink::write_versioned_msg(
            &mut self.uart,
            mavlink::MavlinkVersion::V2,
            mav_header,
            &mav_message,
        )?;
        Ok(())
    }
}

pub struct RadioManager {
    xfer: RadioTransfer,
    buf_select: bool,
}

impl RadioManager {
    pub fn new(
        xfer: RadioTransfer,
    ) -> Self {
        RadioManager {
            xfer,
            buf_select: false,
        }
    }
    pub fn process_message(&mut self, buf: RadioBuffer) {
        info!("Received: {:?}", buf);
        match from_bytes::<Message>(buf) {
            Ok(msg) => {
                info!("Radio: {}", msg);
                spawn!(send_internal, msg); // dump to the Can Bus
            }
            Err(_) => {
                info!("Radio unknown msg");
                return;
            }
        }
    }
}

pub fn radio_dma(cx: crate::app::radio_dma::Context) {
    let manager = cx.local.radio_manager;

    if manager.xfer.complete() {
        cx.shared.em.run(|| {
            let buf = match manager.buf_select {
                false => {
                    manager.buf_select = true;
                    manager.xfer.recycle_source(unsafe{&mut *BUF_DST})?
                }
                true => {
                    manager.buf_select = false;
                    manager.xfer.recycle_source(unsafe{&mut *BUF_DST2})?
                }
            };
            manager.process_message(buf);
            Ok(())
        })
    }
}