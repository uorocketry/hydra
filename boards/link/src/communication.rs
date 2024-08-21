use crate::data_manager::DataManager;
use crate::types::COM_ID;
use common_arm::HydraError;
use fdcan::{
    frame::{FrameFormat, TxFrameHeader},
    id::StandardId,
};
use messages::Message;
use postcard::from_bytes;
use stm32h7xx_hal::prelude::*;
use mavlink::peek_reader::PeekReader;
use messages::mavlink::uorocketry::MavMessage;
use messages::mavlink::{self};
use defmt::{info, error};

/// Clock configuration is out of scope for this builder
/// easiest way to avoid alloc is to use no generics
pub struct CanCommandManager {
    can: fdcan::FdCan<
        stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN1>,
        fdcan::NormalOperationMode,
    >,
}

impl CanCommandManager {
    pub fn new(
        can: fdcan::FdCan<
            stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN1>,
            fdcan::NormalOperationMode,
        >,
    ) -> Self {
        Self { can }
    }
    pub fn send_message(&mut self, m: Message) -> Result<(), HydraError> {
        let mut buf = [0u8; 64];
        let payload = postcard::to_slice(&m, &mut buf)?;
        let header = TxFrameHeader {
            len: payload.len() as u8, // switch to const as this never changes or swtich on message type of known size
            id: StandardId::new(COM_ID.into()).unwrap().into(),
            frame_format: FrameFormat::Standard,
            bit_rate_switching: false,
            marker: None,
        };
        self.can.transmit(header, &payload)?;
        Ok(())
    }
    pub fn process_data(&mut self, data_manager: &mut DataManager) -> Result<(), HydraError> {
        let mut buf = [0u8; 64];
        for message in self.can.receive0(&mut buf) {
            match from_bytes::<Message>(&buf) {
                Ok(data) => {
                    info!("Received message {}", data.clone());
                    data_manager.handle_command(data)?;
                }
                Err(e) => {
                    info!("Error: {:?}", e)
                }
            }
        }
        Ok(())
    }
}


/// Clock configuration is out of scope for this builder
/// easiest way to avoid alloc is to use no generics
pub struct CanDataManager {
    can: fdcan::FdCan<
        stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN2>,
        fdcan::NormalOperationMode,
    >,
}

impl CanDataManager {
    pub fn new(
        can: fdcan::FdCan<
            stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN2>,
            fdcan::NormalOperationMode,
        >,
    ) -> Self {
        Self { can }
    }
    pub fn send_message(&mut self, m: Message) -> Result<(), HydraError> {
        let mut buf = [0u8; 64];
        let payload = postcard::to_slice(&m, &mut buf)?;
        let header = TxFrameHeader {
            len: payload.len() as u8, // switch to const as this never changes or swtich on message type of known size
            id: StandardId::new(COM_ID.into()).unwrap().into(),
            frame_format: FrameFormat::Fdcan,
            bit_rate_switching: false,
            marker: None,
        };
        // self.can.abort(fdcan::Mailbox::_2); // this is needed if boards are not in sync (if they are not in sync that is a bigger problem)

        stm32h7xx_hal::nb::block!(self.can.transmit(header, &payload))?;

        Ok(())
    }
    pub fn process_data(&mut self, data_manager: &mut DataManager) -> Result<(), HydraError> {
        let mut buf = [0u8; 64];
        for message in self.can.receive0(&mut buf) {
            match from_bytes::<Message>(&buf) {
                Ok(data) => {
                    info!("Received message {}", data.clone());
                    crate::app::send_gs::spawn(data).ok();
                    // data_manager.handle_data(data);
                }
                Err(e) => {
                    info!("Error: {:?}", e)
                }
            }
        }
        self.can.clear_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);
        Ok(())
    }
}


pub struct RadioDevice {
    transmitter: stm32h7xx_hal::serial::Tx<stm32h7xx_hal::pac::UART4>,
    pub receiver: PeekReader<stm32h7xx_hal::serial::Rx<stm32h7xx_hal::pac::UART4>>,
}

impl RadioDevice {
    pub fn new(
        uart: stm32h7xx_hal::serial::Serial<stm32h7xx_hal::pac::UART4>,
    ) -> Self {
        
        let (tx, mut rx) = uart.split();

        rx.listen();
        // setup interrupts
        
        RadioDevice {
            transmitter: tx,
            receiver: PeekReader::new(rx),
        }
    }
}

pub struct RadioManager {
    pub radio: RadioDevice,
    mav_sequence: u8,
}

impl RadioManager {
    pub fn new(radio: RadioDevice) -> Self {
        RadioManager {
            radio,
            mav_sequence: 0,
        }
    }
    pub fn send_message(&mut self, payload: &[u8]) -> Result<(), HydraError> {
        let mav_header = mavlink::MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: self.increment_mav_sequence(),
        };
        // Create a fixed-size array and copy the payload into it
        let mut fixed_payload = [0u8; 255];
        let len = payload.len().min(255);
        fixed_payload[..len].copy_from_slice(&payload[..len]);

        let mav_message = mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(
            mavlink::uorocketry::POSTCARD_MESSAGE_DATA {
                message: fixed_payload,
            },
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
        let (_header, msg): (_, MavMessage) =
            mavlink::read_versioned_msg(&mut self.radio.receiver, mavlink::MavlinkVersion::V2)?;
        
        // info!("{:?}", );
        // Do we need the header?
        match msg {
            mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(msg) => {
                return Ok(postcard::from_bytes::<Message>(&msg.message)?);
                // weird Ok syntax to coerce to hydra error type.
            }
            mavlink::uorocketry::MavMessage::COMMAND_MESSAGE(command) => {
                info!("{}", command.command);
                return Ok(postcard::from_bytes::<Message>(&command.command)?);
            }
            mavlink::uorocketry::MavMessage::HEARTBEAT(heartbeat) => {
                info!("Heartbeat");
                return Err(mavlink::error::MessageReadError::Io.into());
            }
            _ => {
                error!("Error, ErrorContext::UnkownPostcardMessage");
                return Err(mavlink::error::MessageReadError::Io.into());
            }
        }
    }
}
