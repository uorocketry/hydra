use crate::data_manager::DataManager;
use crate::types::COM_ID;
use common_arm::HydraError;
use defmt::info;
use fdcan::{
    frame::{FrameFormat, TxFrameHeader},
    id::StandardId,
};
use messages::Message;
use postcard::from_bytes;
use stm32h7xx_hal::prelude::*;

/// Clock configuration is out of scope for this builder
/// easiest way to avoid alloc is to use no generics
pub struct CanDevice {
    can: fdcan::FdCan<
        stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN2>,
        fdcan::NormalOperationMode,
    >,
}

impl CanDevice {
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
                    data_manager.handle_data(data)?;
                }
                Err(e) => {
                    info!("Error: {:?}", e)
                }
            }
        }
        Ok(())
    }
}
