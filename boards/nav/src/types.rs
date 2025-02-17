use hal::pac;
use hal::serial::Serial;
use messages::sender::{Sender, Sender::SensorBoard};
use sbg_rs::sbg::SBG_BUFFER_SIZE;
use stm32h7xx_hal as hal;

pub static COM_ID: Sender = SensorBoard;

pub type SBGSerial = Serial<pac::UART4>;
pub type SBGBuffer = &'static mut [u8; SBG_BUFFER_SIZE];
