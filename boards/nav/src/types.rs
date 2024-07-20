use hal::pac;
use hal::serial::Serial;
use stm32h7xx_hal as hal;

pub type SBGSerial = Serial<pac::UART4>;
