use core::ffi::c_char;
use core::ptr;
use nb::{block, Error};
use defmt::{error, info, warn, debug};
use core::ptr::{null, null_mut};
use crate::bindings;
use crate::bindings::{_SbgInterface, SbgInterfaceHandle};
use embedded_hal::{serial, serial::Read, serial::Write, timer::CountDown, timer::Periodic};
struct UARTSBGInterface {
    interface: bindings::SbgInterface
}

pub struct SBG<T, U> where T: Read<u8> + Write<u8>, U: CountDown + Periodic {
    UARTSBGInterface: UARTSBGInterface,
    serial_device: T,
    timer: U,
} 

impl<T, U> SBG<T, U> where T: Read<u8> + Write<u8>, U: CountDown + Periodic{
    // UART device must implement both read and write traits
    pub fn new(serial_device: T, timer: U) -> Self {
        let interface = UARTSBGInterface {
            interface: _SbgInterface {
                handle: null_mut(),
                type_: 0,
                name: [0; 48],
                pDestroyFunc: None,
                pWriteFunc: None,
                pReadFunc: None,
                pFlushFunc: None,
                pSetSpeedFunc: None,
                pGetSpeedFunc: None,
                pDelayFunc: None,
            },
        };
        // run the init sequence here
        SBG {
            UARTSBGInterface: interface,
            serial_device,
            timer
        }
    }

    // only keep public for testing 
    /**
     * 
     */
    pub fn read(&mut self) -> Result<u8, Error<<T as Read<u8>>::Error>> {
        self.serial_device.read()
    }
    /**
     * 
     */
    pub fn write(&mut self, byte: u8) -> Result<(), Error<<T as Write<u8>>::Error>> {
        self.serial_device.write(byte)
    }
    /**
     * 
     */
    pub fn flush(&mut self) -> Result<(), Error<<T as Write<u8>>::Error>> {
        self.serial_device.flush()
    }
    /**
     * 
     */
    pub fn log(status: u8, message: &str) {
        match status {
            0 => error!("SBG Error {}", message),
            1 => warn!("SBG Warning {}", message),
            2 => info!("SBG Info {}", message),
            3 => debug!("SBG Debug {}", message),
            _ => info!("SBG Unknown {}", message)
        }
    }

    /**
     * 
     */
    pub fn getTime(&mut self) {

    }
    /**
     * 
     */
    pub fn sleep(&mut self) {
        // may need to block this
        // self.timer.wait();
    }
}