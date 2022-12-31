use core::ffi::c_char;
use core::ptr;
use nb::{block, Error};
use core::ptr::{null, null_mut};
use crate::bindings;
use crate::bindings::{_SbgInterface, SbgInterfaceHandle};
use embedded_hal::{serial, serial::Read, serial::Write};
struct UARTSBGInterface {
    interface: bindings::SbgInterface
}

pub struct SBG<T> where T: Read<u8> + Write<u8>  {
    UARTSBGInterface: UARTSBGInterface,
    serial_device: T,
} 

impl<T> SBG<T> where T: Read<u8> + Write<u8> {
    // UART device must implement both read and write traits
    pub fn new(serial_device: T) -> Self {
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
        }
    }

    // only keep public for testing 
    pub fn read(&mut self) -> Result<u8, Error<<T as Read<u8>>::Error>> {
        self.serial_device.read()
    }

    pub fn write(&mut self, byte: u8) -> Result<(), Error<<T as Write<u8>>::Error>> {
        self.serial_device.write(byte)
    }

    /* Need to implement  
    - get current time 
    - sleep 
    - logging 
    - flush serial 
    - destroy serial
    - create serial
    - change serial baud rate 
    */
}