use core::ffi::{c_char, c_void};
use core::ptr;
use nb::{block, Error};
use defmt::{error, info, warn, debug};
use core::ptr::{null, null_mut};
use crate::bindings::{self, _SbgErrorCode_SBG_READ_ERROR, _SbgErrorCode_SBG_NO_ERROR, _SbgErrorCode_SBG_WRITE_ERROR};
use crate::bindings::{_SbgInterface, SbgInterfaceHandle, _SbgErrorCode, SbgInterfaceReadFunc, sbgEComInit, _SbgEComHandle};
use embedded_hal::{serial, serial::Read, serial::Write, timer::CountDown, timer::Periodic};

struct UARTSBGInterface {
    interface: *mut bindings::SbgInterface
}

pub struct SBG<T> where T: Read<u8> + Write<u8>{
    UARTSBGInterface: UARTSBGInterface,
    serial_device: T,
} 

impl<T> SBG<T> where T: Read<u8> + Write<u8> {
    // UART device must implement both read and write traits
    pub fn new(mut serial_device: T) -> Self {
        let serial_ptr: *mut T = &mut serial_device;
        let interface = UARTSBGInterface {
            interface: &mut _SbgInterface {
                handle: serial_ptr.cast(),
                type_: 0,
                name: [0; 48],
                pDestroyFunc: None,
                pWriteFunc: Some(SBG::<T>::SbgInterfaceWriteFunc),
                pReadFunc: Some(SBG::<T>::SbgInterfaceReadFunc),
                pFlushFunc: None,
                pSetSpeedFunc: None,
                pGetSpeedFunc: None,
                pDelayFunc: None,
            },
        };
        let &mut handle: _SbgEComHandle; // initialize with dummy data then pass the handle to the init to be consumed 
        unsafe{sbgEComInit(handle, interface.interface);} // create a safe wrapper
        SBG {
            UARTSBGInterface: interface,
            serial_device,
        }
    }


    #[no_mangle]
    pub unsafe extern "C" fn SbgInterfaceReadFunc(pInterface: *mut _SbgInterface, pBuffer: *mut c_void, pBytesToRead: *mut usize, mut bytesRead: usize) -> _SbgErrorCode {
        // let mut array: &[u8] = (*pBuffer)._data;
        let serial: *mut T = *pInterface.cast();
        let mut counter: usize = 0;
        let mut array: [u8;250] = todo!();
        loop {
            if counter == 250 {
                break;
            }
            let result = serial.as_ref().expect("Serial reference").read();
            match result {
                Ok(word) => array[counter] = word,
                Err(_) => return _SbgErrorCode_SBG_READ_ERROR,
            }
            counter+=1;
        }
        bytesRead = counter;
        _SbgErrorCode_SBG_NO_ERROR
    }

    #[no_mangle]
    pub unsafe extern "C" fn SbgInterfaceWriteFunc(pInterface: *mut _SbgInterface, pBuffer: *const c_void, bytesToWrite: usize) -> _SbgErrorCode {
        let serial: *mut T = *pInterface.cast();
        let mut array: &[u8] = todo!();
        let mut counter: usize = 0;
        loop {
            if bytesToWrite == counter {
                break;
            }
            let result = serial.as_ref().expect("Serial reference").write(array[counter]);
            match result {
                Ok(_) => counter+=1,
                Err(_) => return _SbgErrorCode_SBG_WRITE_ERROR,
            }
        }
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * 
     */
    // fn read(&mut self) -> Result<u8, Error<<T as Read<u8>>::Error>> {
    //     self.serial_device.read()
    // }

    /**
     * 
     */
    // pub fn write(&mut self, byte: u8) -> Result<(), Error<<T as Write<u8>>::Error>> {
    //     self.serial_device.write(byte)
    // }
    /**
     * 
     */
    // pub fn flush(&mut self) -> Result<(), Error<<T as Write<u8>>::Error>> {
    //     self.serial_device.flush()
    // }
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
        todo!()
    }
    /**
     * 
     */
    pub fn sleep(&mut self, duration: u32) {
        todo!()
    }
}

unsafe impl<T> Send for SBG<T> where T: Read<u8> + Write<u8>  {} // this is wrong don't do this. Use a mutex and Arc<Mutex<SBG<T>>>! 