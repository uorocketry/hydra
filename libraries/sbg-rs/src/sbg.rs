use core::ffi::{c_char, c_void, VaListImpl};
use core::ptr;
use nb::{block, Error};
use defmt::{error, info, warn, debug};
use core::ptr::{null, null_mut};
use crate::bindings::{self, _SbgErrorCode_SBG_READ_ERROR, _SbgErrorCode_SBG_NO_ERROR, _SbgErrorCode_SBG_WRITE_ERROR, sbgEComProtocolSend, sbgEComProtocolPayloadConstruct, sbgEComBinaryLogWriteGpsRawData, sbgEComBinaryLogWriteEkfEulerData, _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0};
use crate::bindings::{_SbgInterface, SbgInterfaceHandle, _SbgErrorCode, SbgInterfaceReadFunc, sbgEComInit, _SbgEComHandle, _SbgEComProtocol, _SbgBinaryLogData, _SbgDebugLogType};
use embedded_hal::{serial, serial::Read, serial::Write, timer::CountDown, timer::Periodic};
use core::slice::from_raw_parts;

struct UARTSBGInterface {
    interface: *mut bindings::SbgInterface
}

pub struct SBG<T> where T: Read<u8> + Write<u8> {
    UARTSBGInterface: UARTSBGInterface,
    serial_device: T,
} 

/**
 * Todo
 * - Add assert def
 */
impl<T> SBG<T> where T: Read<u8> + Write<u8> {
    // UART device must implement both read and write traits
    pub fn new(mut serial_device: T) -> Self {
        let serial_ptr: *mut T = &mut serial_device;
        let interface = UARTSBGInterface {
            interface: &mut _SbgInterface {
                handle: serial_ptr.cast(),
                type_: 0,
                name: [0; 48],
                pDestroyFunc: Some(SBG::<T>::SbgDestroyFunc),
                pWriteFunc: Some(SBG::<T>::SbgInterfaceWriteFunc),
                pReadFunc: Some(SBG::<T>::SbgInterfaceReadFunc),
                pFlushFunc: Some(SBG::<T>::SbgFlushFunc),
                pSetSpeedFunc: Some(SBG::<T>::SbgSetSpeedFunc),
                pGetSpeedFunc: Some(SBG::<T>::SbgGetSpeedFunc),
                pDelayFunc: Some(SBG::<T>::SbgDelayFunc),
            },
        };

        let mut x: u8 = 10;
        let pLargeBuffer: *mut u8 = &mut x;
        // Create some dummy data to be able to create the struct. 
        let mut protocol: _SbgEComProtocol = _SbgEComProtocol { pLinkedInterface: interface.interface, rxBuffer: [0;4096usize], rxBufferSize: 4096usize, discardSize: 16, nextLargeTxId: 16, pLargeBuffer, largeBufferSize: 16, msgClass: 0, msgId: 0, transferId: 0, pageIndex: 0, nrPages: 2 };
        unsafe {
        let handle: *mut _SbgEComHandle = &mut _SbgEComHandle {protocolHandle: protocol, pReceiveLogCallback: Some(SBG::<T>::SbgEComReceiveLogFunc), pUserArg: null_mut(), numTrials: 3, cmdDefaultTimeOut: 500};
         // initialize with dummy data then pass the handle to the init to be consumed 
        sbgEComInit(handle, interface.interface);
        let data: &[u8;3] = &[1,2,3]; // simple test data 
        sbgEComProtocolSend(&mut protocol, 0, 0, data.as_ptr() as *const c_void, 3);} // create a safe wrapper
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
        /**
         *  This is an issue
         */
        let mut array: &[u8] = from_raw_parts(pBuffer as *const u8, bytesToWrite);
        let mut counter: usize = 0;
        loop {
            if bytesToWrite == counter {
                break;
            }
            let result = serial.as_mut().expect("Serial reference").write(array[counter]);
            match result {
                Ok(_) => counter+=1,
                Err(_) => return _SbgErrorCode_SBG_WRITE_ERROR,
            }
        }
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * Callback function for handling logs. 
     */
    #[no_mangle]
    pub unsafe extern "C" fn SbgEComReceiveLogFunc(pHandle: *mut _SbgEComHandle, msgClass: u32, msg: u8, pLogData: *const _SbgBinaryLogData, pUserArg: *mut c_void) -> _SbgErrorCode{
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * Unimplemented
     */
    #[no_mangle]
    pub unsafe extern "C" fn SbgDestroyFunc(pInterface: *mut _SbgInterface) -> _SbgErrorCode{
        _SbgErrorCode_SBG_NO_ERROR
    }

    #[no_mangle]
    pub unsafe extern "C" fn SbgFlushFunc(pInterface: *mut _SbgInterface, flags: u32) -> _SbgErrorCode {
        _SbgErrorCode_SBG_NO_ERROR
    }

    #[no_mangle]
    pub unsafe extern "C" fn SbgSetSpeedFunc(pInterface: *mut _SbgInterface, speed: u32) -> _SbgErrorCode {
        _SbgErrorCode_SBG_NO_ERROR
    }

    #[no_mangle]
    pub unsafe extern "C" fn SbgGetSpeedFunc(pInterface: *const _SbgInterface) -> u32 {
        9600
    }
    #[no_mangle]
    pub unsafe extern "C" fn SbgDelayFunc(pInterface: *const _SbgInterface, numBytes: usize) -> u32 {
        200
    }
    // /**
    //  * 
    //  */
    // pub fn log(status: u8, message: &str) {
    //     match status {
    //         0 => error!("SBG Error {}", message),
    //         1 => warn!("SBG Warning {}", message),
    //         2 => info!("SBG Info {}", message),
    //         3 => debug!("SBG Debug {}", message),    
    //         _ => info!("SBG Unknown {}", message)
    //     }
    // }



}

unsafe impl<T> Send for SBG<T> where T: Read<u8> + Write<u8>  {} // this is wrong don't do this. Use a mutex and Arc<Mutex<SBG<T>>>! 


/**
 * To be implemented 
 */
#[no_mangle]
#[feature(c_variadic)]
pub unsafe extern "C" fn sbgPlatformDebugLogMsg( pFileName: *const ::core::ffi::c_char, pFunctionName: *const ::core::ffi::c_char, line: u32, pCategory: *const ::core::ffi::c_char, logType: _SbgDebugLogType, errorCode: _SbgErrorCode, pFormat: *const ::core::ffi::c_char, args: ...) {

}

/**
 * To be implemented 
 */
#[no_mangle] 
pub unsafe extern "C" fn sbgGetTime() -> u32 {
    300
}

#[no_mangle]
pub unsafe extern "C" fn sbgSleep(ms: u32) {

}