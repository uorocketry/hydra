use core::ffi::{c_char, c_void, VaListImpl};
use core::ptr;
use atsamd_hal::timer::TimerCounter2;
use nb::{block, Error};
use defmt::{error, info, warn, debug};
use core::ptr::{null, null_mut};
use crate::bindings::{self, _SbgErrorCode_SBG_READ_ERROR, _SbgErrorCode_SBG_NO_ERROR, _SbgErrorCode_SBG_WRITE_ERROR, sbgEComProtocolSend, sbgEComProtocolPayloadConstruct, sbgEComBinaryLogWriteGpsRawData, sbgEComBinaryLogWriteEkfEulerData, _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_ERROR, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_INFO, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_DEBUG, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_WARNING};
use crate::bindings::{_SbgInterface, SbgInterfaceHandle, _SbgErrorCode, SbgInterfaceReadFunc, sbgEComInit, _SbgEComHandle, _SbgEComProtocol, _SbgBinaryLogData, _SbgDebugLogType};
use embedded_hal::{serial, serial::Read, serial::Write, timer::CountDown, timer::Periodic};
use core::slice::{from_raw_parts, from_raw_parts_mut};
use atsamd_hal::rtc;
use atsamd_hal::pac::TC2;

/**
 * Represents the number of milliseconds that have passed.
 * Overflows after roughly 600 hours. 
 */
pub static mut SBG_COUNT: u32 = 0;

struct UARTSBGInterface {
    interface: *mut bindings::SbgInterface
}

pub struct SBG<T> where T: Read<u8> + Write<u8> {
    UARTSBGInterface: UARTSBGInterface,
    serial_device: T,
} 

impl<T> SBG<T> where T: Read<u8> + Write<u8> {
    /**
     * Creates a new SBG instance to control the desired UART peripheral. 
     */
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

    /**
     * Allows the SBG interface to read data from the serial ports. 
     * The ability to fill the new buffer must be implemented 
     */
    #[no_mangle]
    pub unsafe extern "C" fn SbgInterfaceReadFunc(pInterface: *mut _SbgInterface, pBuffer: *mut c_void, pBytesToRead: *mut usize, mut bytesRead: usize) -> _SbgErrorCode {
        // let mut array: &[u8] = (*pBuffer)._data;
        let serial: *mut T = *pInterface.cast();
        let mut counter: usize = 0;
        let bytesToRead = match pBytesToRead.as_ref() {
            None => 0,
            Some(num) => *num,
        };
        let mut array: &mut [u8] = from_raw_parts_mut(pBuffer as *mut u8, bytesToRead);
        loop {
            if counter == bytesToRead {
                break;
            }
            let result = nb::block!(serial.as_mut().expect("Serial reference").read());
            match result {
                Ok(word) => array[counter] = word,
                Err(_) => return _SbgErrorCode_SBG_READ_ERROR,
            }
            counter+=1;
        }
        bytesRead = counter;
        // fill the pBuffer with the temp buffer.
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * Allows the SBG interface to write to the UART peripheral 
     */
    #[no_mangle]
    pub unsafe extern "C" fn SbgInterfaceWriteFunc(pInterface: *mut _SbgInterface, pBuffer: *const c_void, bytesToWrite: usize) -> _SbgErrorCode {
        let serial: *mut T = *pInterface.cast();
        let mut array: &[u8] = from_raw_parts(pBuffer as *const u8, bytesToWrite);
        let mut counter: usize = 0;
        loop {
            if bytesToWrite == counter {
                break;
            }
            // The block is needed otherwise the operation will not complete. 
            let result = nb::block!(serial.as_mut().expect("Serial reference").write(array[counter]));
            match result {
                Ok(_) => counter+=1,
                Err(_) => return _SbgErrorCode_SBG_WRITE_ERROR,
            }
        }
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * Callback function for handling logs. 
     * To be implemented
     */
    #[no_mangle]
    pub unsafe extern "C" fn SbgEComReceiveLogFunc(pHandle: *mut _SbgEComHandle, msgClass: u32, msg: u8, pLogData: *const _SbgBinaryLogData, pUserArg: *mut c_void) -> _SbgErrorCode{
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * To be implemented 
     */
    #[no_mangle]
    pub unsafe extern "C" fn SbgDestroyFunc(pInterface: *mut _SbgInterface) -> _SbgErrorCode{
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * To be implemented 
     */
    #[no_mangle]
    pub unsafe extern "C" fn SbgFlushFunc(pInterface: *mut _SbgInterface, flags: u32) -> _SbgErrorCode {
        let serial: *mut T = *pInterface.cast();
        let result = serial.as_mut().expect("Serial flush failed.").flush();
        match result {
            Ok(_) => return _SbgErrorCode_SBG_NO_ERROR,
            Err(_) => return _SbgErrorCode_SBG_READ_ERROR,
        }
    }

    /**
     * To be implemented 
     */
    #[no_mangle]
    pub unsafe extern "C" fn SbgSetSpeedFunc(pInterface: *mut _SbgInterface, speed: u32) -> _SbgErrorCode {
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * To be implemented 
     */
    #[no_mangle]
    pub unsafe extern "C" fn SbgGetSpeedFunc(pInterface: *const _SbgInterface) -> u32 {
        9600
    }

    /**
     * To be implemented 
     */
    #[no_mangle]
    pub unsafe extern "C" fn SbgDelayFunc(pInterface: *const _SbgInterface, numBytes: usize) -> u32 {
        200
    }
}

/**
 * This is very wrong and a Arc<Mutex<SBG<T>>> should be used to facilitate the send!!!!!!
 */
unsafe impl<T> Send for SBG<T> where T: Read<u8> + Write<u8>  {} 


/**
 * To be implemented 
 */
#[no_mangle]
#[feature(c_variadic)]
pub unsafe extern "C" fn sbgPlatformDebugLogMsg(pFileName: *const ::core::ffi::c_char, pFunctionName: *const ::core::ffi::c_char, line: u32, pCategory: *const ::core::ffi::c_char, logType: _SbgDebugLogType, errorCode: _SbgErrorCode, pFormat: *const ::core::ffi::c_char, args: ...) {
    // using defmt logs
    match logType {
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_ERROR => error!("SBG Error"),
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_WARNING => warn!("SBG Warning"),
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_INFO => info!("SBG Info"),
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_DEBUG => debug!("SBG Debug"),
        _ => (),
    }
}

/**
 * Returns the number of milliseconds that have passed. 
 */
#[no_mangle] 
pub unsafe extern "C" fn sbgGetTime() -> u32 {
    SBG_COUNT
}



/**
 * To be implemented 
 */
#[no_mangle]
pub unsafe extern "C" fn sbgSleep(ms: u32) {
    
}
