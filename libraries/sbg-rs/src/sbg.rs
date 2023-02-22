use core::borrow::Borrow;
use core::ffi::{c_char, c_void, VaListImpl, CStr};
use core::ptr;
use core::convert::Infallible;
use core::sync::atomic::AtomicU32;
use defmt::{error, info, warn, debug, trace};
use core::ptr::{null, null_mut};
use crate::bindings::{self, _SbgErrorCode_SBG_READ_ERROR, _SbgErrorCode_SBG_NO_ERROR, _SbgErrorCode_SBG_WRITE_ERROR, sbgEComProtocolSend, sbgEComProtocolPayloadConstruct, sbgEComBinaryLogWriteGpsRawData, sbgEComBinaryLogWriteEkfEulerData, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_ERROR, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_INFO, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_DEBUG, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_WARNING, EXIT_SUCCESS, EXIT_FAILURE, sbgEComCmdGetInfo, SbgEComDeviceInfo, sbgEComHandle, sbgEComCmdOutputSetConf, sbgEComSetReceiveLogCallback};
use crate::bindings::{_SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A, _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0, _SbgEComLog_SBG_ECOM_LOG_IMU_DATA, _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_8, _SbgEComLog_SBG_ECOM_LOG_EKF_EULER, _SbgInterface, SbgInterfaceHandle, _SbgErrorCode, SbgInterfaceReadFunc, sbgEComInit, _SbgEComHandle, _SbgEComProtocol, _SbgBinaryLogData, _SbgDebugLogType, _SbgEComDeviceInfo};
use embedded_hal::serial::{Read, Write};
use core::slice::{from_raw_parts, from_raw_parts_mut};
use messages::sensor::Sbg;

/**
 * Represents the number of milliseconds that have passed.
 * Overflows after roughly 600 hours. 
 */
pub static mut SBG_COUNT: AtomicU32 = AtomicU32::new(0);

struct UARTSBGInterface {
    interface: *mut bindings::SbgInterface
}

pub struct SBG<T> where T: Read<u8> + Write<u8> {
    UARTSBGInterface: UARTSBGInterface,
    serial_device: T,
    handle: *mut _SbgEComHandle,
    pub isInitialized: bool,
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

        /*
            The stack is not forever, the handle will not stay. 
            Justify the use of unsafe!
         */

        let mut x: u8 = 10;
        let pLargeBuffer: *mut u8 = &mut x;
        // Create some dummy data to be able to create the struct. 
        let mut protocol: _SbgEComProtocol = _SbgEComProtocol { pLinkedInterface: interface.interface, rxBuffer: [0;4096usize], rxBufferSize: 4096usize, discardSize: 16, nextLargeTxId: 16, pLargeBuffer, largeBufferSize: 16, msgClass: 0, msgId: 0, transferId: 0, pageIndex: 0, nrPages: 2 };
        let handle: *mut _SbgEComHandle = &mut _SbgEComHandle {protocolHandle: protocol, pReceiveLogCallback: Some(SBG::<T>::SbgEComReceiveLogFunc), pUserArg: null_mut(), numTrials: 3, cmdDefaultTimeOut: 500};
         // initialize with dummy data then pass the handle to the init to be consumed 
        unsafe {
            sbgEComInit(handle, interface.interface);
        }
        let mut isInitialized = false;
        SBG {
            UARTSBGInterface: interface,
            serial_device,
            handle,
            isInitialized,
        }
    }
    /**
     * Reads a data frame from the SBG. 
     */
    pub fn readData(&mut self) -> Sbg {
        unsafe {
            sbgEComHandle(self.handle);
        }
        Sbg { accel: 8.0, speed: 8.0, pressure: 8.0, height: 8.0 }
    }

    pub fn setup(&mut self) -> u32 {
        let mut errorCode: _SbgErrorCode = _SbgErrorCode_SBG_NO_ERROR;
		//
		// Showcase how to configure some output logs to 25 Hz, don't stop if there is an error
		//
		unsafe {
        // errorCode = sbgEComCmdOutputSetConf(self.handle, _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A, _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0, 3, _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_8 );
        
		// if errorCode != _SbgErrorCode_SBG_NO_ERROR
		// {
		// 	// warn!(errorCode, "Unable to configure SBG_ECOM_LOG_IMU_DATA log");
        //     info!("Unable to configure imu log");
		// }
		errorCode = sbgEComCmdOutputSetConf(self.handle, _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A, _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0, 6, _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_8 );
        }
		if errorCode != _SbgErrorCode_SBG_NO_ERROR
		{
			// warn!(errorCode, "Unable to configure SBG_ECOM_LOG_EKF_EULER log");
            info!("Unable to configure euler log");
		}

		//
		// Define callbacks for received data and display header
        // This should happen in the new method
		//
		unsafe {sbgEComSetReceiveLogCallback(self.handle, Some(SBG::<T>::SbgEComReceiveLogFunc), null_mut())};
		info!("Euler Angles display with estimated standard deviation - degrees\n");
        if errorCode == _SbgErrorCode_SBG_NO_ERROR {
            self.isInitialized = true;
        };
        errorCode
    }

    pub fn getAndPrintProductInfo(pHandle: *mut _SbgEComHandle) -> _SbgErrorCode {
        let mut errorCode: _SbgErrorCode = _SbgErrorCode_SBG_NO_ERROR;
        let mut pInfo: SbgEComDeviceInfo = _SbgEComDeviceInfo {
            productCode: [0;32],
            serialNumber: 0,
            calibationRev: 0,
            calibrationYear: 0,
            calibrationMonth: 0,
            calibrationDay: 0,
            hardwareRev: 0,
            firmwareRev: 0,
        };
        unsafe {
            errorCode = sbgEComCmdGetInfo(pHandle, &mut pInfo);
            errorCode
        } 
    }

    /**
     * Allows the SBG interface to read data from the serial ports. 
     */
    pub extern "C" fn SbgInterfaceReadFunc(pInterface: *mut _SbgInterface, pBuffer: *mut c_void, pBytesRead: *mut usize, mut bytesToRead: usize) -> _SbgErrorCode {
        // let mut array: &[u8] = (*pBuffer)._data;
        let serial: *mut T = unsafe {(*pInterface).handle.cast()};
        let mut bytesRead = unsafe {*pBytesRead};
        bytesRead = 0;
        let mut array: &mut [u8] = unsafe{from_raw_parts_mut(pBuffer as *mut u8, bytesToRead)};
        loop {
            if bytesToRead == bytesRead {
                break;
            }
            let result = unsafe{nb::block!(serial.as_mut().expect("Serial reference").read())};
            bytesRead = bytesRead + 1;

            match result {
                Ok(word) => array[bytesRead] = word,
                Err(_) => (),
            }
        }
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * Allows the SBG interface to write to the UART peripheral 
     */
    pub unsafe extern "C" fn SbgInterfaceWriteFunc(pInterface: *mut _SbgInterface, pBuffer: *const c_void, bytesToWrite: usize) -> _SbgErrorCode {
        let serial: *mut T = (*pInterface).handle.cast();
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
    pub extern "C" fn SbgEComReceiveLogFunc(pHandle: *mut _SbgEComHandle, msgClass: u32, msg: u8, pLogData: *const _SbgBinaryLogData, pUserArg: *mut c_void) -> _SbgErrorCode{
        if msgClass == _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0 {
            match msg {
                6 => info!("{}", unsafe{(*pLogData).ekfEulerData.euler[0]}), 
                _ => warn!("Unknown log type"),
            }
        }
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * To be implemented 
     */
    pub extern "C" fn SbgDestroyFunc(pInterface: *mut _SbgInterface) -> _SbgErrorCode{
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * To be implemented 
     */
    pub unsafe extern "C" fn SbgFlushFunc(pInterface: *mut _SbgInterface, flags: u32) -> _SbgErrorCode {
        let serial: *mut T = (*pInterface).handle.cast();
        let result = serial.as_mut().expect("Serial flush failed.").flush();
        match result {
            Ok(_) => return _SbgErrorCode_SBG_NO_ERROR,
            Err(_) => return _SbgErrorCode_SBG_READ_ERROR,
        }
    }

    /**
     * To be implemented 
     */
    pub unsafe extern "C" fn SbgSetSpeedFunc(pInterface: *mut _SbgInterface, speed: u32) -> _SbgErrorCode {
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * To be implemented 
     */
    pub unsafe extern "C" fn SbgGetSpeedFunc(pInterface: *const _SbgInterface) -> u32 {
        115200
    }

    /**
     * To be implemented 
     */
    pub unsafe extern "C" fn SbgDelayFunc(pInterface: *const _SbgInterface, numBytes: usize) -> u32 {
        200
    }
}

unsafe impl<T> Send for SBG<T> where T: Read<u8> + Write<u8>  {} 

/**
 * To be implemented 
 */
#[no_mangle]
#[feature(c_variadic)]
pub unsafe extern "C" fn sbgPlatformDebugLogMsg(pFileName: *const ::core::ffi::c_char, pFunctionName: *const ::core::ffi::c_char, line: u32, pCategory: *const ::core::ffi::c_char, logType: _SbgDebugLogType, errorCode: _SbgErrorCode, pFormat: *const ::core::ffi::c_char, args: ...) {
    // using defmt logs
    let file = CStr::from_ptr(pFileName).to_str().unwrap();
    let function = CStr::from_ptr(pFunctionName).to_str().unwrap();
    let category = CStr::from_ptr(pCategory).to_str().unwrap();
    let format = CStr::from_ptr(pFormat).to_str().unwrap();
    // let mut arg_message = *"";
    // for _ in 0..n {
    //     arg_message = arg_message + *args.arg::<&str>();
    // }
    // let message = format_args!("{} {}", format, arg_message);
    match logType {
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_ERROR => error!("SBG Error"),
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_WARNING => warn!("SBG Warning"),
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_INFO => info!("SBG Info"),
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_DEBUG => debug!("SBG Debug"),
        _ => trace!(""),
    };
}

/**
 * Returns the number of milliseconds that have passed. 
 */
#[no_mangle] 
pub unsafe extern "C" fn sbgGetTime() -> u32 {
    *SBG_COUNT.get_mut()
}


/**
 * Sleeps the sbg execution 
 */
#[no_mangle]
pub unsafe extern "C" fn sbgSleep(ms: u32) {
    let start_time = sbgGetTime();
    while (*SBG_COUNT.get_mut() - start_time) < ms {
        // do nothing 
    }
}