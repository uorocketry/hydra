use core::ffi::{ c_void, VaListImpl, CStr};
use core::sync::atomic::AtomicU32;
use defmt::{error, info, warn, debug, trace};
use hal::gpio::{PA09, PA08, PB17, PB16};
use hal::sercom::uart::Duplex;
use hal::sercom::{Sercom0, IoSet1, Sercom5};
use hal::sercom::uart::{self, EightBit, Uart};
use core::ptr::null_mut;
use crate::bindings::{self, _SbgErrorCode_SBG_READ_ERROR, _SbgErrorCode_SBG_NO_ERROR, _SbgErrorCode_SBG_WRITE_ERROR, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_ERROR, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_INFO, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_DEBUG, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_WARNING, sbgEComCmdGetInfo, SbgEComDeviceInfo, sbgEComHandle, sbgEComCmdOutputSetConf, sbgEComSetReceiveLogCallback, _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_20};
use crate::bindings::{_SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A, _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0, _SbgEComLog_SBG_ECOM_LOG_IMU_DATA, _SbgEComLog_SBG_ECOM_LOG_EKF_EULER, _SbgInterface, _SbgErrorCode, sbgEComInit, _SbgEComHandle, _SbgEComProtocol, _SbgBinaryLogData, _SbgDebugLogType, _SbgEComDeviceInfo, _SbgEComLog_SBG_ECOM_LOG_EKF_QUAT, _SbgEComLog_SBG_ECOM_LOG_GPS1_POS};
use embedded_hal::serial::{Read, Write};
use core::slice::{from_raw_parts, from_raw_parts_mut};
use messages::sensor::Sbg;
use ring_buffer::ring_buffer::RingBuffer;
use atsamd_hal as hal;
type Pads = uart::PadsFromIds<Sercom0, IoSet1, PA09, PA08>;
type PadsCDC = uart::PadsFromIds<Sercom5, IoSet1, PB17, PB16>;
type Config = uart::Config<Pads, EightBit>;

/**
 * Represents the number of milliseconds that have passed.
 * Overflows after roughly 600 hours. 
 */
pub static mut SBG_COUNT: AtomicU32 = AtomicU32::new(0);
/**
 * Represents the ring buffer that is used to store the incoming data from the SBG.
 */
pub static mut SBG_RING_BUFFER: RingBuffer = RingBuffer::new();

struct UARTSBGInterface {
    interface: *mut bindings::SbgInterface
}

pub struct SBG {
    UARTSBGInterface: UARTSBGInterface,
    pub serial_device: Uart<Config, Duplex>,
    handle: _SbgEComHandle,
    pub isInitialized: bool,
} 

impl SBG {
    /**
     * Creates a new SBG instance to control the desired UART peripheral. 
     */
    pub fn new(mut serial_device: Uart<Config, Duplex>) -> Self {
        let interface = UARTSBGInterface {
            interface: &mut _SbgInterface {
                handle: &mut serial_device as *mut Uart<Config, Duplex> as *mut c_void,
                type_: 0,
                name: [0; 48],
                pDestroyFunc: Some(SBG::SbgDestroyFunc),
                pWriteFunc: Some(SBG::SbgInterfaceWriteFunc),
                pReadFunc: Some(SBG::SbgInterfaceReadFunc),
                pFlushFunc: Some(SBG::SbgFlushFunc),
                pSetSpeedFunc: Some(SBG::SbgSetSpeedFunc),
                pGetSpeedFunc: Some(SBG::SbgGetSpeedFunc),
                pDelayFunc: Some(SBG::SbgDelayFunc),
            },
        };

        /*
            The stack is not forever, the handle will not stay. 
            Justify the use of unsafe!
         */

        let pLargeBuffer: *mut u8 = null_mut();
        let mut protocol: _SbgEComProtocol = _SbgEComProtocol { pLinkedInterface: interface.interface, rxBuffer: [0;4096usize], rxBufferSize: 0, discardSize: 0, nextLargeTxId: 0, pLargeBuffer, largeBufferSize: 0, msgClass: 0, msgId: 0, transferId: 0, pageIndex: 0, nrPages: 0 };
        let mut handle: _SbgEComHandle = _SbgEComHandle {protocolHandle: protocol, pReceiveLogCallback: Some(SBG::SbgEComReceiveLogFunc), pUserArg: null_mut(), numTrials: 3, cmdDefaultTimeOut: 500};
        let mut isInitialized = false;
        SBG {
            UARTSBGInterface: interface,
            serial_device,
            handle: handle,
            isInitialized,
        }
    }
    /**
     * Reads a data frame from the SBG. 
     */
    pub fn readData(&mut self) -> Sbg {
        unsafe {
            sbgEComHandle(&mut self.handle);
        }
        Sbg { accel: 8.0, speed: 8.0, pressure: 8.0, height: 8.0 }
    }

    pub fn setup(&mut self) -> u32 {
        unsafe {
            sbgEComInit(&mut self.handle, self.UARTSBGInterface.interface);
        }
        let mut errorCode: _SbgErrorCode = _SbgErrorCode_SBG_NO_ERROR;
		unsafe {
            errorCode = sbgEComCmdOutputSetConf(&mut self.handle, _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A, _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0, _SbgEComLog_SBG_ECOM_LOG_GPS1_POS, _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_20 );
        }
        if errorCode != _SbgErrorCode_SBG_NO_ERROR
        {
            info!("Unable to configure GPS logs to 20 cycles");
        }

		// unsafe {
		//     errorCode = sbgEComCmdOutputSetConf(&mut self.handle, _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A, _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0, _SbgEComLog_SBG_ECOM_LOG_EKF_EULER, _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_20 );
        // }
		// if errorCode != _SbgErrorCode_SBG_NO_ERROR
		// {
        //     info!("Unable to configure Euler logs to 20 cycles");
		// }

        // unsafe {
        //     errorCode = sbgEComCmdOutputSetConf(&mut self.handle, _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A, _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0, _SbgEComLog_SBG_ECOM_LOG_IMU_DATA, _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40 );
        // }
        // if errorCode != _SbgErrorCode_SBG_NO_ERROR
        // {
        //     info!("Unable to configure IMU logs to 40 cycles");
        // }

		//
		// Define callbacks for received data and display header
        // This should happen in the new method
		//
		unsafe {sbgEComSetReceiveLogCallback(&mut self.handle, Some(SBG::SbgEComReceiveLogFunc), null_mut())};
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
        let mut bytesRead = unsafe {*pBytesRead};
        let mut array: &mut [u8] = unsafe{from_raw_parts_mut(pBuffer as *mut u8, bytesToRead)};
        bytesRead = 0;
        loop {
            if bytesRead >= bytesToRead {
                break
            }
            let result = unsafe {SBG_RING_BUFFER.pop()};
            match result {
                Ok(word) => array[bytesRead] = word,
                Err(_) => return _SbgErrorCode_SBG_READ_ERROR,
            }
            bytesRead = bytesRead + 1;
        }
        unsafe {*pBytesRead = bytesRead;}
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * Allows the SBG interface to write to the UART peripheral 
     */
    pub extern "C" fn SbgInterfaceWriteFunc(pInterface: *mut _SbgInterface, pBuffer: *const c_void, bytesToWrite: usize) -> _SbgErrorCode {
        let serial: *mut Uart<Config, Duplex> = unsafe {(*pInterface).handle as *mut Uart<Config, Duplex>};
        let mut array: &[u8] = unsafe{from_raw_parts(pBuffer as *const u8, bytesToWrite)};
        // unsafe{serial.write_bytes(array, bytesToWrite)};
        let mut counter: usize = 0;

        loop {
            if bytesToWrite == counter {
                break
            }
            // The block is needed otherwise the operation will not complete. 

            // write a u8 from the array
            let result = unsafe{nb::block!(serial.as_mut().unwrap().write(array[counter]))};
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
    pub extern "C" fn SbgEComReceiveLogFunc(pHandle: *mut _SbgEComHandle, msgClass: u32, msg: u32, pLogData: *const _SbgBinaryLogData, pUserArg: *mut c_void) -> _SbgErrorCode{
        if msgClass == _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0 {
            match msg {
                _SbgEComLog_SBG_ECOM_LOG_EKF_EULER => info!("Roll {}, Pitch {}, Yaw {}", unsafe{(*pLogData).ekfEulerData.euler[0]}, unsafe{(*pLogData).ekfEulerData.euler[1]}, unsafe{(*pLogData).ekfEulerData.euler[2]}), 
                _SbgEComLog_SBG_ECOM_LOG_EKF_QUAT => info!("Quat X {}, Y {}, Z {}, W {}", unsafe{(*pLogData).ekfQuatData.quaternion[0]}, unsafe{(*pLogData).ekfQuatData.quaternion[1]}, unsafe{(*pLogData).ekfQuatData.quaternion[2]}, unsafe{(*pLogData).ekfQuatData.quaternion[3]}),
                _SbgEComLog_SBG_ECOM_LOG_IMU_DATA => info!("Accel X {}, Y {}, Z {}", unsafe{(*pLogData).imuData.accelerometers[0]}, unsafe{(*pLogData).imuData.accelerometers[1]}, unsafe{(*pLogData).imuData.accelerometers[2]}),
                _SbgEComLog_SBG_ECOM_LOG_GPS1_POS => info!("GPS1 Lat {}, Long {}, Alt {}", unsafe{(*pLogData).gpsPosData.latitude}, unsafe{(*pLogData).gpsPosData.longitude}, unsafe{(*pLogData).gpsPosData.altitude}),
                _ => (),
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
    pub extern "C" fn SbgFlushFunc(pInterface: *mut _SbgInterface, flags: u32) -> _SbgErrorCode {
        let serial: *mut Uart<Config, Duplex> = unsafe {(*pInterface).handle as *mut Uart<Config, Duplex>};
        let result = unsafe {serial.as_mut().unwrap().flush()};
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
        501
    }
}

unsafe impl Send for SBG {} 

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
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_ERROR => error!("SBG Error {} {}", file, function),
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