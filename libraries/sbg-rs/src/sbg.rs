use crate::bindings::{
    self, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_DEBUG, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_INFO,
    _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_WARNING, _SbgEComLog_SBG_ECOM_LOG_AIR_DATA,
    _SbgEComLog_SBG_ECOM_LOG_EKF_NAV, _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40,
    _SbgErrorCode_SBG_NO_ERROR, _SbgErrorCode_SBG_NULL_POINTER, _SbgErrorCode_SBG_READ_ERROR,
    _SbgErrorCode_SBG_WRITE_ERROR, sbgEComCmdOutputSetConf, sbgEComHandle,
};
use crate::bindings::{
    _SbgBinaryLogData, _SbgDebugLogType, _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0, _SbgEComHandle,
    _SbgEComLog_SBG_ECOM_LOG_EKF_EULER, _SbgEComLog_SBG_ECOM_LOG_EKF_QUAT,
    _SbgEComLog_SBG_ECOM_LOG_IMU_DATA, _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A, _SbgEComProtocol,
    _SbgErrorCode, _SbgInterface,
};
use atsamd_hal as hal;
use core::ffi::{c_void, CStr};
use core::ptr::null_mut;
use core::slice::{from_raw_parts, from_raw_parts_mut};
use core::sync::atomic::AtomicUsize;
use defmt::{debug, flush, info, warn};
use embedded_hal::serial::Write;
use hal::gpio::{PA08, PA09, PB16, PB17};
use hal::sercom::uart::Duplex;
use hal::sercom::uart::{self, EightBit, Uart};
use hal::sercom::{IoSet1, Sercom0, Sercom5};
use messages::sensor::Sbg;
type Pads = uart::PadsFromIds<Sercom0, IoSet1, PA09, PA08>;
type PadsCDC = uart::PadsFromIds<Sercom5, IoSet1, PB17, PB16>;
type Config = uart::Config<Pads, EightBit>;

/**
 * Represents the index of the buffer that is currently being used.
 */
static mut BUF_INDEX: AtomicUsize = AtomicUsize::new(0);
/**
 * Points to the buffer that is currently being used.
 */
static mut BUF: &'static [u8; SBG_BUFFER_SIZE] = &[0; SBG_BUFFER_SIZE];

/**
 * Holds the RTC instance. This is used to get the current time.
 */
static mut RTC: Option<hal::rtc::Rtc<hal::rtc::Count32Mode>> = None;

/**
 * Holds the latest SBG data that was received.
 */
static mut DATA: Sbg = Sbg {
    accel_x: 0.0,
    accel_y: 0.0,
    accel_z: 0.0,
    velocity_n: 0.0,
    velocity_e: 0.0,
    velocity_d: 0.0,
    quant_w: 0.0,
    quant_x: 0.0,
    quant_y: 0.0,
    quant_z: 0.0,
    pressure: 0.0,
    height: 0.0,
    roll: 0.0,
    yaw: 0.0,
    pitch: 0.0,
    latitude: 0.0,
    longitude: 0.0,
};

/**
 * Max buffer size for SBG messages.
 */
const SBG_BUFFER_SIZE: usize = 4096;
struct UARTSBGInterface {
    interface: *mut bindings::SbgInterface,
}

pub struct SBG {
    UARTSBGInterface: UARTSBGInterface,
    serial_device: Uart<Config, uart::TxDuplex>,
    handle: _SbgEComHandle,
    isInitialized: bool,
}

impl SBG {
    /**
     * Creates a new SBG instance.
     * Takes ownership of the serial device and RTC instance.
     */
    pub fn new(
        mut serial_device: Uart<Config, uart::TxDuplex>,
        rtc: hal::rtc::Rtc<hal::rtc::Count32Mode>,
    ) -> Self {
        // Safe because we are in a static context
        unsafe { RTC = Some(rtc) };
        let interface = UARTSBGInterface {
            interface: &mut _SbgInterface {
                handle: &mut serial_device as *mut Uart<Config, uart::TxDuplex> as *mut c_void,
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
        let pLargeBuffer: *mut u8 = null_mut();
        let protocol: _SbgEComProtocol = _SbgEComProtocol {
            pLinkedInterface: interface.interface,
            rxBuffer: [0; 4096usize],
            rxBufferSize: 0,
            discardSize: 0,
            nextLargeTxId: 0,
            pLargeBuffer,
            largeBufferSize: 0,
            msgClass: 0,
            msgId: 0,
            transferId: 0,
            pageIndex: 0,
            nrPages: 0,
        };
        let handle: _SbgEComHandle = _SbgEComHandle {
            protocolHandle: protocol,
            pReceiveLogCallback: Some(SBG::SbgEComReceiveLogFunc),
            pUserArg: null_mut(),
            numTrials: 3,
            cmdDefaultTimeOut: 500,
        };
        let isInitialized = false;
        SBG {
            UARTSBGInterface: interface,
            serial_device,
            handle: handle,
            isInitialized,
        }
    }
    /**
     * Returns true if the SBG is initialized.
     */
    pub fn isInitialized(&self) -> bool {
        self.isInitialized
    }
    /**
     * Reads SBG data frames for a buffer and returns the most recent data.
     */
    pub fn readData(&mut self, buffer: &'static [u8; SBG_BUFFER_SIZE]) -> Sbg {
        unsafe { BUF = buffer };
        // safe because we are in an atomic context
        unsafe {
            *BUF_INDEX.get_mut() = 0;
        }
        // unsafe because we are calling a C function
        unsafe {
            sbgEComHandle(&mut self.handle);
        }
        // safe because DATA is not accessed by another task and we are on a single threaded system
        unsafe { DATA.clone() }
    }

    /**
     * Configures the SBG to output the following data
     * Air data
     * IMU data
     * Extended Kalman Filter Euler data
     * Extended Kalman Filter Quaternions
     * Extended Kalman Filter Navigation data
     */
    pub fn setup(&mut self) -> u32 {
        let mut errorCode: _SbgErrorCode = _SbgErrorCode_SBG_NO_ERROR;
        // unsafe because we are calling a C function
        unsafe {
            errorCode = sbgEComCmdOutputSetConf(
                &mut self.handle,
                _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A,
                _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0,
                _SbgEComLog_SBG_ECOM_LOG_AIR_DATA,
                _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40,
            );
        }
        if errorCode != _SbgErrorCode_SBG_NO_ERROR {
            warn!("Unable to configure Air Data logs to 40 cycles");
        }
        // unsafe because we are calling a C function
        unsafe {
            errorCode = sbgEComCmdOutputSetConf(
                &mut self.handle,
                _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A,
                _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0,
                _SbgEComLog_SBG_ECOM_LOG_EKF_EULER,
                _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40,
            );
        }
        if errorCode != _SbgErrorCode_SBG_NO_ERROR {
            warn!("Unable to configure EKF Euler logs to 40 cycles");
        }
        // unsafe because we are calling a C function
        unsafe {
            errorCode = sbgEComCmdOutputSetConf(
                &mut self.handle,
                _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A,
                _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0,
                _SbgEComLog_SBG_ECOM_LOG_EKF_QUAT,
                _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40,
            );
        }
        if errorCode != _SbgErrorCode_SBG_NO_ERROR {
            warn!("Unable to configure EKF Euler logs to 40 cycles");
        }
        // unsafe because we are calling a C function
        unsafe {
            errorCode = sbgEComCmdOutputSetConf(
                &mut self.handle,
                _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A,
                _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0,
                _SbgEComLog_SBG_ECOM_LOG_EKF_NAV,
                _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40,
            );
        }
        if errorCode != _SbgErrorCode_SBG_NO_ERROR {
            warn!("Unable to configure EKF Nav logs to 40 cycles");
        }
        // unsafe because we are calling a C function
        unsafe {
            errorCode = sbgEComCmdOutputSetConf(
                &mut self.handle,
                _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A,
                _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0,
                _SbgEComLog_SBG_ECOM_LOG_IMU_DATA,
                _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40,
            );
        }
        if errorCode != _SbgErrorCode_SBG_NO_ERROR {
            warn!("Unable to configure IMU logs to 40 cycles");
        } else {
            self.isInitialized = true;
        };
        errorCode
    }

    /**
     * Allows the SBG interface to read data from the serial ports.
     */
    pub extern "C" fn SbgInterfaceReadFunc(
        _pInterface: *mut _SbgInterface,
        pBuffer: *mut c_void,
        pBytesRead: *mut usize,
        mut bytesToRead: usize,
    ) -> _SbgErrorCode {
        if pBuffer.is_null() {
            return _SbgErrorCode_SBG_NULL_POINTER;
        }
        if pBytesRead.is_null() {
            return _SbgErrorCode_SBG_NULL_POINTER;
        }
        // Safe because we check if pBuffer is null and the SBGECom library ensures the buffer given is of the correct size
        unsafe {
            let array: &mut [u8] = from_raw_parts_mut(pBuffer as *mut u8, bytesToRead);
        // Could be unsafe
            let index = *BUF_INDEX.get_mut();
        
        if index + bytesToRead > SBG_BUFFER_SIZE {
            // Read what we can.
            bytesToRead = SBG_BUFFER_SIZE - index;
            if bytesToRead == 0 {
                // Safe because we check if pBytesRead is null
                    *pBytesRead = 0;
                return _SbgErrorCode_SBG_READ_ERROR; // no data
            }
            let end = bytesToRead + index;
            // Could be unsafe  
            array[0..bytesToRead - 1].copy_from_slice( &BUF[index..end - 1] );
            // Could be unsafe 
                *BUF_INDEX.get_mut() = index + bytesToRead;
            // Safe because we check if pBytesRead is null
                *pBytesRead = bytesToRead;
            return _SbgErrorCode_SBG_NO_ERROR;
        }
        let end = bytesToRead + index;
        // Could be unsafe
        array[0..bytesToRead - 1].copy_from_slice(&BUF[index..end - 1] );
        // Could be unsafe
            *BUF_INDEX.get_mut() = index + bytesToRead;
        
        // Safe because we check if pBytesRead is null
            *pBytesRead = bytesToRead;
        }
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * Allows the SBG interface to write to the UART peripheral
     */
    pub extern "C" fn SbgInterfaceWriteFunc(
        pInterface: *mut _SbgInterface,
        pBuffer: *const c_void,
        bytesToWrite: usize,
    ) -> _SbgErrorCode {
        if pInterface.is_null() {
            return _SbgErrorCode_SBG_NULL_POINTER;
        }
        if pBuffer.is_null() {
            return _SbgErrorCode_SBG_NULL_POINTER;
        }
        // Safe because we check if pInterface is null
        let serial: *mut Uart<Config, uart::TxDuplex> =
            unsafe { (*pInterface).handle as *mut Uart<Config, uart::TxDuplex> };
        // Safe because we check if pBuffer is null
        let array: &[u8] = unsafe { from_raw_parts(pBuffer as *const u8, bytesToWrite) };
        let mut counter: usize = 0;
        loop {
            if bytesToWrite == counter {
                break;
            }
            // The block is needed otherwise the operation will not complete.
            // Safe because serial is valid
            let result = unsafe { nb::block!(serial.as_mut().unwrap().write(array[counter])) };
            match result {
                Ok(_) => counter += 1,
                Err(_) => return _SbgErrorCode_SBG_WRITE_ERROR,
            }
        }
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * Callback function for handling logs.
     */
    pub extern "C" fn SbgEComReceiveLogFunc(
        _pHandle: *mut _SbgEComHandle,
        msgClass: u32,
        msg: u32,
        pLogData: *const _SbgBinaryLogData,
        _pUserArg: *mut c_void,
    ) -> _SbgErrorCode {
        if pLogData.is_null() {
            return _SbgErrorCode_SBG_NULL_POINTER;
        }
        if msgClass == _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0 {
            match msg {
                _SbgEComLog_SBG_ECOM_LOG_AIR_DATA => 
                // Safe because we check if pLogData is null, but DATA could be unsafe and may need a MUTEX. 
                unsafe {
                    DATA.pressure = (*pLogData).airData.pressureAbs;
                },
                _SbgEComLog_SBG_ECOM_LOG_EKF_EULER => 
                // Safe because we check if pLogData is null, but DATA could be unsafe and may need a MUTEX. 
                unsafe {
                    DATA.roll = (*pLogData).ekfEulerData.euler[0];
                    DATA.pitch = (*pLogData).ekfEulerData.euler[1];
                    DATA.yaw = (*pLogData).ekfEulerData.euler[2];
                },
                _SbgEComLog_SBG_ECOM_LOG_EKF_QUAT => 
                // Safe because we check if pLogData is null, but DATA could be unsafe and may need a MUTEX. 
                unsafe {
                    DATA.quant_w = (*pLogData).ekfQuatData.quaternion[0];
                    DATA.quant_x = (*pLogData).ekfQuatData.quaternion[1];
                    DATA.quant_y = (*pLogData).ekfQuatData.quaternion[2];
                    DATA.quant_z = (*pLogData).ekfQuatData.quaternion[3];
                },
                _SbgEComLog_SBG_ECOM_LOG_IMU_DATA => 
                // Safe because we check if pLogData is null, but DATA could be unsafe and may need a MUTEX. 
                unsafe {
                    DATA.accel_x = (*pLogData).imuData.accelerometers[0];
                    DATA.accel_y = (*pLogData).imuData.accelerometers[1];
                    DATA.accel_z = (*pLogData).imuData.accelerometers[2];
                },
                _SbgEComLog_SBG_ECOM_LOG_EKF_NAV => 
                // Safe because we check if pLogData is null, but DATA could be unsafe and may need a MUTEX. 
                unsafe {
                    DATA.latitude = (*pLogData).ekfNavData.position[0];
                    DATA.longitude = (*pLogData).ekfNavData.position[1];
                    DATA.height = (*pLogData).ekfNavData.position[2];
                    DATA.velocity_n = (*pLogData).ekfNavData.velocity[0];
                    DATA.velocity_e = (*pLogData).ekfNavData.velocity[1];
                    DATA.velocity_d = (*pLogData).ekfNavData.velocity[2];
                },
                _ => (),
            }
        }
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * The SBG interface does not need to be destroyed.
     */
    pub extern "C" fn SbgDestroyFunc(_pInterface: *mut _SbgInterface) -> _SbgErrorCode {
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * Flushes the UART peripheral.
     */
    pub extern "C" fn SbgFlushFunc(pInterface: *mut _SbgInterface, _flags: u32) -> _SbgErrorCode {
        if pInterface.is_null() {
            return _SbgErrorCode_SBG_NULL_POINTER;
        }
        // Safe because we check if pInterface is null
        let serial: *mut Uart<Config, Duplex> =
            unsafe { (*pInterface).handle as *mut Uart<Config, Duplex> };
        let result = unsafe { serial.as_mut().unwrap().flush() };
        match result {
            Ok(_) => return _SbgErrorCode_SBG_NO_ERROR,
            Err(_) => return _SbgErrorCode_SBG_READ_ERROR,
        }
    }

    /**
     * The baud rate is fixed to 115200 and hence this function does nothing.
     */
    pub extern "C" fn SbgSetSpeedFunc(
        _pInterface: *mut _SbgInterface,
        _speed: u32,
    ) -> _SbgErrorCode {
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * The baud rate is fixed to 115200
     */
    pub extern "C" fn SbgGetSpeedFunc(_pInterface: *const _SbgInterface) -> u32 {
        115200
    }

    /**
     * Optional method used to compute an expected delay to transmit/receive X bytes
     */
    pub extern "C" fn SbgDelayFunc(_pInterface: *const _SbgInterface, _numBytes: usize) -> u32 {
        501
    }
}

unsafe impl Send for SBG {}

/**
 * Logs the message to the console.
 * Needs to be updated to handle the Variadic arguments.
 */
#[no_mangle]
#[feature(c_variadic)]
pub unsafe extern "C" fn sbgPlatformDebugLogMsg(
    pFileName: *const ::core::ffi::c_char,
    pFunctionName: *const ::core::ffi::c_char,
    _line: u32,
    pCategory: *const ::core::ffi::c_char,
    logType: _SbgDebugLogType,
    _errorCode: _SbgErrorCode,
    pFormat: *const ::core::ffi::c_char,
    _args: ...
) {
    if pFileName.is_null() || pFunctionName.is_null() || pCategory.is_null() || pFormat.is_null() {
        return;
    }
    // Safe since we check that the pointers are not null
        let file = CStr::from_ptr(pFileName).to_str().unwrap();
        let function = CStr::from_ptr(pFunctionName).to_str().unwrap();
        let _category = CStr::from_ptr(pCategory).to_str().unwrap();
        let _format = CStr::from_ptr(pFormat).to_str().unwrap();
    
    match logType {
        // silently handle errors
        // _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_ERROR => error!("SBG Error {} {}", file, function),
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_WARNING => warn!("SBG Warning {} {}", file, function),
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_INFO => info!("SBG Info {} {}", file, function),
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_DEBUG => debug!("SBG Debug {} {}", file, function),
        _ => (),
    };
    flush(); 
}

/**
 * Returns the number of milliseconds that have passed.
 */
#[no_mangle]
pub extern "C" fn sbgGetTime() -> u32 {
    // Could be unsafe  
    unsafe {
        match &RTC {
            Some(x) => x.count32(),
            None => 0, // bad error handling but we can't panic, maybe we should force the timeout to be zero in the event there is no RTC.
        }
    }
}

/**
 * Sleeps the sbg execution
 */
#[no_mangle]
pub extern "C" fn sbgSleep(ms: u32) {
    let start_time = sbgGetTime();
    while (sbgGetTime() - start_time) < ms {
        // do nothing
    }
}
