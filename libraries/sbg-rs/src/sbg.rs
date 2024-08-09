use crate::bindings::{
    self, _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_WARNING, _SbgEComLog_SBG_ECOM_LOG_AIR_DATA,
    _SbgEComLog_SBG_ECOM_LOG_EKF_NAV, _SbgEComLog_SBG_ECOM_LOG_GPS1_POS,
    _SbgEComLog_SBG_ECOM_LOG_GPS1_VEL, _SbgEComLog_SBG_ECOM_LOG_UTC_TIME,
    _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40, _SbgErrorCode_SBG_NO_ERROR,
    _SbgErrorCode_SBG_NULL_POINTER, _SbgErrorCode_SBG_READ_ERROR, _SbgErrorCode_SBG_WRITE_ERROR,
    sbgEComCmdOutputSetConf, sbgEComHandle,
};
use crate::bindings::{
    _SbgBinaryLogData, _SbgDebugLogType, _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0, _SbgEComHandle,
    _SbgEComLog_SBG_ECOM_LOG_EKF_QUAT, _SbgEComLog_SBG_ECOM_LOG_IMU_DATA,
    _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A, _SbgEComProtocol, _SbgErrorCode, _SbgInterface,
};
use core::ffi::c_void;
use core::ptr::null_mut;
use core::slice::{from_raw_parts, from_raw_parts_mut};
use core::sync::atomic::AtomicUsize;
use defmt::{flush, debug, info, warn, error};
use embedded_hal::serial::Write;
use heapless::Deque;
use heapless::Vec;
use core::ffi::CStr;
use messages::sensor::*;

/**
 * Max buffer size for SBG messages.
 */
pub const SBG_BUFFER_SIZE: usize = 1024;

/**
 * Represents the index of the buffer that is currently being used.
 */
static mut BUF_INDEX: AtomicUsize = AtomicUsize::new(0);
/**
 * Points to the buffer that is currently being used.
 */
static mut BUF: &[u8; SBG_BUFFER_SIZE] = &[0; SBG_BUFFER_SIZE];

static mut DEQ: Deque<u8, 4096> = Deque::new();

static mut DATA_CALLBACK: Option<fn(CallbackData)> = None;

static mut SERIAL_WRITE_CALLBACK: Option<fn(Vec<u8, SBG_BUFFER_SIZE>)> = None;

static mut RTC_GET_TIME: Option<fn() -> u32> = None;

static mut SERIAL_FLUSH_CALLBACK: Option<fn()> = None;

pub enum CallbackData {
    UtcTime(UtcTime),
    Air(Air),
    EkfQuat(EkfQuat),
    EkfNav((EkfNav1, EkfNav2, EkfNavAcc)),
    Imu((Imu1, Imu2)),
    GpsVel((GpsVel, GpsVelAcc)),
    GpsPos((GpsPos1, GpsPos2, GpsPosAcc)),
}

struct UARTSBGInterface {
    interface: *mut bindings::SbgInterface,
}

pub struct SBG {
    UARTSBGInterface: UARTSBGInterface,
    handle: _SbgEComHandle,
    isInitialized: bool,
}

impl SBG {
    /**
     * Creates a new SBG instance.
     * Takes ownership of the serial device and RTC instance.
     */
    pub fn new(
        callback: fn(CallbackData),
        serial_write_callback: fn(Vec<u8, SBG_BUFFER_SIZE>),
        rtc_get_time: fn() -> u32,
        serial_flush_callback: fn(),
    ) -> Self {
        unsafe {
            DATA_CALLBACK = Some(callback);
            SERIAL_WRITE_CALLBACK = Some(serial_write_callback);
            RTC_GET_TIME = Some(rtc_get_time);
            SERIAL_FLUSH_CALLBACK = Some(serial_flush_callback);
        }
        // SAFETY: We are assigning the RTC instance to a static variable.
        // This is safe because we are the only ones who have access to it.
        let interface = UARTSBGInterface {
            interface: &mut _SbgInterface {
                handle: null_mut(), // SAFEY: No idea what I just did.
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
    pub fn read_data(&mut self, buffer: &[u8; SBG_BUFFER_SIZE]) {
        // SAFETY: We are assigning a static mut variable.
        // Buf can only be accessed from functions called by sbgEComHandle after this assignment.
        // unsafe { BUF = buffer };
        for i in buffer {
            unsafe {
                match DEQ.push_back(*i) {
                    Ok(_) => (),
                    Err(_) => warn!("Deque SBG Error"),
                }
            };
        }
        // SAFETY: We are assigning a static variable.
        // This is safe because are the only thread reading since SBG is locked.
        unsafe {
            *BUF_INDEX.get_mut() = 0;
        }
        // SAFETY: We are calling a C function.
        // This is safe because it is assumed the SBG library is safe.
        unsafe {
            sbgEComHandle(&mut self.handle);
        }
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
        // SAFETY: We are calling a C function.
        // This is safe because it is assumed the SBG library is safe.
        let errorCode: _SbgErrorCode = unsafe {
            sbgEComCmdOutputSetConf(
                &mut self.handle,
                _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A,
                _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0,
                _SbgEComLog_SBG_ECOM_LOG_GPS1_VEL,
                _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40,
            )
        };
        if errorCode != _SbgErrorCode_SBG_NO_ERROR {
            warn!("Unable to configure UTC Time logs to 40 cycles");
        }

        // SAFETY: We are calling a C function.
        // This is safe because it is assumed the SBG library is safe.
        let errorCode: _SbgErrorCode = unsafe {
            sbgEComCmdOutputSetConf(
                &mut self.handle,
                _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A,
                _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0,
                _SbgEComLog_SBG_ECOM_LOG_UTC_TIME,
                _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40,
            )
        };
        if errorCode != _SbgErrorCode_SBG_NO_ERROR {
            warn!("Unable to configure UTC Time logs to 40 cycles");
        }

        // SAFETY: We are calling a C function.
        // This is safe because it is assumed the SBG library is safe.
        let errorCode: _SbgErrorCode = unsafe {
            sbgEComCmdOutputSetConf(
                &mut self.handle,
                _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A,
                _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0,
                _SbgEComLog_SBG_ECOM_LOG_AIR_DATA,
                _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40,
            )
        };
        if errorCode != _SbgErrorCode_SBG_NO_ERROR {
            warn!("Unable to configure Air Data logs to 40 cycles");
        }

        // SAFETY: We are calling a C function.
        // This is safe because it is assumed the SBG library is safe.
        let errorCode = unsafe {
            sbgEComCmdOutputSetConf(
                &mut self.handle,
                _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A,
                _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0,
                _SbgEComLog_SBG_ECOM_LOG_EKF_QUAT,
                _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40,
            )
        };
        if errorCode != _SbgErrorCode_SBG_NO_ERROR {
            warn!("Unable to configure EKF Quat logs to 40 cycles");
        }
        // SAFETY: We are calling a C function.
        // This is safe because it is assumed the SBG library is safe.
        let errorCode = unsafe {
            sbgEComCmdOutputSetConf(
                &mut self.handle,
                _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A,
                _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0,
                _SbgEComLog_SBG_ECOM_LOG_EKF_NAV,
                _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40,
            )
        };
        if errorCode != _SbgErrorCode_SBG_NO_ERROR {
            warn!("Unable to configure EKF Nav logs to 40 cycles");
        }
        // SAFETY: We are calling a C function.
        // This is safe because it is assumed the SBG library is safe.
        let errorCode = unsafe {
            sbgEComCmdOutputSetConf(
                &mut self.handle,
                _SbgEComOutputPort_SBG_ECOM_OUTPUT_PORT_A,
                _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0,
                _SbgEComLog_SBG_ECOM_LOG_IMU_DATA,
                _SbgEComOutputMode_SBG_ECOM_OUTPUT_MODE_DIV_40,
            )
        };
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
    pub unsafe extern "C" fn SbgInterfaceReadFunc(
        _pInterface: *mut _SbgInterface,
        pBuffer: *mut c_void,
        pBytesRead: *mut usize,
        bytesToRead: usize,
    ) -> _SbgErrorCode {
        if pBuffer.is_null() {
            return _SbgErrorCode_SBG_NULL_POINTER;
        }
        if pBytesRead.is_null() {
            return _SbgErrorCode_SBG_NULL_POINTER;
        }
        // SAFETY: We are casting a c_void pointer to a u8 pointer and then creating a slice from it.
        // This is safe because we ensure pBuffer is valid, pBuffer is not accessed during the lifetime of this function,
        // and the SBGECom library ensures the buffer given is of the correct size.
        let array: &mut [u8] = unsafe { from_raw_parts_mut(pBuffer as *mut u8, bytesToRead) };
        let mut readBytes = 0;
        for i in 0..(bytesToRead) {
            if let Some(front) = DEQ.pop_front() {
                readBytes += 1;
                array[i] = front;
            } else {
                // info!("No item in dequeue");
                break;
            }
        }
        // info!("Bytes Read {}", readBytes);
        unsafe { *pBytesRead = readBytes };
        return _SbgErrorCode_SBG_NO_ERROR;
    }

    /**
     * Allows the SBG interface to write to the UART peripheral
     */
    pub unsafe extern "C" fn SbgInterfaceWriteFunc(
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

        // SAFETY: We are casting a c_void pointer to a u8 pointer and then creating a slice from it.
        // This is safe because we ensure pBuffer is valid, pBuffer is not accessed during the lifetime of this function,
        // and the SBGECom library ensures the buffer given is of the correct size.
        let array: &[u8] = unsafe { from_raw_parts(pBuffer as *const u8, bytesToWrite) };
        let vec = array.iter().copied().collect::<Vec<u8, SBG_BUFFER_SIZE>>();
        match unsafe { SERIAL_WRITE_CALLBACK } {
            Some(callback) => callback(vec),
            None => return _SbgErrorCode_SBG_WRITE_ERROR,
        }
        // let mut counter: usize = 0;
        // loop {
        //     if bytesToWrite == counter {
        //         break;
        //     }
        //     // SAFETY: We are accessing a Uart Peripheral pointer.
        //     // This is safe because we ensure that the pointer is not accessed during the lifetime of this function.
        //     match unsafe { SERIAL_WRITE_CALLBACK } {
        //         Some(callback) => callback(&array[counter..counter + 1]),
        //         None => return _SbgErrorCode_SBG_WRITE_ERROR,
        //     }
        // }
        _SbgErrorCode_SBG_NO_ERROR
    }

    /**
     * Callback function for handling logs.
     */
    pub unsafe extern "C" fn SbgEComReceiveLogFunc(
        _pHandle: *mut _SbgEComHandle,
        msgClass: u32,
        msg: u32,
        pLogData: *const _SbgBinaryLogData,
        _pUserArg: *mut c_void,
    ) -> _SbgErrorCode {
        if pLogData.is_null() {
            return _SbgErrorCode_SBG_NULL_POINTER;
        }

        // SAFETY: DATA_CALLBACK is set once, before this function is called,
        // so no race conditions can happen.
        if let Some(callback) = unsafe { DATA_CALLBACK } {
            if msgClass == _SbgEComClass_SBG_ECOM_CLASS_LOG_ECOM_0 {
                // SAFETY: pLogData is not null, and we are checking the union flag before accessing it
                unsafe {
                    match msg {
                        _SbgEComLog_SBG_ECOM_LOG_AIR_DATA => {
                            callback(CallbackData::Air((*pLogData).airData.into()))
                        }
                        _SbgEComLog_SBG_ECOM_LOG_EKF_QUAT => {
                            callback(CallbackData::EkfQuat((*pLogData).ekfQuatData.into()))
                        }
                        _SbgEComLog_SBG_ECOM_LOG_IMU_DATA => {
                            callback(CallbackData::Imu((*pLogData).imuData.into()))
                        }
                        _SbgEComLog_SBG_ECOM_LOG_EKF_NAV => {
                            callback(CallbackData::EkfNav((*pLogData).ekfNavData.into()))
                        }
                        _SbgEComLog_SBG_ECOM_LOG_GPS1_POS => {
                            callback(CallbackData::GpsPos((*pLogData).gpsPosData.into()))
                        }
                        _ => (),
                    }
                }
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
    pub unsafe extern "C" fn SbgFlushFunc(
        pInterface: *mut _SbgInterface,
        _flags: u32,
    ) -> _SbgErrorCode {
        if pInterface.is_null() {
            return _SbgErrorCode_SBG_NULL_POINTER;
        }
        // SAFETY: We are casting a c_void pointer to a Uart peripheral pointer.
        // This is safe because we only have one sbg object and we ensure that
        // the peripheral pointer is not accessed during the lifetime of this function.
        match unsafe { SERIAL_FLUSH_CALLBACK } {
            Some(callback) => callback(),
            None => return _SbgErrorCode_SBG_WRITE_ERROR,
        }
        _SbgErrorCode_SBG_NO_ERROR
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

// SAFETY: No one besides us has the raw pointer to the SBG struct.
// We can safely transfer the SBG struct between threads.
unsafe impl Send for SBG {}

/**
 * Logs the message to the console.
 * Needs to be updated to handle the Variadic arguments.
 */
#[no_mangle]
pub unsafe extern "C" fn sbgPlatformDebugLogMsg(
    pFileName: *const ::core::ffi::c_char,
    pFunctionName: *const ::core::ffi::c_char,
    line: u32,
    pCategory: *const ::core::ffi::c_char,
    logType: _SbgDebugLogType,
    errorCode: _SbgErrorCode,
    pFormat: *const ::core::ffi::c_char,
) {
    if pFileName.is_null() || pFunctionName.is_null() || pCategory.is_null() || pFormat.is_null() {
        return;
    }
    // // SAFETY: We are converting a raw pointer to a CStr and then to a str.
    // // This is safe because we check if the pointers are null and
    // // the pointers can only be accessed during the lifetime of this function.
    let file = unsafe { CStr::from_ptr(pFileName).to_str().unwrap() };
    let function = unsafe { CStr::from_ptr(pFunctionName).to_str().unwrap() };
    let category = unsafe { CStr::from_ptr(pCategory).to_str().unwrap() };
    let format = unsafe { CStr::from_ptr(pFormat).to_str().unwrap() };

    info!("{}:{}:{}:{}:{}:{}", file, function, line, category, errorCode, format);

    match logType {
        // silently handle errors
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_ERROR => error!("SBG Error"),
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_WARNING => warn!("SBG Warning"),
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_INFO => info!("SBG Info "),
        _SbgDebugLogType_SBG_DEBUG_LOG_TYPE_DEBUG => debug!("SBG Debug "),
        _ => (),
    };
    flush();
}

/**
 * Returns the number of milliseconds that have passed.
 */
#[no_mangle]
pub extern "C" fn sbgGetTime() -> u32 {
    // SAFETY: We are accessing a static mut variable.
    // This is safe because this is the only place where we access the RTC.
    match unsafe { RTC_GET_TIME } {
        Some(get_time) => {
            get_time()
        }
        None => 0,
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
