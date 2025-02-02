use core::alloc::{GlobalAlloc, Layout};
use core::ffi::c_void;
use core::mem::size_of;
use core::ptr;
// use atsamd_hal::time::*;
use crate::app::sbg_flush;
use crate::app::sbg_handle_data;
// use crate::app::sbg_sd_task as sbg_sd;
use crate::app::sbg_write_data;
use crate::RTC;
use chrono::{NaiveDate, NaiveDateTime, NaiveTime};
use core::mem::MaybeUninit;
use defmt::{info, panic};
use embedded_alloc::Heap;
use heapless::Vec;
use messages::mavlink::embedded::{Read, Write};
use sbg_rs::sbg;
use sbg_rs::sbg::{CallbackData, SBG, SBG_BUFFER_SIZE};
use stm32h7xx_hal::dma::dma::StreamX;
use stm32h7xx_hal::dma::{
    dma::{DmaConfig, StreamsTuple},
    PeripheralToMemory, Transfer,
};
use stm32h7xx_hal::pac::UART4;
use stm32h7xx_hal::serial::{Rx, Tx};
// use cortex_m::{asm};
use rtic::Mutex;

//#[link_section = ".axisram.buffers"]
//static mut SBG_BUFFER: MayberUninit<[u8; SBG_BUFFER_SIZE]> = MaybeUninit::uninit();

#[link_section = ".axisram.buffers"]
pub static mut SBG_BUFFER: MaybeUninit<[u8; SBG_BUFFER_SIZE]> = MaybeUninit::uninit();

// Simple heap required by the SBG library
static HEAP: Heap = Heap::empty();

pub struct SBGManager {
    sbg_device: SBG,
    xfer: Option<
        Transfer<
            StreamX<stm32h7xx_hal::pac::DMA1, 1>,
            Rx<stm32h7xx_hal::pac::UART4>,
            stm32h7xx_hal::dma::PeripheralToMemory,
            &'static mut [u8; SBG_BUFFER_SIZE],
            stm32h7xx_hal::dma::DBTransfer,
        >,
    >,
    sbg_tx: Tx<UART4>,
}

impl SBGManager {
    pub fn new(
        mut serial: stm32h7xx_hal::serial::Serial<stm32h7xx_hal::pac::UART4>,
        stream_tuple: StreamsTuple<stm32h7xx_hal::pac::DMA1>,
        //mut dma_channel: dmac::Channel<dmac::Ch0, dmac::Ready>,
    ) -> Self {
        /* Initialize the Heap */
        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 1024;
            // TODO: Could add a link section here to memory.
            static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
        }

        let (sbg_tx, mut sbg_rx) = serial.split();

        // TODO: This could be wrong. It's a bit of a guess.
        // let sbg_buffer: &'static mut [u8; SBG_BUFFER_SIZE] = {
        //     let buf: &mut [MaybeUninit<u8>; SBG_BUFFER_SIZE] =
        //         unsafe { &mut *(core::ptr::addr_of_mut!(SBG_BUFFER) as *mut _) };
        //     for (i, value) in buf.iter_mut().enumerate() {
        //         unsafe { value.as_mut_ptr().write(i as u8) };
        //     }
        //     unsafe { SBG_BUFFER.assume_init_mut() }
        // };
        unsafe {
            // Convert an uninitialised array into an array of uninitialised
            let buf: &mut [core::mem::MaybeUninit<u8>; SBG_BUFFER_SIZE] =
                &mut *(core::ptr::addr_of_mut!(SBG_BUFFER) as *mut _);
            buf.iter_mut().for_each(|x| x.as_mut_ptr().write(0));
        }

        let config = DmaConfig::default().memory_increment(true).transfer_complete_interrupt(true);
        let mut transfer: Transfer<
            StreamX<stm32h7xx_hal::pac::DMA1, 1>,
            Rx<stm32h7xx_hal::pac::UART4>,
            PeripheralToMemory,
            &mut [u8; SBG_BUFFER_SIZE],
            stm32h7xx_hal::dma::DBTransfer,
        > = Transfer::init(
            stream_tuple.1,
            sbg_rx,
            unsafe { SBG_BUFFER.assume_init_mut() }, // Uninitialised memory
            None,
            config,
        );



        info!("Starting transfer");
        transfer.start(|serial| {
            serial.enable_dma_rx();
        
        });
        info!("Transfer started");

        while !transfer.get_transfer_complete_flag() {
            // info!("Transfer not complete");
        }
        
        let mut sbg: sbg::SBG = sbg::SBG::new(
            |data| {
                sbg_handle_data::spawn(data).ok();
            },
            |data| {
                sbg_write_data::spawn(data).ok();
            },
            || sbg_get_time(),
            || {
                sbg_flush::spawn().ok();
            },
        );
        sbg.read_data(&unsafe { SBG_BUFFER.assume_init_read() });
        SBGManager {
            sbg_device: sbg,
            xfer: Some(transfer),
            sbg_tx,
        }
    }
}

pub fn sbg_flush(cx: sbg_flush::Context<'_>) {
    // cx.shared.sbg_manager.lock(|sbg| {
    // sbg.sbg_tx
    // });
}
pub fn sbg_write_data(mut cx: sbg_write_data::Context<'_>, data: Vec<u8, SBG_BUFFER_SIZE>) {
    cx.shared.sbg_manager.lock(|sbg| {
        sbg.sbg_tx.write_all(data.as_slice());
    });
}

pub fn sbg_get_time() -> u32 {
    cortex_m::interrupt::free(|cs| {
        let mut rc = RTC.borrow(cs).borrow_mut();
        let rtc = rc.as_mut().unwrap();
        rtc.date_time()
            .unwrap_or(NaiveDateTime::new(
                NaiveDate::from_ymd_opt(2024, 1, 1).unwrap(),
                NaiveTime::from_hms_milli_opt(0, 0, 0, 0).unwrap(),
            ))
            .and_utc()
            .timestamp_subsec_millis()
    })
}

pub fn sbg_handle_data(mut cx: sbg_handle_data::Context<'_>, data: CallbackData) {
    cx.shared.data_manager.lock(|manager| match data {
        CallbackData::UtcTime(x) => manager.utc_time = Some(x),
        CallbackData::Air(x) => manager.air = Some(x),
        CallbackData::EkfQuat(x) => manager.ekf_quat = Some(x),
        CallbackData::EkfNav(x) => manager.ekf_nav = Some(x),
        CallbackData::Imu(x) => manager.imu = Some(x),
        CallbackData::GpsVel(x) => manager.gps_vel = Some(x),
        CallbackData::GpsPos(x) => manager.gps_pos = Some(x),
    });
}

/**
 * Handles the DMA interrupt.
 * Handles the SBG data.
 */
pub fn sbg_dma(mut cx: crate::app::sbg_dma::Context) {
    info!("DMA");
    cx.shared.sbg_manager.lock(|sbg| {
        match &mut sbg.xfer {
            Some(xfer) => {
                if xfer.get_transfer_complete_flag() {
                    let data = unsafe { SBG_BUFFER.assume_init_read() };
                    xfer.clear_transfer_complete_interrupt();
                    xfer.next_transfer(
                        unsafe { (*core::ptr::addr_of_mut!(SBG_BUFFER)).assume_init_mut() }, // Uninitialised memory
                    );
                    sbg.sbg_device.read_data(&data);
                }
            }
            None => {
                // it should be impossible to reach here.
                info!("None");
            }
        }
    });
}

/// Stored right before an allocation. Stores information that is needed to deallocate memory.
#[derive(Copy, Clone)]
struct AllocInfo {
    layout: Layout,
    ptr: *mut u8,
}

/// Custom malloc for the SBG library. This uses the HEAP object initialized at the start of the
/// [`SBGManager`]. The [`Layout`] of the allocation is stored right before the returned pointed,
/// which makes it possible to implement [`free`] without any other data structures.
#[no_mangle]
pub extern "C" fn malloc(size: usize) -> *mut c_void {
    if size == 0 {
        return ptr::null_mut();
    }

    // Get a layout for both the requested size
    let header_layout = Layout::new::<AllocInfo>();
    let requested_layout = Layout::from_size_align(size, 8).unwrap();
    let (layout, offset) = header_layout.extend(requested_layout).unwrap();

    // Ask the allocator for memory
    let orig_ptr = unsafe { HEAP.alloc(layout) };
    if orig_ptr.is_null() {
        return orig_ptr as *mut c_void;
    }

    // Compute the pointer that we will return
    let result_ptr = unsafe { orig_ptr.add(offset) };

    // Store the allocation information right before the returned pointer
    let info_ptr = unsafe { result_ptr.sub(size_of::<AllocInfo>()) as *mut AllocInfo };
    unsafe {
        info_ptr.write_unaligned(AllocInfo {
            layout,
            ptr: orig_ptr,
        });
    }

    result_ptr as *mut c_void
}

/// Custom free implementation for the SBG library. This uses the stored allocation information
/// right before the pointer to free up the resources.
///
/// SAFETY: The value passed to ptr must have been obtained from a previous call to [`malloc`].
#[no_mangle]
pub unsafe extern "C" fn free(ptr: *mut c_void) {
    assert!(!ptr.is_null());

    let info_ptr = unsafe { ptr.sub(size_of::<AllocInfo>()) as *const AllocInfo };
    let info = unsafe { info_ptr.read_unaligned() };
    unsafe {
        HEAP.dealloc(info.ptr, info.layout);
    }
}
