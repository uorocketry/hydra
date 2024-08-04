use crate::types::SBGBuffer;
use core::alloc::{GlobalAlloc, Layout};
use core::ffi::c_void;
use core::mem::size_of;
use core::ptr;
// use atsamd_hal::time::*;
use crate::app::sbg_flush;
use crate::app::sbg_get_time;
use crate::app::sbg_handle_data;
use crate::app::sbg_sd_task as sbg_sd;
use crate::app::sbg_write_data;
use common_arm::spawn;
use core::{mem, mem::MaybeUninit};
use defmt::info;
use embedded_alloc::Heap;
use rtic::Mutex;
use sbg_rs::sbg;
use sbg_rs::sbg::{CallbackData, SBG, SBG_BUFFER_SIZE};
use stm32h7xx_hal::dma::dma::StreamX;
use stm32h7xx_hal::dma::{
    dma::{DmaConfig, StreamsTuple},
    PeripheralToMemory, Transfer,
};
use stm32h7xx_hal::gpio::Alternate;
use stm32h7xx_hal::gpio::Pin;
use stm32h7xx_hal::rtc::Rtc;
use stm32h7xx_hal::serial::Rx;

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
            StreamX<stm32h7xx_hal::pac::DMA1, 0>,
            Rx<stm32h7xx_hal::pac::UART4>,
            stm32h7xx_hal::dma::PeripheralToMemory,
            MaybeUninit<[u8;SBG_BUFFER_SIZE]>,
            stm32h7xx_hal::dma::DBTransfer,
        >,
    >,
}

impl SBGManager {
    pub fn new(
        serial: stm32h7xx_hal::serial::Serial<stm32h7xx_hal::pac::UART4>,
        mut stream_tuple: StreamsTuple<stm32h7xx_hal::pac::DMA1>,
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

        let (sbg_tx, sbg_rx) = serial.split();

        // TODO: This could be wrong. It's a bit of a guess.
        let sbg_buffer: &'static mut [u8; SBG_BUFFER_SIZE] = {
            let buf: &mut [MaybeUninit<u8>; SBG_BUFFER_SIZE] =
                unsafe { &mut *(core::ptr::addr_of_mut!(SBG_BUFFER) as *mut _) };
            for (i, value) in buf.iter_mut().enumerate() {
                unsafe { value.as_mut_ptr().write(i as u8) };
            }
            unsafe { SBG_BUFFER.assume_init_mut() }
        };

        let config = DmaConfig::default().memory_increment(true);
        let transfer: Transfer<
            StreamX<stm32h7xx_hal::pac::DMA1, 0>,
            Rx<stm32h7xx_hal::pac::UART4>,
            stm32h7xx_hal::dma::PeripheralToMemory,
            &mut [u8],
            stm32h7xx_hal::dma::DBTransfer,
        > = Transfer::init(stream_tuple.0, sbg_rx, &mut sbg_buffer[..], None, config);

        transfer.start(|serial| {
            serial.enable_dma_rx();
        });

        let sbg: sbg::SBG = sbg::SBG::new(
            |data| {
                sbg_handle_data::spawn(data).ok();
            },
            |data| {
                sbg_write_data::spawn(data).ok();
            },
            || sbg_get_time::spawn(),
            || {
                sbg_flush::spawn().ok();
            },
        );

        SBGManager {
            sbg_device: sbg,
            xfer: Some(transfer),
        }
    }
}

pub fn sbg_flush(mut cx: sbg_flush::Context) {
    cx.shared.sbg_manager.lock(|sbg| {
        sbg.sbg_device.flush();
    });
}
pub fn sbg_write_data(mut cx: sbg_write_data::Context, data: &[u8]) {
    cx.shared.sbg_manager.lock(|sbg| {
        for byte in data {
            sbg.sbg_device.write(*byte);
        }
    });
}

pub fn sbg_get_time(mut cx: sbg_get_time::Context) -> u32 {
    cx.shared
        .rtc
        .lock(|rtc| rtc.date_time().unwrap().and_utc().timestamp_subsec_millis())
}

pub fn sbg_handle_data(mut cx: sbg_handle_data::Context, data: CallbackData) {
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

pub fn sbg_sd_task(mut cx: crate::app::sbg_sd_task::Context, data: [u8; SBG_BUFFER_SIZE]) {
    cx.shared.sd_manager.lock(|manager| {
        if let Some(mut file) = manager.file.take() {
            cx.shared.em.run(|| {
                manager.write(&mut file, &data)?;
                Ok(())
            });
            manager.file = Some(file); // give the file back after use
        } else if let Ok(mut file) = manager.open_file("sbg.txt") {
            cx.shared.em.run(|| {
                manager.write(&mut file, &data)?;
                Ok(())
            });
            manager.file = Some(file);
        }
    });
}
/**
 * Handles the DMA interrupt.
 * Handles the SBG data.
 */
pub fn sbg_dma(cx: crate::app::sbg_dma::Context) {
    let sbg = cx.local.sbg_manager;

    match &mut sbg.xfer {
        Some(xfer) => if xfer.complete() {},
        None => {
            // it should be impossible to reach here.
            info!("None");
        }
    }
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
