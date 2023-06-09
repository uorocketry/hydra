use crate::types::{ConfigSBG, SBGBuffer, SBGTransfer};
use atsamd_hal::clock::v2::gclk::Gclk0Id;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::dmac;
use atsamd_hal::dmac::Transfer;
use atsamd_hal::gpio::{Pin, Reset, PA08, PA09};
use atsamd_hal::pac::{MCLK, RTC};
use atsamd_hal::prelude::_atsamd21_hal_time_U32Ext;
use atsamd_hal::rtc::Rtc;
use core::alloc::{GlobalAlloc, Layout};
use core::ffi::c_void;
use core::mem::size_of;
use core::ptr;

use crate::app::sbg_handle_data;
use atsamd_hal::sercom::{uart, Sercom, Sercom0};
use embedded_alloc::Heap;
use rtic::Mutex;
use sbg_rs::sbg;
use sbg_rs::sbg::{CallbackData, SBG, SBG_BUFFER_SIZE};

pub static mut BUF_DST: SBGBuffer = &mut [0; SBG_BUFFER_SIZE];
pub static mut BUF_DST2: SBGBuffer = &mut [0; SBG_BUFFER_SIZE];

// Simple heap required by the SBG library
static HEAP: Heap = Heap::empty();

pub struct SBGManager {
    sbg_device: SBG,
    xfer: SBGTransfer,
    buf_select: bool,
}

impl SBGManager {
    pub fn new(
        rx: Pin<PA09, Reset>,
        tx: Pin<PA08, Reset>,
        pclk_sercom0: Pclk<Sercom0, Gclk0Id>,
        mclk: &mut MCLK,
        sercom0: Sercom0,
        rtc: RTC,
        mut dma_channel: dmac::Channel<dmac::Ch0, dmac::Ready>,
    ) -> Self {
        /* Initialize the Heap */
        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 1024;
            static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
        }

        let pads_sbg = uart::Pads::<Sercom0, _>::default().rx(rx).tx(tx);
        let uart_sbg = ConfigSBG::new(mclk, sercom0, pads_sbg, pclk_sercom0.freq())
            .baud(
                115200.hz(),
                uart::BaudMode::Fractional(uart::Oversampling::Bits8),
            )
            .enable();

        let (sbg_rx, sbg_tx) = uart_sbg.split();

        /* DMAC config */
        dma_channel
            .as_mut()
            .enable_interrupts(dmac::InterruptFlags::new().with_tcmpl(true));
        let xfer = Transfer::new(dma_channel, sbg_rx, unsafe { &mut *BUF_DST }, false)
            .expect("DMA err")
            .begin(Sercom0::DMA_RX_TRIGGER, dmac::TriggerAction::BURST);

        // There is a bug within the HAL that improperly configures the RTC
        // in count32 mode. This is circumvented by first using clock mode then
        // converting to count32 mode.
        let rtc_temp = Rtc::clock_mode(rtc, 1024.hz(), mclk);
        let mut rtc = rtc_temp.into_count32_mode();
        rtc.set_count32(0);

        let sbg: sbg::SBG = sbg::SBG::new(sbg_tx, rtc, |data| {
            sbg_handle_data::spawn(data).ok();
        });

        SBGManager {
            sbg_device: sbg,
            buf_select: false,
            xfer,
        }
    }
}

pub fn sbg_handle_data(mut cx: sbg_handle_data::Context, data: CallbackData) {
    cx.shared.data_manager.lock(|manager| match data {
        CallbackData::UtcTime(x) => manager.utc_time = Some(x),
        CallbackData::Air(x) => manager.air = Some(x),
        CallbackData::EkfQuat(x) => manager.ekf_quat = Some(x),
        CallbackData::EkfNav(x) => manager.ekf_nav = Some(x),
        CallbackData::Imu(x) => manager.imu = Some(x),
        CallbackData::GpsVel(x) => manager.gps_vel = Some(x),
    });
}

/**
 * Handles the DMA interrupt.
 * Handles the SBG data.
 * Logs data to the SD card.
 */
pub fn sbg_dma(cx: crate::app::sbg_dma::Context) {
    let sbg = cx.local.sbg_manager;

    if sbg.xfer.complete() {
        cx.shared.em.run(|| {
            let buf = match sbg.buf_select {
                false => {
                    sbg.buf_select = true;
                    sbg.xfer.recycle_source(unsafe { &mut *BUF_DST })?
                }
                true => {
                    sbg.buf_select = false;
                    sbg.xfer.recycle_source(unsafe { &mut *BUF_DST2 })?
                }
            };

            sbg.sbg_device.read_data(buf);

            Ok(())
        });
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
