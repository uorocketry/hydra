use crate::types::ConfigSBG;
use atsamd_hal::clock::v2::gclk::Gclk0Id;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::dmac;
use atsamd_hal::dmac::{BufferPair, Transfer};
use atsamd_hal::gpio::{Pin, Reset, PA08, PA09};
use atsamd_hal::pac::{MCLK, RTC};
use atsamd_hal::prelude::_atsamd21_hal_time_U32Ext;
use atsamd_hal::rtc::Rtc;
use atsamd_hal::sercom::uart::Uart;
use atsamd_hal::sercom::{uart, Sercom, Sercom0};
use rtic::Mutex;
use sbg_rs::sbg;
use sbg_rs::sbg::{SBG, SBG_BUFFER_SIZE};

pub static mut BUF_DST: SBGBuffer = &mut [0; SBG_BUFFER_SIZE];
pub static mut BUF_DST2: SBGBuffer = &mut [0; SBG_BUFFER_SIZE];

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
        let pads_sbg = uart::Pads::<Sercom0, _>::default().rx(rx).tx(tx);
        let uart_sbg = ConfigSBG::new(&mclk, sercom0, pads_sbg, pclk_sercom0.freq())
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

        let sbg: sbg::SBG = sbg::SBG::new(sbg_tx, rtc);

        SBGManager {
            sbg_device: sbg,
            buf_select: false,
            xfer,
        }
    }
}

/**
 * Handles the DMA interrupt.
 * Handles the SBG data.
 * Logs data to the SD card.
 */
pub fn sbg_dma(mut cx: crate::app::sbg_dma::Context) {
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

            cx.shared.sbg_data.lock(|(sbg_long_data, sbg_short_data)| {
                (*sbg_long_data, *sbg_short_data) = sbg.sbg_device.read_data(buf);
            });
            Ok(())
        });
    }
}

/// This is a hack to get the linker to not complain about missing symbols.
#[no_mangle]
pub extern "C" fn _sbrk() {}

#[no_mangle]
pub extern "C" fn _write() {}

#[no_mangle]
pub extern "C" fn _close() {}

#[no_mangle]
pub extern "C" fn _lseek() {}

#[no_mangle]
pub extern "C" fn _read() {}

#[no_mangle]
pub extern "C" fn _fstat() {}

#[no_mangle]
pub extern "C" fn _isatty() {}

#[no_mangle]
pub extern "C" fn _exit() {}

#[no_mangle]
pub extern "C" fn _open() {}

#[no_mangle]
pub extern "C" fn _kill() {}

#[no_mangle]
pub extern "C" fn _getpid() {}
