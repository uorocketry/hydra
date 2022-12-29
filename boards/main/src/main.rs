#![no_std]
#![no_main]
use atsamd_hal as hal;
use atsamd_hal::gpio::*;
use atsamd_hal::pac;
use atsamd_hal::prelude::nb::block;
use atsamd_hal::sercom::uart::EightBit;
use atsamd_hal::sercom::uart::{Duplex, Uart};
use atsamd_hal::sercom::{uart, IoSet1};
use common_arm::*;
use core::sync::atomic::AtomicU8;
use defmt::info;
use embedded_sdmmc::File;
use hal::dmac;
use hal::dmac::BufferPair;
use hal::gpio::Pins;
use hal::gpio::PA14;
use hal::gpio::{Pin, PushPullOutput};
use hal::prelude::*;
use hal::sercom::Sercom0;
use hal::sercom::Sercom3;
use hal::time::Hertz;
use hal::{dmac::Transfer, sercom::Sercom};
use heapless::Vec;
use messages::sender::Sender::MainBoard;
use messages::sensor::{Sbg, Sensor};
use messages::*;
use panic_halt as _;
use postcard::to_vec_cobs;
use sbg_rs::sbg;
use systick_monotonic::*;
const SBG_BUFFER_SIZE: usize = 4096;
static mut BUF_DST: SBGBuffer = &mut [0; SBG_BUFFER_SIZE];
static mut BUF_DST2: SBGBuffer = &mut [0; SBG_BUFFER_SIZE];
type Pads = uart::PadsFromIds<Sercom3, IoSet1, PA23, PA22>;
type PadsSBG = uart::PadsFromIds<Sercom0, IoSet1, PA09, PA08>;
type Config = uart::Config<Pads, EightBit>;
type ConfigSBG = uart::Config<PadsSBG, EightBit>;
type SBGTransfer = dmac::Transfer<
    dmac::Channel<dmac::Ch0, dmac::Busy>,
    BufferPair<Uart<ConfigSBG, uart::RxDuplex>, SBGBuffer>,
>;
type SBGBuffer = &'static mut [u8; SBG_BUFFER_SIZE];

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        opt_xfer: SBGTransfer,
        buf_select: AtomicU8,
        sbg: sbg::SBG,
        sensor_data: Sbg,
    }

    #[local]
    struct Local {
        led: Pin<PA14, PushPullOutput>,
        uart: Uart<Config, Duplex>,
        sd: SdInterface,
        sbg_file: File,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut peripherals = cx.device;
        let core = cx.core;

        let mut clocks = hal::clock::GenericClockController::with_external_32kosc(
            peripherals.GCLK,
            &mut peripherals.MCLK,
            &mut peripherals.OSC32KCTRL,
            &mut peripherals.OSCCTRL,
            &mut peripherals.NVMCTRL,
        );

        let pins = Pins::new(peripherals.PORT);
        let led = pins.pa14.into_push_pull_output();

        /* SD config */
        let mclk = &mut peripherals.MCLK;
        clocks.configure_gclk_divider_and_source(
            pac::gclk::pchctrl::GEN_A::GCLK5,
            1,
            pac::gclk::genctrl::SRC_A::DFLL,
            false,
        );
        let gclk5 = clocks
            .get_gclk(pac::gclk::pchctrl::GEN_A::GCLK5)
            .expect("Could not get gclk 5.");
        let spi_clk = clocks
            .sercom1_core(&gclk5)
            .expect("Could not configure Sercom 1 clock.");
        let sercom = peripherals.SERCOM1;
        let cs = pins.pa18.into_push_pull_output();
        let sck = pins.pa17.into_push_pull_output();
        let miso = pins.pa19.into_push_pull_output();
        let mosi = pins.pa16.into_push_pull_output();
        let mut sd = SdInterface::new(mclk, sercom, spi_clk, cs, sck, miso, mosi);
        let sbg_file = sd.open_file("raw.txt").expect("Could not open file");
        /* End SD config */
        /* Radio config */
        clocks.configure_gclk_divider_and_source(
            pac::gclk::pchctrl::GEN_A::GCLK2,
            1,
            pac::gclk::genctrl::SRC_A::DFLL,
            false,
        );
        let gclk2 = clocks
            .get_gclk(pac::gclk::pchctrl::GEN_A::GCLK2)
            .expect("Could not get gclk 2.");
        let uart_clk = clocks
            .sercom3_core(&gclk2)
            .expect("Could not configure Sercom 1 clock.");

        let pads = uart::Pads::<hal::sercom::Sercom3, _>::default()
            .rx(pins.pa23)
            .tx(pins.pa22);
        let uart = Config::new(
            &peripherals.MCLK,
            peripherals.SERCOM3,
            pads,
            uart_clk.freq(),
        )
        .baud(
            57600.hz(),
            uart::BaudMode::Fractional(uart::Oversampling::Bits16),
        )
        .enable();
        /* End Radio config */

        /* SBG config */
        clocks.configure_gclk_divider_and_source(
            pac::gclk::pchctrl::GEN_A::GCLK3,
            1,
            pac::gclk::genctrl::SRC_A::DFLL,
            false,
        );
        let gclk3 = clocks
            .get_gclk(pac::gclk::pchctrl::GEN_A::GCLK3)
            .expect("Could not get gclk 2.");

        let sbg_clk = clocks
            .sercom0_core(&gclk3)
            .expect("Could not configure Sercom 0 clock.");

        let padsSBG = uart::Pads::<Sercom0, _>::default()
            .rx(pins.pa09)
            .tx(pins.pa08);
        let uart_sbg = ConfigSBG::new(
            &peripherals.MCLK,
            peripherals.SERCOM0,
            padsSBG,
            sbg_clk.freq(),
        )
        .baud(
            115200.hz(),
            uart::BaudMode::Fractional(uart::Oversampling::Bits8),
        )
        .enable();

        let (sbg_rx, sbg_tx) = uart_sbg.split();

        /* DMAC config */
        let mut dmac = dmac::DmaController::init(peripherals.DMAC, &mut peripherals.PM);
        let channels = dmac.split();
        let mut chan0 = channels.0.init(dmac::PriorityLevel::LVL3);
        chan0
            .as_mut()
            .enable_interrupts(dmac::InterruptFlags::new().with_tcmpl(true));
        let xfer = Transfer::new(chan0, sbg_rx, unsafe { &mut *BUF_DST }, false)
            .expect("DMA err")
            .begin(
                hal::sercom::Sercom0::DMA_RX_TRIGGER,
                dmac::TriggerAction::BURST,
            );
        /* End DMAC config */

        // There is a bug within the HAL that improperly configures the RTC
        // in count32 mode. This is circumvented by first using clock mode then
        // converting to count32 mode.
        let rtc_temp = hal::rtc::Rtc::clock_mode(peripherals.RTC, 1024.hz(), &mut peripherals.MCLK);
        let mut rtc = rtc_temp.into_count32_mode();
        rtc.set_count32(0);

        sbg_init::spawn().ok();
        state_send::spawn().ok();
        sensor_send::spawn().ok();
        blink::spawn().ok();
        let sysclk: Hertz = clocks.gclk0().into();
        let mono = Systick::new(core.SYST, sysclk.0);

        let sbg: sbg::SBG = sbg::SBG::new(sbg_tx, rtc);
        (
            Shared {
                em: ErrorManager::new(),
                opt_xfer: xfer,
                buf_select: AtomicU8::new(0),
                sbg,
                sensor_data: Sbg {
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
                },
            },
            Local {
                led,
                uart,
                sd,
                sbg_file,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    /**
     * Setups the SBG to output the appropriate data.
     */
    #[task(priority = 3, shared = [sbg])]
    fn sbg_init(mut cx: sbg_init::Context) {
        cx.shared.sbg.lock(|sbg| {
            sbg.setup();
        });
    }

    /**
     * Handles the DMA interrupt.
     * Handles the SBG data.
     * Logs data to the SD card.
     */
    #[task(binds = DMAC_0, local = [sd, sbg_file], shared = [sensor_data, opt_xfer, buf_select, sbg, &em])]
    fn dmac0(mut cx: dmac0::Context) {
        // This can be optimized to reduce CPU cycles
        cx.shared.opt_xfer.lock(|xfer| {
            cx.shared.sbg.lock(|sbg| {
                if xfer.complete() {
                    let buf_select = cx
                        .shared
                        .buf_select
                        .lock(|buf_select| *buf_select.get_mut());
                    cx.shared.em.run(|| {
                        match buf_select {
                            0 => {
                                let buf: &'static [u8; SBG_BUFFER_SIZE] =
                                    xfer.recycle_source(unsafe { BUF_DST }).expect("err");
                                cx.shared
                                    .sensor_data
                                    .lock(|sensor_data| *sensor_data = sbg.readData(buf));
                                cx.local.sd.write(&mut cx.local.sbg_file, buf)?;
                                cx.shared
                                    .buf_select
                                    .lock(|buf_select| *buf_select.get_mut() = 1)
                            }
                            1 => {
                                let buf: &'static [u8; SBG_BUFFER_SIZE] =
                                    xfer.recycle_source(unsafe { BUF_DST2 }).expect("err");
                                cx.shared
                                    .sensor_data
                                    .lock(|sensor_data| *sensor_data = sbg.readData(buf));
                                cx.local.sd.write(&mut cx.local.sbg_file, buf)?;
                                cx.shared
                                    .buf_select
                                    .lock(|buf_select| *buf_select.get_mut() = 0)
                            }
                            _ => (),
                        }
                        Ok(())
                    })
                }
            });
        });
    }

    /**
     * Sends a message to the radio over UART.
     */
    #[task(capacity = 10, local = [uart], shared = [&em])]
    fn send_message(cx: send_message::Context, m: Message) {
        cx.shared.em.run(|| {
            let uart = cx.local.uart;

            let payload: Vec<u8, 255> = to_vec_cobs(&m)?;

            for x in payload {
                block!(uart.write(x))?;
            }

            info!("Sent message: {:?}", m);

            Ok(())
        });
    }

    /**
     * Sends the state of the system.
     */
    #[task(shared = [&em])]
    fn state_send(cx: state_send::Context) {
        let em = cx.shared.em;

        let state = State {
            status: Status::Uninitialized,
            has_error: em.has_error(),
            voltage: 12.1,
        };

        let message = Message::new(&monotonics::now(), MainBoard, state);

        cx.shared.em.run(|| {
            spawn!(send_message, message)?;

            Ok(())
        });

        spawn_after!(state_send, 10.secs()).ok();
    }

    /**
     * Sends information about the sensors.
     */
    #[task(shared = [sensor_data, &em])]
    fn sensor_send(mut cx: sensor_send::Context) {
        cx.shared.sensor_data.lock(|sensor_data| {
            cx.shared.em.run(|| {
                let message = Message::new(
                    &monotonics::now(),
                    MainBoard,
                    Sensor::new(9, sensor_data.clone()),
                );

                spawn!(send_message, message)?;

                Ok(())
            });
        });

        spawn_after!(sensor_send, 2.secs()).ok();
    }

    /**
     * Simple blink task to test the system.
     */
    #[task(local = [led], shared = [&em])]
    fn blink(cx: blink::Context) {
        cx.shared.em.run(|| {
            cx.local.led.toggle()?;

            let time = monotonics::now().duration_since_epoch().to_secs();
            info!("Seconds since epoch: {}", time);
            if cx.shared.em.has_error() {
                spawn_after!(blink, 200.millis())?;
            } else {
                spawn_after!(blink, 1.secs())?;
            }
            Ok(())
        });
    }
}

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
