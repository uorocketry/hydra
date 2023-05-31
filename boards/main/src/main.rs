#![no_std]
#![no_main]
mod communication;
use communication::RadioDevice;
mod types;
use atsamd_hal as hal;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::sercom::uart;
use common_arm::mcan;
use common_arm::*;
use embedded_sdmmc::File;
use hal::dmac;
use hal::gpio::Pins;
use hal::gpio::PA14;
use hal::gpio::{Pin, PushPullOutput};
use hal::prelude::*;
use hal::sercom::Sercom0;
use hal::{dmac::Transfer, sercom::Sercom};
use mcan::messageram::SharedMemory;
use messages::sender::Sender::LogicBoard;
use messages::sensor::{Sbg, SbgShort, Sensor};
use messages::*;
use panic_halt as _;
use sbg_rs::sbg;
use systick_monotonic::*;
use types::*;

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {
    use super::*;
    #[shared]
    struct Shared {
        em: ErrorManager,
        opt_xfer: SBGTransfer,
        buf_select: bool,
        sbg: sbg::SBG,
        sbg_data: (Sbg, SbgShort),
        can0: communication::CanDevice0,
    }

    #[local]
    struct Local {
        led: Pin<PA14, PushPullOutput>,
        radio: RadioDevice,
        sd: SdInterface,
        sbg_file: File,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init(local = [
        #[link_section = ".can"]
        can_memory: SharedMemory<Capacities> = SharedMemory::new()
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut peripherals = cx.device;
        let core = cx.core;

        let (_, clocks, tokens) = atsamd_hal::clock::v2::clock_system_at_reset(
            peripherals.OSCCTRL,
            peripherals.OSC32KCTRL,
            peripherals.GCLK,
            peripherals.MCLK,
            &mut peripherals.NVMCTRL,
        );

        // SAFETY: Misusing the PAC API can break the system.
        // This is safe because we only steal the MCLK.
        let (_, _, _, mut mclk) = unsafe { clocks.pac.steal() };
        let gclk0 = clocks.gclk0;
        let pins = Pins::new(peripherals.PORT);
        let led = pins.pa14.into_push_pull_output();
        /* CAN config */
        // ! This is needs to be ran before calling the constructor.
        // ! This is because gclk0 does not play nice and cannot
        // ! be incremented twice in a single function since we
        // ! then no longer can return the gclk0.
        let (pclk_can, gclk0) = Pclk::enable(tokens.pclks.can0, gclk0);
        let (can0, gclk0) = communication::CanDevice0::new(
            pins.pa23.into_mode(),
            pins.pa22.into_mode(),
            pclk_can,
            clocks.ahbs.can0,
            peripherals.CAN0,
            gclk0,
            cx.local.can_memory,
            false,
        );
        /* SD config */
        let (pclk_sd, gclk0) =
            atsamd_hal::clock::v2::pclk::Pclk::enable(tokens.pclks.sercom1, gclk0);
        let mut sd = SdInterface::new(
            &mclk,
            peripherals.SERCOM1,
            pclk_sd.freq(),
            pins.pa18.into_push_pull_output(),
            pins.pa17.into_push_pull_output(),
            pins.pa19.into_push_pull_output(),
            pins.pa16.into_push_pull_output(),
        );
        let sbg_file = sd.open_file("raw.txt").expect("Could not open file");
        /* Radio config */
        let (radio, gclk0) = RadioDevice::new(
            tokens.pclks.sercom5,
            &mclk,
            peripherals.SERCOM5,
            pins.pb17,
            pins.pb16,
            gclk0,
        );

        /* SBG config */
        let (pclk_sbg, _gclk0) =
            atsamd_hal::clock::v2::pclk::Pclk::enable(tokens.pclks.sercom0, gclk0);
        let padsSBG = uart::Pads::<Sercom0, _>::default()
            .rx(pins.pa09)
            .tx(pins.pa08);
        let uart_sbg = ConfigSBG::new(&mclk, peripherals.SERCOM0, padsSBG, pclk_sbg.freq())
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

        // There is a bug within the HAL that improperly configures the RTC
        // in count32 mode. This is circumvented by first using clock mode then
        // converting to count32 mode.
        let rtc_temp = hal::rtc::Rtc::clock_mode(peripherals.RTC, 1024.hz(), &mut mclk);
        let mut rtc = rtc_temp.into_count32_mode();
        rtc.set_count32(0);

        let sbg: sbg::SBG = sbg::SBG::new(sbg_tx, rtc);

        /* Spawn tasks */
        state_send::spawn().ok();
        sensor_send::spawn().ok();
        blink::spawn().ok();
        
        let mono = Systick::new(core.SYST, 48000000);
        (
            Shared {
                em: ErrorManager::new(),
                opt_xfer: xfer,
                buf_select: false,
                sbg,
                sbg_data: (
                    Sbg {
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
                    SbgShort {
                        accel_y: 0.0,
                        pressure: 0.0,
                        height: 0.0,
                        quant_w: 0.0,
                        quant_x: 0.0,
                        quant_y: 0.0,
                        quant_z: 0.0,
                    },
                ),
                can0,
            },
            Local {
                led,
                radio,
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

    #[task(priority = 3, binds = CAN0, shared = [can0])]
    fn can0(mut cx: can0::Context) {
        cx.shared.can0.lock(|can| {
            can.process_data();
        });
    }

    // UNSAFE: This is a race condition between the SBG buffer and the sd card. 
    // This will be revisited in the future.
    #[task(capacity = 10, local = [sd, sbg_file], shared = [&em])]
    fn send_sd(mut cx: send_sd::Context, buf: &'static [u8]) {
        cx.shared.em.run(|| {
            cx.local.sd.write(&mut cx.local.sbg_file, buf)?;
            Ok(())
        });
    }

    /**
     * Handles the DMA interrupt.
     * Handles the SBG data.
     * Logs data to the SD card.
     */
    #[task(binds = DMAC_0, shared = [sbg_data, opt_xfer, buf_select, sbg, &em])]
    fn dmac0(mut cx: dmac0::Context) {
        cx.shared.opt_xfer.lock(|xfer| {
            if xfer.complete() {
                cx.shared.buf_select.lock(|buf_select| {
                    cx.shared.em.run(|| {
                        let buf = match *buf_select {
                            false => {
                                *buf_select = true;
                                xfer.recycle_source(unsafe { &mut *BUF_DST })?               
                            }
                            true => {
                                *buf_select = false;
                                xfer.recycle_source(unsafe { &mut *BUF_DST2 })?
                            }
                        };

                        cx.shared.sbg.lock(|sbg| {
                            cx.shared.sbg_data.lock(|(sbg_long_data, sbg_short_data)| {
                                (*sbg_long_data, *sbg_short_data) = sbg.read_data(buf);
                            });
                        });
                        Ok(())
                    });
                });
            }
        });
    }

    /**
     * Sends a message over CAN.
     */
    #[task(capacity = 10, local = [counter: u16 = 0], shared = [can0, &em])]
    fn send_can(mut cx: send_can::Context, m: Message) {
        cx.shared.em.run(|| {
            cx.shared.can0.lock(|can| can.send_message(m))?;
            Ok(())
        });
    }

    /**
     * Sends a message to the radio over UART.
     */
    #[task(capacity = 10, local = [radio], shared = [&em])]
    fn send_message(cx: send_message::Context, m: Message) {
        cx.shared.em.run(|| {
            cx.local.radio.send_message(m)?;
            Ok(())
        });
    }

    /**
     * Sends the state of the system.
     */
    #[task(shared = [&em])]
    fn state_send(cx: state_send::Context) {
        let em = cx.shared.em;
        let state = messages::State {
            status: messages::Status::Running,
            has_error: em.has_error(),
            voltage: 12.1,
        };

        let message = Message::new(&monotonics::now(), LogicBoard, state);

        cx.shared.em.run(|| {
            spawn!(send_message, message.clone())?;
            spawn!(send_can, message)?;
            Ok(())
        });

        spawn_after!(state_send, 5.secs()).ok();
    }

    /**
     * Sends information about the sensors.
     */
    #[task(shared = [sbg_data, &em])]
    fn sensor_send(mut cx: sensor_send::Context) {
        let (data_long_sbg, data_short_sbg) =
            cx.shared.sbg_data.lock(|(sbg_long_data, sbg_short_data)| {
                (sbg_long_data.clone(), sbg_short_data.clone())
            });
        let message_radio = Message::new(
            &monotonics::now(),
            LogicBoard,
            Sensor::new(9, data_long_sbg),
        );
        let message_can = Message::new(
            &monotonics::now(),
            LogicBoard,
            Sensor::new(9, data_short_sbg),
        );

        cx.shared.em.run(|| {
            spawn!(send_message, message_radio)?;

            spawn!(send_can, message_can)?;

            Ok(())
        });
        spawn_after!(sensor_send, 2.secs()).ok();
    }

    /**
     * Simple blink task to test the system.
     * Acts as a heartbeat for the system.
     */
    #[task(local = [led], shared = [&em])]
    fn blink(cx: blink::Context) {
        cx.shared.em.run(|| {
            cx.local.led.toggle()?;
            if cx.shared.em.has_error() {
                spawn_after!(blink, 200.millis())?;
            } else {
                spawn_after!(blink, 1.secs())?;
            }
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
