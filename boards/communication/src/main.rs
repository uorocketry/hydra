#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod types;

use atsamd_hal as hal;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::clock::v2::Source;
use atsamd_hal::dmac::DmaController;
use common_arm::mcan;
use common_arm::*;
use communication::Capacities;
use communication::RadioDevice;
use data_manager::DataManager;
use hal::dmac;
use hal::gpio::Pins;
use hal::gpio::PA14;
use hal::gpio::{Pin, PushPullOutput};
use hal::prelude::*;
use mcan::messageram::SharedMemory;
use messages::sender::Sender::LogicBoard;
use messages::sensor::Sensor;
use messages::*;
use panic_halt as _;
use systick_monotonic::*;

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        data_manager: DataManager,
        can0: communication::CanDevice0,
    }

    #[local]
    struct Local {
        led: Pin<PA14, PushPullOutput>,
        radio: RadioDevice,
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
        let pins = Pins::new(peripherals.PORT);

        /* Clock setup */
        let (_, clocks, tokens) = atsamd_hal::clock::v2::clock_system_at_reset(
            peripherals.OSCCTRL,
            peripherals.OSC32KCTRL,
            peripherals.GCLK,
            peripherals.MCLK,
            &mut peripherals.NVMCTRL,
        );
        let gclk0 = clocks.gclk0;

        // SAFETY: Misusing the PAC API can break the system.
        // This is safe because we only steal the MCLK.
        let (_, _, _, mut mclk) = unsafe { clocks.pac.steal() };

        /* CAN config */
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

        /* Radio config */
        let (radio, gclk0) = RadioDevice::new(
            tokens.pclks.sercom5,
            &mclk,
            peripherals.SERCOM5,
            pins.pb17,
            pins.pb16,
            gclk0,
        );

        /* Status LED */
        let led = pins.pa14.into_push_pull_output();

        /* Spawn tasks */
        blink::spawn().ok();

        /* Monotonic clock */
        let mono = Systick::new(core.SYST, gclk0.freq().0);

        (
            Shared {
                em: ErrorManager::new(),
                data_manager: DataManager::new(),
                can0,
            },
            Local {
                led,
                radio,
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
            // can.process_data();
            // send data to the send_gs task
        });
    }

    /**
     * Sends a message over CAN.
     */
    #[task(capacity = 10, local = [counter: u16 = 0], shared = [can0, &em])]
    fn send_internal(mut cx: send_internal::Context, m: Message) {
        cx.shared.em.run(|| {
            cx.shared.can0.lock(|can| can.send_message(m))?;
            Ok(())
        });
    }

    /**
     * Sends a message to the radio over UART.
     */
    #[task(capacity = 10, local = [radio], shared = [&em])]
    fn send_gs(cx: send_gs::Context, m: Message) {
        cx.shared.em.run(|| {
            cx.local.radio.send_message(m)?;
            Ok(())
        });
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