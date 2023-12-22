#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod gpio_manager;
mod types;
mod state_machine;

use communication::Capacities;
use crate::state_machine::{StateMachine, StateMachineContext};
use crate::data_manager::DataManager;
use crate::gpio_manager::GPIOManager;
use common_arm::ErrorManager;
use atsamd_hal as hal;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::clock::v2::Source;
use common_arm::mcan;
use hal::gpio::Pins;
use mcan::messageram::SharedMemory;
use panic_halt as _;
use systick_monotonic::*;
use common_arm::*;
use hal::gpio::{PB17, PushPullOutput, Pin, PB16};
use hal::prelude::*;


#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        data_manager: DataManager,
        can0: communication::CanDevice0,
        gpio_manager: GPIOManager,
    }

    #[local]
    struct Local {
        state_machine: StateMachine,
        led_red: Pin<PB17, PushPullOutput>,
        led_green: Pin<PB16, PushPullOutput>,
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
        let (_, _, _, _mclk) = unsafe { clocks.pac.steal() };

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

        /* Status LED */
        let led_red = pins.pb17.into_push_pull_output();
        let led_green = pins.pb16.into_push_pull_output(); 
        let gpio = GPIOManager::new(
            pins.pa09.into_push_pull_output(),
            pins.pa06.into_push_pull_output(),
        );

        let state_machine = StateMachine::new();

        /* Spawn tasks */                
        run_sm::spawn().ok();
        blink::spawn().ok();
        /* Monotonic clock */
        let mono = Systick::new(core.SYST, gclk0.freq().to_Hz());

        (Shared {em: ErrorManager::new(), data_manager: DataManager::new(), can0, gpio_manager: gpio}, Local {state_machine, led_red , led_green}, init::Monotonics(mono))
    }

    /// Idle task for when no other tasks are running.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    /// Handles the CAN0 interrupt.
    #[task(binds = CAN0, shared = [can0, data_manager])]
    fn can0(mut cx: can0::Context) {
        cx.shared.can0.lock(|can| {
            cx.shared
                .data_manager
                .lock(|data_manager| can.process_data(data_manager));
        });
    }

    #[task(priority = 3, local = [num: u8 = 0], shared = [gpio_manager, &em])]
    fn toggle_cams(mut cx: toggle_cams::Context) {
        cx.shared.em.run(|| {
            cx.shared.gpio_manager.lock(|gpio| {
                gpio.toggle_cam1();
                gpio.toggle_cam2();
            });
            if *cx.local.num < 3 {
                *cx.local.num += 1;
                
                spawn_after!(toggle_cams, ExtU64::millis(150))?;
            }
            Ok(())
        });
    }

    /// Runs the state machine.
    /// This takes control of the shared resources.
    #[task(priority = 3, local = [state_machine], shared = [can0, gpio_manager, data_manager, &em])]
    fn run_sm(mut cx: run_sm::Context) {
        cx.local.state_machine.run(&mut StateMachineContext {
            shared_resources: &mut cx.shared,
        });
        spawn_after!(run_sm, ExtU64::millis(500)).ok();
    }


    /// Simple blink task to test the system.
    /// Acts as a heartbeat for the system.
    #[task(local = [led_green, led_red], shared = [&em])]
    fn blink(cx: blink::Context) {
        cx.shared.em.run(|| {
            if cx.shared.em.has_error() {
                cx.local.led_red.toggle()?;
                spawn_after!(blink, ExtU64::millis(200))?;
            } else {
                cx.local.led_green.toggle()?;
                spawn_after!(blink, ExtU64::secs(1))?;
            }
            Ok(())
        });
    }    
}
