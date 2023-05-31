#![no_std]
#![no_main]
mod communication;
mod statemachine;
mod types;
use types::Capacities;
use atsamd_hal as hal;
use atsamd_hal::clock::v2::pclk::Pclk;

use common_arm::mcan;
use common_arm::sfsm::StateMachine;
use common_arm::*;


use hal::gpio::Pins;
use hal::gpio::PA14;
use hal::gpio::{Pin, PushPullOutput};
use hal::prelude::*;

use mcan::messageram::SharedMemory;



use panic_halt as _;

use systick_monotonic::*;

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {
    use super::*;

    /// check where locks already occur and depend on what first
    /// check what can be moved to local (buf_select, opt_xfer)
    /// check what can stay in shared (sbg_data)
    #[shared]
    struct Shared {
        em: ErrorManager,
        can0: communication::CanDevice0,
        state_machine: statemachine::RecoveryBoard,
    }

    #[local]
    struct Local {
        led: Pin<PA14, PushPullOutput>,
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

        let (_, clocks, tokens) = hal::clock::v2::clock_system_at_reset(
            peripherals.OSCCTRL,
            peripherals.OSC32KCTRL,
            peripherals.GCLK,
            peripherals.MCLK,
            &mut peripherals.NVMCTRL,
        );

        // SAFETY: Misusing the PAC API can break the system.
        // This is safe because we only steal the MCLK.
        let (_, _, _, _mclk) = unsafe { clocks.pac.steal() };
        let gclk0 = clocks.gclk0;
        let pins = Pins::new(peripherals.PORT);
        let led = pins.pa14.into_push_pull_output();
        /* CAN config */
        // ! This is needs to be ran before calling the constructor.
        // ! This is because gclk0 does not play nice and cannot
        // ! be incremented twice in a single function since we
        // ! then no longer can return the gclk0.
        let (pclk_can, gclk0) = Pclk::enable(tokens.pclks.can0, gclk0);
        let (can0, _gclk0) = communication::CanDevice0::new(
            pins.pa23.into_mode(),
            pins.pa22.into_mode(),
            pclk_can,
            clocks.ahbs.can0,
            peripherals.CAN0,
            gclk0,
            cx.local.can_memory,
        );
        /* State Machine Setup */
        let mut rocket = statemachine::RecoveryBoard::new();
        let wait_for_launch = statemachine::WaitForLaunch {
            accel_y: 0.0,
            pressure: 0.0,
            altitude: 0.0,
        };
        rocket.start(wait_for_launch);
        /* Spawn tasks */
        blink::spawn().ok();
        let mono = Systick::new(core.SYST, 48000000);
        (
            Shared {
                em: ErrorManager::new(),
                can0,
                state_machine: rocket,
            },
            Local {
                led,
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

    #[task(shared = [&em, state_machine])]
    fn step_state(mut cx: step_state::Context) {
        cx.shared.em.run(|| {
            let mut state_machine = cx.shared.state_machine.lock(|state_machine| {
                state_machine.step();
            });
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
            let _time = monotonics::now().duration_since_epoch().to_secs();
            if cx.shared.em.has_error() {
                spawn_after!(blink, 200.millis())?;
            } else {
                spawn_after!(blink, 1.secs())?;
            }
            Ok(())
        });
    }
}
