#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod state_machine;
mod types;

use atsamd_hal as hal;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::clock::v2::Source;
use atsamd_hal::dmac::DmaController;
use common_arm::mcan;
use common_arm::*;
use communication::Capacities;
use data_manager::DataManager;
use hal::gpio::{Pin, Pins, PushPullOutput, PA14};
use hal::prelude::*;
use mcan::messageram::SharedMemory;
use messages::*;
use panic_halt as _;
use state_machine::{StateMachine, StateMachineContext};
use systick_monotonic::*;
use types::GPIOController;
use types::COM_ID;
use crate::state_machine::RocketStates;

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        data_manager: DataManager,
        can0: communication::CanDevice0,
        gpio: GPIOController,
    }

    #[local]
    struct Local {
        led: Pin<PA14, PushPullOutput>,
        state_machine: StateMachine,
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

        let mut dmac = DmaController::init(peripherals.DMAC, &mut peripherals.PM);
        let _dmaChannels = dmac.split();

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

        /* GPIO config */
        let led = pins.pa14.into_push_pull_output();
        let gpio = GPIOController::new(
            pins.pa18.into_push_pull_output(),
            pins.pa19.into_push_pull_output(),
        );
        /* State Machine config */
        let state_machine = StateMachine::new();

        /* Spawn tasks */
        run_sm::spawn().ok();
        state_send::spawn().ok();
        blink::spawn().ok();

        /* Monotonic clock */
        let mono = Systick::new(core.SYST, gclk0.freq().0);

        (
            Shared {
                em: ErrorManager::new(),
                data_manager: DataManager::new(),
                can0,
                gpio,
            },
            Local { led, state_machine },
            init::Monotonics(mono),
        )
    }

    // #[idle]
    // fn idle(_: idle::Context) -> ! {
    //     loop {
    //         rtic::export::wfi();
    //     }
    // }

    #[task(priority = 3, local = [state_machine], shared = [can0, gpio, data_manager])]
    fn run_sm(mut cx: run_sm::Context) {
        cx.local.state_machine.run(&mut StateMachineContext {
            shared_resources: &mut cx.shared,
        })
    }

    #[task(priority = 3, binds = CAN0, shared = [can0, data_manager])]
    fn can0(mut cx: can0::Context) {
        cx.shared.can0.lock(|can| {
            cx.shared
                .data_manager
                .lock(|data_manager| can.process_data(data_manager));
        });
    }

    /**
     * Sends a message over CAN.
     */
    #[task(capacity = 10, shared = [can0, &em])]
    fn send_internal(mut cx: send_internal::Context, m: Message) {
        cx.shared.em.run(|| {
            cx.shared.can0.lock(|can| can.send_message(m))?;
            Ok(())
        });
    }

    /**
     * Sends the state of the system.
     */
    #[task(shared = [data_manager, &em])]
    fn state_send(mut cx: state_send::Context) {
        let em_error = cx.shared.em.has_error();
        cx.shared.em.run(|| {
            let rocket_state = cx.shared.data_manager.lock(|data| data.current_state.clone());
            let state = if let Some(rocket_state) = rocket_state {
                rocket_state
            } else {
                RocketStates::Initializing(state_machine::Initializing {})
            };
            let board_state = messages::State {
                status: state.into(),
                has_error: em_error,
            };
            let message = Message::new(&monotonics::now(), COM_ID, board_state);
            spawn!(send_internal, message)?;  
            spawn_after!(state_send, 5.secs())?;
            Ok(())
        })
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
