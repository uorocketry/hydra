#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod gpio_manager;
mod state_machine;
mod types;

use atsamd_hal as hal;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::clock::v2::Source;
use atsamd_hal::dmac::DmaController;
use common_arm::hinfo;
use common_arm::mcan;
use common_arm::*;
use communication::Capacities;
use data_manager::DataManager;
use gpio_manager::GPIOManager;
use hal::gpio::{Pin, Pins, PushPullOutput, PB16, PB17};
use hal::prelude::*;
use hal::timer::TimerCounter2;
use mcan::messageram::SharedMemory;
use messages::*;
use state_machine::{StateMachine, StateMachineContext};
use systick_monotonic::*;
use types::COM_ID;

/// Custom panic handler.
/// Reset the system if a panic occurs.
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    atsamd_hal::pac::SCB::sys_reset();
}

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        data_manager: DataManager,
        can0: communication::CanDevice0,
        gpio: GPIOManager,
        recovery_timer: TimerCounter2,
    }

    #[local]
    struct Local {
        led_green: Pin<PB16, PushPullOutput>,
        led_red: Pin<PB17, PushPullOutput>,
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

        /* GPIO config */
        let led_green = pins.pb16.into_push_pull_output();
        let led_red = pins.pb17.into_push_pull_output();
        let gpio = GPIOManager::new(
            // pins switched from schematic
            pins.pa09.into_push_pull_output(),
            pins.pa06.into_push_pull_output(),
        );
        /* State Machine config */
        let state_machine = StateMachine::new();

        /* Recovery Timer config */
        let (pclk_tc2tc3, gclk0) = Pclk::enable(tokens.pclks.tc2_tc3, gclk0);
        let timerclk: hal::clock::v1::Tc2Tc3Clock = pclk_tc2tc3.into();
        let mut recovery_timer =
            hal::timer::TimerCounter2::tc2_(&timerclk, peripherals.TC2, &mut mclk);

        /* Spawn tasks */
        run_sm::spawn().ok();
        state_send::spawn().ok();
        blink::spawn().ok();
        // fire_main::spawn_after(ExtU64::secs(15)).ok();
        // fire_drogue::spawn_after(ExtU64::secs(15)).ok();

        /* Monotonic clock */
        let mono = Systick::new(core.SYST, gclk0.freq().to_Hz());

        (
            Shared {
                em: ErrorManager::new(),
                data_manager: DataManager::new(),
                can0,
                gpio,
                recovery_timer,
            },
            Local {
                led_green,
                led_red,
                state_machine,
            },
            init::Monotonics(mono),
        )
    }

    /// Idle task for when no other tasks are running.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    // interrupt handler for recovery counter
    #[task(binds=TC2, shared=[data_manager, recovery_timer])]
    fn recovery_counter_tick(mut cx: recovery_counter_tick::Context) {
        cx.shared.recovery_timer.lock(|timer| {
            if timer.wait().is_ok() {
                cx.shared.data_manager.lock(|data| {
                    data.recovery_counter += 1;
                });
                // restart timer after interrupt
                let duration_mins = atsamd_hal::fugit::MinutesDurationU32::minutes(1);
                // timer requires specific duration format
                let timer_duration: atsamd_hal::fugit::Duration<u32, 1, 1000000000> = duration_mins.convert();
                timer.start(timer_duration);
            }
            timer.enable_interrupt(); // clear interrupt
        });
    }

    #[task(priority = 3, local = [fired: bool = false], shared=[gpio, &em])]
    fn fire_drogue(mut cx: fire_drogue::Context) {
        cx.shared.em.run(|| {
            if !(*cx.local.fired) {
                cx.shared.gpio.lock(|gpio| {
                    gpio.fire_drogue();
                    *cx.local.fired = true;
                });
                spawn_after!(fire_drogue, ExtU64::secs(5))?; // this becomes redundant with a proper error manager
            } else {
                cx.shared.gpio.lock(|gpio| {
                    gpio.close_drogue();
                });
            }
            Ok(())
        });
    }

    #[task(priority = 3, local = [fired: bool = false], shared=[gpio, &em])]
    fn fire_main(mut cx: fire_main::Context) {
        cx.shared.em.run(|| {
            if !(*cx.local.fired) {
                cx.shared.gpio.lock(|gpio| {
                    gpio.fire_main();
                    hinfo!(MainDeploy);
                    *cx.local.fired = true;
                });
                spawn_after!(fire_main, ExtU64::secs(5))?; // this becomes redundant with a proper error manager
            } else {
                cx.shared.gpio.lock(|gpio| {
                    gpio.close_main();
                });
            }
            Ok(())
        });
    }

    /// Runs the state machine.
    /// This takes control of the shared resources.
    #[task(priority = 3, local = [state_machine], shared = [can0, gpio, data_manager, &em, recovery_timer])]
    fn run_sm(mut cx: run_sm::Context) {
        cx.local.state_machine.run(&mut StateMachineContext {
            shared_resources: &mut cx.shared,
        });
        cx.shared.data_manager.lock(|data| {
            data.set_state(cx.local.state_machine.get_state());
        });
        spawn_after!(run_sm, ExtU64::millis(500)).ok();
    }

    /// Handles the CAN0 interrupt.
    #[task(binds = CAN0, shared = [can0, data_manager, &em])]
    fn can0(mut cx: can0::Context) {
        cx.shared.can0.lock(|can| {
            cx.shared.data_manager.lock(|data_manager| {
                cx.shared.em.run(|| {
                    can.process_data(data_manager)?;
                    Ok(())
                });
            });
        });
    }

    /// Sends a message over CAN.
    #[task(capacity = 10, shared = [can0, &em])]
    fn send_internal(mut cx: send_internal::Context, m: Message) {
        cx.shared.em.run(|| {
            cx.shared.can0.lock(|can| can.send_message(m))?;
            Ok(())
        });
    }

    /// Sends info about the current state of the system.
    #[task(shared = [data_manager, &em])]
    fn state_send(mut cx: state_send::Context) {
        cx.shared.em.run(|| {
            let rocket_state = cx
                .shared
                .data_manager
                .lock(|data| data.current_state.clone());
            let state = if let Some(rocket_state) = rocket_state {
                rocket_state
            } else {
                // This isn't really an error, we just don't have data yet.
                return Ok(());
            };
            let board_state = messages::state::State { data: state.into() };
            let message = Message::new(&monotonics::now(), COM_ID, board_state);
            spawn!(send_internal, message)?;
            spawn_after!(state_send, ExtU64::secs(2))?; // I think this is fine here.
            Ok(())
        });
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
