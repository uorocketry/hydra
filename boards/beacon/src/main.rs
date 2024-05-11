#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod types;

use common_arm::SdManager;
use common_arm::*;
use data_manager::DataManager;
// use defmt::info;
use hal::{
    gpio::*,
    gpio::{
        gpioa::{PA10, PA9},
        Output, PushPull,
    },
    prelude::*,
    rcc::Config,
};
use messages::sensor::Sensor;
use messages::*;
use stm32l0xx_hal as hal;

use systick_monotonic::*;

// use https://github.com/lora-rs/lora-rs.git

/// Custom panic handler.
/// Reset the system if a panic occurs.
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    stm32l0xx_hal::pac::SCB::sys_reset();
}

// Add dispatchers
#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EXTI0_1, EXTI2_3, EXTI4_15])]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        data_manager: DataManager,
    }

    #[local]
    struct Local {
        // add leds
        green_led: PA9<Output<PushPull>>,
        red_led: PA10<Output<PushPull>>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let device = cx.device;
        let core = cx.core;

        // configure the clock
        let mut rcc = device.RCC.freeze(Config::hse(64_000_000u32.Hz()));

        // configure leds
        let gpioa = device.GPIOA.split(&mut rcc);
        let mut green_led = gpioa.pa9.into_push_pull_output();
        let mut red_led = gpioa.pa10.into_push_pull_output();

        /* Monotonic clock */
        // implement the monotonic clock
        let mono = Systick::new(core.SYST, rcc.clocks.sys_clk().0);

        (
            Shared {
                em: ErrorManager::new(),
                data_manager: DataManager::new(),
            },
            Local { green_led, red_led },
            init::Monotonics(mono),
        )
    }

    /// Idle task for when no other tasks are running.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    /**
     * Sends a message over CAN.
     */
    #[task(capacity = 10, local = [counter: u16 = 0], shared = [&em])]
    fn send_internal(cx: send_internal::Context, m: Message) {
        todo!("Send messages over CAN");
    }

    #[task(capacity = 5)]
    fn send_external(cx: send_external::Context, m: Message) {
        todo!("Send messages over LORA");
    }

    /**
     * Simple blink task to test the system.
     * Acts as a heartbeat for the system.
     */
    #[task(local = [green_led, red_led], shared = [&em])]
    fn blink(cx: blink::Context) {
        cx.shared.em.run(|| {
            if cx.shared.em.has_error() {
                cx.local.red_led.toggle().ok();
                spawn_after!(blink, ExtU64::millis(200))?;
            } else {
                cx.local.green_led.toggle().ok(); // doesn't matter if an LED fails to blink
                spawn_after!(blink, ExtU64::secs(1))?;
            }
            Ok(())
        });
    }
}
