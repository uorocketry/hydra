#![no_std]
#![no_main]

mod data_manager;

use common_arm::*;
use data_manager::DataManager;
use defmt::info;
use stm32h7xx_hal::gpio::gpioa::{PA1, PA2};
use stm32h7xx_hal::gpio::gpioc::{PC13, PC3};
use stm32h7xx_hal::gpio::Input;
use stm32h7xx_hal::gpio::{Output, PushPull};
use stm32h7xx_hal::prelude::*;
use systick_monotonic::*;

/// Custom panic handler.
/// Reset the system if a panic occurs.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    stm32h7xx_hal::pac::SCB::sys_reset();
}

#[rtic::app(device = stm32h7xx_hal::stm32, dispatchers = [EXTI0, EXTI1])]
mod app {
    use super::*;

    #[shared]
    struct SharedResources {
        data_manager: DataManager,
        em: ErrorManager,
    }
    #[local]
    struct LocalResources {
        led_red: PA1<Output<PushPull>>,
        led_green: PA2<Output<PushPull>>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init]
    fn init(mut ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
        let core = ctx.core;

        let pwr = ctx.device.PWR.constrain();
        // We could use smps, but the board is not designed for it
        // let pwrcfg = example_power!(pwr).freeze();
        let pwrcfg = pwr.freeze();

        // RCC
        let rcc = ctx.device.RCC.constrain();
        let ccdr = rcc
            .use_hse(48.MHz())
            .sys_ck(300.MHz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);

        // GPIO
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);

        // leds
        let mut led_red = gpioa.pa1.into_push_pull_output();
        let mut led_green = gpioa.pa2.into_push_pull_output();

        blink::spawn().ok();

        /* Monotonic clock */
        let mono = Systick::new(core.SYST, ccdr.clocks.hclk().raw());

        info!("Online");

        (
            SharedResources {
                data_manager: DataManager::new(),
                em: ErrorManager::new(),
            },
            LocalResources { led_red, led_green },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop(); // do nothing. just keep the MCU running
        }
    }

    #[task(local = [led_red, led_green], shared = [&em])]
    fn blink(mut cx: blink::Context) {
        cx.shared.em.run(|| {
            if cx.shared.em.has_error() {
                cx.local.led_red.toggle();
                spawn_after!(blink, ExtU64::millis(200))?;
            } else {
                cx.local.led_green.toggle();
                spawn_after!(blink, ExtU64::secs(1))?;
            }
            Ok(())
        });
    }

    #[task(shared = [&em])]
    fn sleep_system(mut cx: sleep_system::Context) {
        // Turn off the SBG and CAN
    }
}
