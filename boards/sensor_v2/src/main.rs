#![no_std]
#![no_main]

mod data_manager;

use data_manager::DataManager;

use stm32h7xx_hal::gpio::gpioc::{PC13, PC3};
use stm32h7xx_hal::gpio::Input;
use stm32h7xx_hal::gpio::{Output, PushPull};
use stm32h7xx_hal::prelude::*;

use panic_halt as _;

#[rtic::app(device = stm32h7xx_hal::stm32, dispatchers = [EXTI0, EXTI1])]
mod app {
    use super::*;

    #[shared]
    struct SharedResources {
        data_manager: DataManager,
    }
    #[local]
    struct LocalResources {
        button: PC13<Input>,
        led: PC3<Output<PushPull>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
        let pwr = ctx.device.PWR.constrain();
        // We could use smps, but the board is not designed for it
        // let pwrcfg = example_power!(pwr).freeze();
        let pwrcfg = pwr.freeze();

        // RCC
        let rcc = ctx.device.RCC.constrain();
        let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &ctx.device.SYSCFG);

        // GPIO
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);

        // Button
        let mut button = gpioc.pc13.into_floating_input();
        (
            SharedResources {
                data_manager: DataManager::new(),
            },
            LocalResources {
                button,
                led: gpioc.pc3.into_push_pull_output(),
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {}
    }

    #[task(local = [button, led])]
    fn sleep_system(mut cx: sleep_system::Context) {
        // Turn off the SBG and CAN
    }
}
