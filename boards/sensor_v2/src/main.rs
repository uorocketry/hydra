#![no_std]
#![no_main]

use stm32h7xx_hal::gpio::gpioc::{PC13, PC3};
use stm32h7xx_hal::gpio::{Edge, ExtiPin, Input};
use stm32h7xx_hal::gpio::{Output, PushPull};
use stm32h7xx_hal::prelude::*;

use panic_halt as _;

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct SharedResources {}
    #[local]
    struct LocalResources {
        button: PC13<Input>,
        led: PC3<Output<PushPull>>,
    }

    #[init]
    fn init(
        mut ctx: init::Context,
    ) -> (SharedResources, LocalResources, init::Monotonics) {
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
        button.make_interrupt_source(&mut ctx.device.SYSCFG);
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::Rising);
        button.enable_interrupt(&mut ctx.device.EXTI);

        (
            SharedResources {},
            LocalResources {
                button,
                led: gpioc.pc3.into_push_pull_output(),
            },
            init::Monotonics(),
        )
    }

    #[task(binds = EXTI15_10, local = [button, led])]
    fn button_click(ctx: button_click::Context) {
        ctx.local.button.clear_interrupt_pending_bit();
        ctx.local.led.toggle();
    }
}
