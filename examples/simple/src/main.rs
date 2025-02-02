#![no_main]
#![no_std]

use panic_probe as _;

use defmt::Format;
use defmt_rtt as _; // global logger

#[inline(never)]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

use core::cell::RefCell;
mod data_manager;
use stm32h7xx_hal::gpio::{Pin, Alternate, PA3};
use heapless::Vec;

use data_manager::DataManager;

use chrono::prelude::*;
use cortex_m::{asm, interrupt::Mutex};
use cortex_m_rt::entry;
use defmt::info;

use pac::interrupt;
use stm32h7xx_hal::{pac, prelude::*, rtc};
mod sbg_manager;
use sbg_manager::SBGManager;
use sbg_manager::{sbg_dma, sbg_flush, sbg_handle_data, sbg_write_data};
use sbg_rs::sbg::CallbackData;
use sbg_rs::sbg::SBG_BUFFER_SIZE;
use stm32h7xx_hal::dma::dma::StreamsTuple;
use common_arm::ErrorManager;
static RTC: Mutex<RefCell<Option<rtc::Rtc>>> = Mutex::new(RefCell::new(None));

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [EXTI2])]
mod app {
    use stm32h7xx_hal::gpio::gpioc::{PC13, PC3};
    use stm32h7xx_hal::gpio::{Edge, ExtiPin, Input, PA2};
    use stm32h7xx_hal::gpio::{Output, PushPull};
    use stm32h7xx_hal::prelude::*;

    use super::*;

    #[shared]
    struct SharedResources {
        sbg_manager: SBGManager,
        em: ErrorManager,
        data_manager: DataManager,
    }
    #[local]
    struct LocalResources {
        button: PC13<Input>,
        led: PC3<Output<PushPull>>,
        green_led: PA3<Output<PushPull>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
        let pwr = ctx.device.PWR.constrain();
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

        // GPIO
        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiod = ctx.device.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
        // leds
        let led_red = gpioa.pa2.into_push_pull_output();
        let green_led = gpioa.pa3.into_push_pull_output();

        // sbg power pin
        let mut sbg_power = gpiob.pb4.into_push_pull_output();
        sbg_power.set_high();

        // UART for sbg
        let tx: Pin<'D', 1, Alternate<8>> = gpiod.pd1.into_alternate();
        let rx: Pin<'D', 0, Alternate<8>> = gpiod.pd0.into_alternate();

        let stream_tuple = StreamsTuple::new(ctx.device.DMA1, ccdr.peripheral.DMA1);
        let uart_sbg = ctx
            .device
            .UART4
            .serial((tx, rx), 115_200.bps(), ccdr.peripheral.UART4, &ccdr.clocks)
            .unwrap();
        let sbg_manager = sbg_manager::SBGManager::new(uart_sbg, stream_tuple);

        (
            SharedResources {
                sbg_manager,
                em: ErrorManager::new(),
                data_manager: DataManager::new(),
            },
            LocalResources {
                button,
                led: gpioc.pc3.into_push_pull_output(),
                green_led,
            },
            init::Monotonics(),
        )
    }

    #[idle(shared = [data_manager])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            info!("Data {}", cx.shared.data_manager.lock(|data| data.clone_sensors()))
        }
    }

    #[task(binds = EXTI15_10, local = [button, led])]
    fn button_click(ctx: button_click::Context) {
        ctx.local.button.clear_interrupt_pending_bit();
        ctx.local.led.toggle();
    }

    extern "Rust" {

        #[task(priority = 3, binds = DMA1_STR1, shared = [&em, sbg_manager])]
        fn sbg_dma(mut context: sbg_dma::Context);

        #[task(priority = 1, shared = [data_manager])]
        fn sbg_handle_data(context: sbg_handle_data::Context, data: CallbackData);

        #[task(priority = 1, shared = [&em, sbg_manager])]
        fn sbg_flush(context: sbg_flush::Context);

        #[task(priority = 1, shared = [&em, sbg_manager])]
        fn sbg_write_data(context: sbg_write_data::Context, data: Vec<u8, SBG_BUFFER_SIZE>);
    }
}
