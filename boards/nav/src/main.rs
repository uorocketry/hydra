#![no_std]
#![no_main]

mod data_manager;

use common_arm::*;
use core::num::{NonZeroU16, NonZeroU8};
use data_manager::DataManager;
use defmt::info;
use fdcan::{
    config::NominalBitTiming,
    filter::{StandardFilter, StandardFilterSlot},
    frame::{FrameFormat, TxFrameHeader},
    id::StandardId,
};
use stm32h7xx_hal::gpio::gpioa::{PA1, PA2};
use stm32h7xx_hal::gpio::gpioc::{PC13, PC3};
use stm32h7xx_hal::gpio::Input;
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::gpio::{Output, PushPull};
use stm32h7xx_hal::pac;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::spi;
use stm32h7xx_hal::{rcc, rcc::rec};
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
            .pll1_strategy(rcc::PllConfigStrategy::Iterative)
            .pll1_q_ck(24.MHz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);

        let btr = NominalBitTiming {
            prescaler: NonZeroU16::new(12).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        };

        // GPIO
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiod = ctx.device.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
        assert_eq!(ccdr.clocks.pll1_q_ck().unwrap().raw(), 24_000_000);
        let fdcan_prec = ccdr
            .peripheral
            .FDCAN
            .kernel_clk_mux(rec::FdcanClkSel::Pll1Q);

        let can1 = {
            let rx = gpiob.pb12.into_alternate().speed(Speed::VeryHigh);
            let tx = gpiob.pb13.into_alternate().speed(Speed::VeryHigh);
            ctx.device.FDCAN2.fdcan(tx, rx, fdcan_prec)
        };

        let mut can = can1;
        can.set_protocol_exception_handling(false);

        can.set_nominal_bit_timing(btr);

        can.set_standard_filter(
            StandardFilterSlot::_0,
            StandardFilter::accept_all_into_fifo0(),
        );

        let (sck, miso, mosi) = (gpioa.pa5, gpioa.pa6, gpioa.pa7);

        let spi = ctx
            .device
            .SPI1
            .spi((sck, miso, mosi), spi::MODE_0, 1.MHz(), &ccdr.clocks);

        // leds
        let mut led_red = gpioa.pa1.into_push_pull_output();
        let mut led_green = gpioa.pa2.into_push_pull_output();

        // UART for sbg
        let tx = gpiod.pd1.into_alternate();
        let rx = gpiod.pd0.into_alternate();

        let serial = ctx
            .device
            .UART4
            .serial((tx, rx), 9_800.bps(), ccdr.peripheral.UART4, &ccdr.clocks)
            .unwrap();
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
