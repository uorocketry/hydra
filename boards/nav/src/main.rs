#![no_std]
#![no_main]

mod data_manager;
// mod sbg_manager;
mod types;
mod communication;

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
use chrono::NaiveDate;
use stm32h7xx_hal::prelude::*;
use sbg_rs::sbg::CallbackData;
use sbg_rs::sbg::SBG_BUFFER_SIZE;
use fugit::RateExtU32;
use stm32h7xx_hal::gpio::gpioa::{PA1, PA2, PA3, PA4};
use stm32h7xx_hal::gpio::gpioc::{PC13, PC3};
use stm32h7xx_hal::gpio::Input;
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::gpio::{Output, PushPull};
use stm32h7xx_hal::pac;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::spi;
use stm32h7xx_hal::{rcc, rcc::rec};
use systick_monotonic::*;
use types::SBGSerial;
use heapless::Vec;
use stm32h7xx_hal::rtc::Rtc;
use stm32h7xx_hal::dma::dma::StreamsTuple;
// use panic_halt as _;
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    stm32h7xx_hal::pac::SCB::sys_reset();
}

#[rtic::app(device = stm32h7xx_hal::stm32, dispatchers = [EXTI0, EXTI1])]
mod app {
    use stm32h7xx_hal::{gpio::{Alternate, Pin}, pac::SPI1};

    use super::*;

    #[shared]
    struct SharedResources {
        data_manager: DataManager,
        em: ErrorManager,
        sd_manager:
            SdManager<stm32h7xx_hal::spi::Spi<stm32h7xx_hal::pac::SPI1, stm32h7xx_hal::spi::Enabled>, PA4<Output<PushPull>>>,
        rtc: Rtc,
    }
    #[local]
    struct LocalResources {
        led_red: PA2<Output<PushPull>>,
        led_green: PA3<Output<PushPull>>,
        // sbg_manager: sbg_manager::SBGManager,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init]
    fn init(mut ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
        let core = ctx.core;

        let pwr = ctx.device.PWR.constrain();
        // We could use smps, but the board is not designed for it
        // let pwrcfg = example_power!(pwr).freeze();
        let mut pwrcfg = pwr.freeze();

        info!("Power enabled");
        let backup = pwrcfg.backup().unwrap();
        info!("Backup domain enabled");
        // RCC
        let rcc = ctx.device.RCC.constrain();
        info!("RCC enabled");
        let ccdr = rcc
            .use_hse(48.MHz())
            .sys_ck(200.MHz())
            .pll1_strategy(rcc::PllConfigStrategy::Iterative)
            .pll1_q_ck(24.MHz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);
        info!("RCC configured");

        let btr = NominalBitTiming {
            prescaler: NonZeroU16::new(4).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        };
        info!("CAN enabled");
        // GPIO
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiod = ctx.device.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);

        let pins = gpiob.pb14.into_alternate();
        let mut c0 = ctx
            .device
            .TIM12
            .pwm(pins, 4.kHz(), ccdr.peripheral.TIM12, &ccdr.clocks);

        c0.set_duty(c0.get_max_duty() / 2);
        // PWM outputs are disabled by default
        // c0.enable();

        info!("PWM enabled");
        // assert_eq!(ccdr.clocks.pll1_q_ck().unwrap().raw(), 24_000_000); waaat 
        info!("PLL1Q:");
        let fdcan_prec = ccdr
            .peripheral
            .FDCAN
            .kernel_clk_mux(rec::FdcanClkSel::Pll1Q);

        let can1: fdcan::FdCan<stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN2>, fdcan::ConfigMode> = {
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

        let spi_sd: stm32h7xx_hal::spi::Spi<
            stm32h7xx_hal::stm32::SPI1,
            stm32h7xx_hal::spi::Enabled,
            u8,
        > = ctx.device.SPI1.spi(
            (
                gpioa.pa5.into_alternate::<5>(),
                gpioa.pa6.into_alternate(),
                gpioa.pa7.into_alternate(),
            ),
            spi::Config::new(spi::MODE_0),
            16.MHz(),
            ccdr.peripheral.SPI1,
            &ccdr.clocks,
        );

        let cs_sd = gpioa.pa4.into_push_pull_output();

        let mut sd = SdManager::new(spi_sd, cs_sd);

        // leds
        let mut led_red = gpioa.pa2.into_push_pull_output();
        let mut led_green = gpioa.pa3.into_push_pull_output();

        // sbg power pin 
        let mut sbg_power = gpiob.pb4.into_push_pull_output();
        sbg_power.set_high();

        // UART for sbg
        let tx: Pin<'D', 1, Alternate<8>> = gpiod.pd1.into_alternate();
        let rx: Pin<'D', 0, Alternate<8>>  = gpiod.pd0.into_alternate();

        let stream_tuple = StreamsTuple::new(ctx.device.DMA1, ccdr.peripheral.DMA1);
        let uart_sbg = ctx
        .device
        .UART4
        .serial((tx, rx), 9_800.bps(), ccdr.peripheral.UART4, &ccdr.clocks)
        .unwrap();
        // let sbg_manager = sbg_manager::SBGManager::new(uart_sbg, stream_tuple);
        // let serial = ctx
        //     .device
        //     .UART4
        //     .serial((tx, rx), 9_800.bps(), ccdr.peripheral.UART4, &ccdr.clocks)
        //     .unwrap();
        // let (sbg_rx, sbg_tx) = serial.split();

        let mut rtc = stm32h7xx_hal::rtc::Rtc::open_or_init(
            ctx.device.RTC,
            backup.RTC,
            stm32h7xx_hal::rtc::RtcClock::Lsi,
            &ccdr.clocks,
        );
    
        // TODO: Get current time from some source
        let now = NaiveDate::from_ymd_opt(2001, 1, 1)
            .unwrap()
            .and_hms_opt(0, 0, 0)
            .unwrap();
    
        rtc.set_date_time(now);
        rtc.enable_wakeup(10);
        rtc.listen(&mut ctx.device.EXTI, stm32h7xx_hal::rtc::Event::Wakeup);
    
        unsafe {
            pac::NVIC::unmask(pac::interrupt::RTC_WKUP);
        }
    
        info!("Time: {}", rtc.date_time().unwrap().and_utc().timestamp_subsec_millis());

        blink::spawn().ok();

        /* Monotonic clock */
        let mono = Systick::new(core.SYST, ccdr.clocks.hclk().raw());

        info!("Online");

        (
            SharedResources {
                data_manager: DataManager::new(),
                em: ErrorManager::new(),
                sd_manager: sd,
                rtc,
            },
            LocalResources {
                led_red,
                led_green,
                // sbg_manager: sbg_manager,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop(); // do nothing. just keep the MCU running
        }
    }

    #[task(local = [led_red, led_green], shared = [&em,rtc])]
    fn blink(mut cx: blink::Context) {
        cx.shared.em.run(|| {
            if cx.shared.em.has_error() {
                cx.local.led_red.toggle();
                spawn_after!(blink, ExtU64::millis(200))?;
            } else {
                cx.local.led_green.toggle();
                info!("Blinking");
                cx.shared.rtc.lock(|rtc| {
                    info!("Time: {}", rtc.date_time().unwrap().and_utc().timestamp_subsec_millis());
                });
                spawn_after!(blink, ExtU64::secs(1))?;
            }
            Ok(())
        });
    }

    #[task(shared = [&em])]
    fn sleep_system(mut cx: sleep_system::Context) {
        // Turn off the SBG and CAN
    }

    // extern "Rust" {
    //     #[task(capacity = 3, shared = [&em, sd_manager])]
    //     fn sbg_sd_task(context: sbg_sd_task::Context, data: [u8; SBG_BUFFER_SIZE]);

    //     #[task(binds = DMA1_STR0, shared = [&em], local = [sbg_manager])]
    //     fn sbg_dma(context: sbg_dma::Context);

    //     #[task(capacity = 20, shared = [data_manager])]
    //     fn sbg_handle_data(context: sbg_handle_data::Context, data: CallbackData);

    //     #[task(shared = [rtc, &em])]
    //     fn sbg_get_time(context: sbg_get_time::Context);

    //     #[task()]
    //     fn sbg_flush(context: sbg_flush::Context);

    //     #[task(shared = [&em])]
    //     fn sbg_write_data(context: sbg_write_data::Context, data: Vec<u8, SBG_BUFFER_SIZE>);
    // }
}
