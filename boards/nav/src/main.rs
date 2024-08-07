#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod sbg_manager;
mod types;

use types::COM_ID;
use chrono::{NaiveDate, NaiveDateTime, NaiveTime};
use common_arm::*;
use core::num::{NonZeroU16, NonZeroU8};
use data_manager::DataManager;
use defmt::info;
use fdcan::{
    config::NominalBitTiming,
    filter::{StandardFilter, StandardFilterSlot},
};
use messages::Data;
use fugit::RateExtU32;
use heapless::Vec;
use sbg_manager::{sbg_dma, sbg_flush, sbg_sd_task, sbg_handle_data, sbg_write_data};
use sbg_rs::sbg::CallbackData;
use sbg_rs::sbg::SBG_BUFFER_SIZE;
use stm32h7xx_hal::dma::dma::StreamsTuple;
use stm32h7xx_hal::gpio::gpioa::{PA2, PA3, PA4};
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::gpio::{Output, PushPull};
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::rtc;
use stm32h7xx_hal::spi;
use stm32h7xx_hal::{rcc, rcc::rec};
use systick_monotonic::*;
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;
use panic_probe as _;
use defmt_rtt as _; // global logger

#[inline(never)]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Hardfault handler.
///
/// Terminates the application and makes a semihosting-capable debug tool exit
/// with an error. This seems better than the default, which is to spin in a
/// loop.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    loop {}
}

static RTC: Mutex<RefCell<Option<rtc::Rtc>>> = Mutex::new(RefCell::new(None));

#[rtic::app(device = stm32h7xx_hal::stm32, dispatchers = [SPI1, SPI2, SPI3])]
mod app {
    use stm32h7xx_hal::gpio::{Alternate, Pin};

    use super::*;

    #[shared]
    struct SharedResources {
        data_manager: DataManager,
        em: ErrorManager,
        sd_manager: SdManager<
            stm32h7xx_hal::spi::Spi<stm32h7xx_hal::pac::SPI1, stm32h7xx_hal::spi::Enabled>,
            PA4<Output<PushPull>>,
        >,
        sbg_manager: sbg_manager::SBGManager,
    }
    #[local]
    struct LocalResources {
        led_red: PA2<Output<PushPull>>,
        led_green: PA3<Output<PushPull>>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<500>; // 100 Hz / 10 ms granularity

    #[init]
    fn init(ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
        let core = ctx.core;

        /* Logging Setup */
        HydraLogging::set_ground_station_callback(queue_gs_message);

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
            .use_hse(48.MHz()) // check the clock hardware 
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

        let can1: fdcan::FdCan<
            stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN2>,
            fdcan::ConfigMode,
        > = {
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

        let sd = SdManager::new(spi_sd, cs_sd);

        // leds
        let led_red = gpioa.pa2.into_push_pull_output();
        let led_green = gpioa.pa3.into_push_pull_output();

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

        cortex_m::interrupt::free(|cs| {
            RTC.borrow(cs).replace(Some(rtc));
        });


        /* Monotonic clock */
        let mono = Systick::new(core.SYST, ccdr.clocks.sysclk().to_Hz());
        blink::spawn().ok();
        display_data::spawn().ok();

        info!("Online");

        (
            SharedResources {
                data_manager: DataManager::new(),
                em: ErrorManager::new(),
                sd_manager: sd,
                sbg_manager,
            },
            LocalResources { led_red, led_green },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(ctx: idle::Context) -> ! {
        loop {
        }
    }

    #[task(shared = [&em, data_manager])]
    fn display_data(mut cx: display_data::Context) {
        let data = cx.shared.data_manager.lock(|manager| manager.clone_sensors());
        info!("{:?}", data);
        spawn_after!(display_data, ExtU64::secs(1)).ok();
    }

    /// Receives a log message from the custom logger so that it can be sent over the radio.
    pub fn queue_gs_message(d: impl Into<Data>) {
        let message = messages::Message::new(
            cortex_m::interrupt::free(|cs| {
                let mut rc = RTC.borrow(cs).borrow_mut();
                let rtc = rc.as_mut().unwrap();
                rtc.date_time()
                    .unwrap_or(NaiveDateTime::new(
                        NaiveDate::from_ymd_opt(2024, 1, 1).unwrap(),
                        NaiveTime::from_hms_milli_opt(0, 0, 0, 0).unwrap(),
                    ))
                    .and_utc()
                    .timestamp_subsec_millis()
            }),
            COM_ID,
            d.into(),
        );
        info!("{:?}", message);
        // send_in::spawn(message).ok();
    }

    #[task(local = [led_red, led_green], shared = [&em])]
    fn blink(cx: blink::Context) {
        info!("Blinking");
        cx.shared.em.run(|| {
            if cx.shared.em.has_error() {
                cx.local.led_red.toggle();
                info!("Error");
                spawn_after!(blink, ExtU64::millis(200))?;
            } else {
                cx.local.led_green.toggle();
                info!("Blinking");
                spawn_after!(blink, ExtU64::secs(1))?;
            }
            Ok(())
        });
    }

    #[task(shared = [&em])]
    fn sleep_system(cx: sleep_system::Context) {
        // Turn off the SBG and CAN
    }

    extern "Rust" {
        #[task(capacity = 3, shared = [&em, sd_manager])]
        fn sbg_sd_task(context: sbg_sd_task::Context, data: [u8; SBG_BUFFER_SIZE]);

        #[task(priority = 3, binds = DMA1_STR1, shared = [&em, sbg_manager])]
        fn sbg_dma(mut context: sbg_dma::Context);

        #[task(capacity = 10, shared = [data_manager])]
        fn sbg_handle_data(context: sbg_handle_data::Context, data: CallbackData);

        #[task(shared = [&em, sbg_manager])]
        fn sbg_flush(context: sbg_flush::Context);

        #[task(shared = [&em, sbg_manager])]
        fn sbg_write_data(context: sbg_write_data::Context, data: Vec<u8, SBG_BUFFER_SIZE>);
    }
}
