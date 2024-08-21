#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod sbg_manager;
mod types;

use chrono::{NaiveDate, NaiveDateTime, NaiveTime};
use common_arm::*;
use communication::{CanCommandManager, CanDataManager};
use core::cell::RefCell;
use core::num::{NonZeroU16, NonZeroU8};
use cortex_m::interrupt::Mutex;
use data_manager::DataManager;
use defmt::info;
use defmt_rtt as _;
use fdcan::{
    config::{DataBitTiming, NominalBitTiming},
    filter::{StandardFilter, StandardFilterSlot},
};
use heapless::Vec;
use messages::sensor::Sensor;
use messages::Data;
use panic_probe as _;
use rtic_monotonics::systick::prelude::*;
use rtic_sync::{channel::*, make_channel};
use sbg_manager::{sbg_dma, sbg_flush, sbg_handle_data, sbg_sd_task, sbg_write_data};
use sbg_rs::sbg::CallbackData;
use sbg_rs::sbg::SBG_BUFFER_SIZE;
use stm32h7xx_hal::dma::dma::StreamsTuple;
use stm32h7xx_hal::gpio::gpioa::{PA2, PA3, PA4};
use stm32h7xx_hal::gpio::gpiob::PB4;
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::gpio::{Output, PushPull};
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::rtc;
use stm32h7xx_hal::spi;
use stm32h7xx_hal::{rcc, rcc::rec};
use types::COM_ID; // global logger

const DATA_CHANNEL_CAPACITY: usize = 10;

systick_monotonic!(Mono, 500); // 2ms ticks

#[inline(never)]
#[defmt::panic_handler]
fn panic() -> ! {
    stm32h7xx_hal::pac::SCB::sys_reset()
}

static RTC: Mutex<RefCell<Option<rtc::Rtc>>> = Mutex::new(RefCell::new(None));

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [EXTI0, EXTI1, EXTI2, SPI3, SPI2])]
mod app {
    use chrono::Timelike;
    use messages::Message;
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
        can_command_manager: CanCommandManager,
        can_data_manager: CanDataManager,
        sbg_power: PB4<Output<PushPull>>,
    }
    #[local]
    struct LocalResources {
        led_red: PA2<Output<PushPull>>,
        led_green: PA3<Output<PushPull>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (SharedResources, LocalResources) {
        // channel setup
        let (mut s, mut r) = make_channel!(Message, DATA_CHANNEL_CAPACITY);
        let (mut s_command, mut r_command) = make_channel!(Message, DATA_CHANNEL_CAPACITY);

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
        let mut rcc = ctx.device.RCC.constrain();
        let reset = rcc.get_reset_reason();
        info!("Reset reason: {:?}", reset);
        // match reset {
        //     // rcc::ResetReason::PowerOnReset => {
        //     //     stm32h7xx_hal::pac::SCB::sys_reset();
        //     //     info!("Power on reset");
        //     // }
        //     _ => {
        //         info!("Reset reason: {:?}", reset);
        //     }
        // }
        let fdcan_prec_unsafe = unsafe { rcc.steal_peripheral_rec() }
            .FDCAN
            .kernel_clk_mux(rec::FdcanClkSel::Pll1Q);

        let ccdr = rcc
            .use_hse(48.MHz()) // check the clock hardware
            .sys_ck(200.MHz())
            .pll1_strategy(rcc::PllConfigStrategy::Iterative)
            .pll1_q_ck(32.MHz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);
        info!("RCC configured");
        let fdcan_prec = ccdr
            .peripheral
            .FDCAN
            .kernel_clk_mux(rec::FdcanClkSel::Pll1Q);

        let btr = NominalBitTiming {
            prescaler: NonZeroU16::new(10).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        };

        // let data_bit_timing = DataBitTiming {
        //     prescaler: NonZeroU8::new(10).unwrap(),
        //     seg1: NonZeroU8::new(13).unwrap(),
        //     seg2: NonZeroU8::new(2).unwrap(),
        //     sync_jump_width: NonZeroU8::new(4).unwrap(),
        //     transceiver_delay_compensation: true,
        // };

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

        c0.set_duty(c0.get_max_duty() / 4);
        // PWM outputs are disabled by default
        // c0.enable();

        info!("PWM enabled");
        // assert_eq!(ccdr.clocks.pll1_q_ck().unwrap().raw(), 32_000_000);
        info!("PLL1Q:");
        // https://github.com/stm32-rs/stm32h7xx-hal/issues/369 This needs to be stolen. Grrr I hate the imaturity of the stm32-hal

        let can2: fdcan::FdCan<
            stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN2>,
            fdcan::ConfigMode,
        > = {
            let rx = gpiob.pb12.into_alternate().speed(Speed::VeryHigh);
            let tx = gpiob.pb13.into_alternate().speed(Speed::VeryHigh);
            ctx.device.FDCAN2.fdcan(tx, rx, fdcan_prec)
        };

        let mut can_data = can2;
        can_data.set_protocol_exception_handling(false);

        can_data.set_nominal_bit_timing(btr);

        can_data.set_automatic_retransmit(false); // data can be dropped due to its volume.

        // can_command.set_data_bit_timing(data_bit_timing);

        can_data.set_standard_filter(
            StandardFilterSlot::_0,
            StandardFilter::accept_all_into_fifo0(),
        );

        can_data.set_standard_filter(
            StandardFilterSlot::_1,
            StandardFilter::accept_all_into_fifo0(),
        );

        can_data.set_standard_filter(
            StandardFilterSlot::_2,
            StandardFilter::accept_all_into_fifo0(),
        );

        let config = can_data
            .get_config()
            .set_frame_transmit(fdcan::config::FrameTransmissionConfig::AllowFdCan);
        can_data.apply_config(config);

        can_data.enable_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);

        can_data.enable_interrupt_line(fdcan::interrupt::InterruptLine::_0, true);
        let can_data_manager = CanDataManager::new(can_data.into_normal());

        /// SAFETY: This is done as a result of a single memory mapped bit in hardware. Safe in this context.
        let can1: fdcan::FdCan<
            stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN1>,
            fdcan::ConfigMode,
        > = {
            let rx = gpioa.pa11.into_alternate().speed(Speed::VeryHigh);
            let tx = gpioa.pa12.into_alternate().speed(Speed::VeryHigh);
            ctx.device.FDCAN1.fdcan(tx, rx, fdcan_prec_unsafe)
        };

        let mut can_command = can1;
        can_command.set_protocol_exception_handling(false);

        can_command.set_nominal_bit_timing(btr);

        can_command.set_standard_filter(
            StandardFilterSlot::_0,
            StandardFilter::accept_all_into_fifo0(),
        );
        // can_data.set_data_bit_timing(data_bit_timing);

        let config = can_command
            .get_config()
            .set_frame_transmit(fdcan::config::FrameTransmissionConfig::AllowFdCanAndBRS); // check this maybe don't bit switch allow.
        can_command.apply_config(config);
        can_command.enable_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);

        can_command.enable_interrupt_line(fdcan::interrupt::InterruptLine::_0, true);

        let can_command_manager = CanCommandManager::new(can_command.into_normal());

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

        let sd_manager = SdManager::new(spi_sd, cs_sd);

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
        let mut sbg_manager = sbg_manager::SBGManager::new(uart_sbg, stream_tuple);

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
        // let mono = Systick::new(core.SYST, ccdr.clocks.sysclk().to_Hz());
        // blink::spawn().ok();
        // display_data::spawn().ok();
        Mono::start(core.SYST, 200_000_000);

        info!("Online");
        let data_manager = DataManager::new();
        let em = ErrorManager::new();
        // let mono_token = rtic_monotonics::create_systick_token!();
        // let mono = Systick::start(ctx.core.SYST, 36_000_000, mono_token);
        blink::spawn().ok();
        send_data_internal::spawn(r).ok();
        display_data::spawn(s).ok();
        // sbg_power_on::spawn().ok();
        let message = Message::new(
            0,
            COM_ID,
            messages::command::Command {
                data: messages::command::CommandData::Online(messages::command::Online {
                    online: true,
                }),
            },
        );
        send_command_internal::spawn(r_command).ok();
        async {
            s_command.send(message).await;
        };

        (
            SharedResources {
                data_manager,
                em,
                sd_manager,
                sbg_manager,
                can_command_manager,
                can_data_manager,
                sbg_power,
            },
            LocalResources { led_red, led_green },
        )
    }

    #[idle]
    fn idle(ctx: idle::Context) -> ! {
        loop {
            // info!("Idle");
            // cortex_m::asm::wfi(); // could case issue with CAN Bus. Was an issue with ATSAME51.
        }
    }

    #[task(priority = 2, shared = [&em, data_manager])]
    async fn display_data(
        mut cx: display_data::Context,
        mut sender: Sender<'static, Message, DATA_CHANNEL_CAPACITY>,
    ) {
        loop {
            let data = cx
                .shared
                .data_manager
                .lock(|manager| manager.take_sensors());
            // info!("{:?}", data.clone());
            for msg in data {
                sender
                    .send(Message::new(Mono::now().ticks(), COM_ID, Sensor::new(msg)))
                    .await;
            }
            Mono::delay(200.millis()).await; // what if there was no delay and we used chanenls to control the rate of messages.
        }
    }

    /// Receives a log message from the custom logger so that it can be sent over the radio.
    pub fn queue_gs_message(d: impl Into<Data>) {
        info!("Queueing message");
        let message = messages::Message::new(Mono::now().ticks(), COM_ID, d.into());
        info!("{:?}", message);
        // send_in::spawn(message).ok();
    }

    #[task(priority = 3, binds = FDCAN1_IT0, shared = [can_command_manager, data_manager])]
    fn can_command(mut cx: can_command::Context) {
        info!("CAN Command");
        cx.shared.can_command_manager.lock(|can| {
            cx.shared
                .data_manager
                .lock(|data_manager| can.process_data(data_manager));
        });
    }

    #[task(priority = 3, shared = [sbg_power])]
    async fn sbg_power_on(mut cx: sbg_power_on::Context) {
        loop {
            cx.shared.sbg_power.lock(|sbg| {
                sbg.set_high();
            });
            Mono::delay(10000.millis()).await;
        }
    }

    // Might be the wrong interrupt
    #[task(priority = 3, binds = FDCAN2_IT0, shared = [can_data_manager, data_manager])]
    fn can_data(mut cx: can_data::Context) {
        cx.shared.can_data_manager.lock(|can| {
            cx.shared
                .data_manager
                .lock(|data_manager| can.process_data(data_manager));
        });
    }

    #[task(priority = 2, shared = [can_data_manager, data_manager])]
    async fn send_data_internal(
        mut cx: send_data_internal::Context,
        mut receiver: Receiver<'static, Message, DATA_CHANNEL_CAPACITY>,
    ) {
        loop {
            if let Ok(m) = receiver.recv().await {
                cx.shared.can_data_manager.lock(|can| {
                    can.send_message(m);
                });
            }
        }
    }

    #[task(priority = 2, shared = [can_command_manager, data_manager])]
    async fn send_command_internal(
        mut cx: send_command_internal::Context,
        mut receiver: Receiver<'static, Message, DATA_CHANNEL_CAPACITY>,
    ) {
        while let Ok(m) = receiver.recv().await {
            cx.shared.can_command_manager.lock(|can| {
                can.send_message(m);
            });
        }
    }

    #[task(priority = 1, local = [led_red, led_green], shared = [&em])]
    async fn blink(cx: blink::Context) {
        loop {
            cx.shared.em.run(|| {
                if cx.shared.em.has_error() {
                    cx.local.led_red.toggle();
                } else {
                    cx.local.led_green.toggle();
                }
                Ok(())
            });
            Mono::delay(1000.millis()).await;
        }
    }

    #[task(priority = 3, shared = [&em, sbg_power])]
    async fn sleep_system(mut cx: sleep_system::Context) {
        // Turn off the SBG and CAN, also start a timer to wake up the system. Put the chip in sleep mode.
        cx.shared.sbg_power.lock(|sbg| {
            sbg.set_low();
        });
    }

    extern "Rust" {
        #[task(priority = 1, shared = [&em, sd_manager])]
        async fn sbg_sd_task(context: sbg_sd_task::Context, data: [u8; SBG_BUFFER_SIZE]);

        #[task(priority = 3, binds = DMA1_STR1, shared = [&em, sbg_manager])]
        fn sbg_dma(mut context: sbg_dma::Context);

        #[task(priority = 2, shared = [data_manager])]
        async fn sbg_handle_data(context: sbg_handle_data::Context, data: CallbackData);

        #[task(priority = 1, shared = [&em, sbg_manager])]
        async fn sbg_flush(context: sbg_flush::Context);

        #[task(priority = 1, shared = [&em, sbg_manager])]
        async fn sbg_write_data(context: sbg_write_data::Context, data: Vec<u8, SBG_BUFFER_SIZE>);
    }
}
