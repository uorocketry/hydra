#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod sbg_manager;
mod types;

use atsamd_hal as hal;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::clock::v2::Source;
use atsamd_hal::dmac::DmaController;
use common_arm::mcan;
use common_arm::SdManager;
use common_arm::*;
use communication::Capacities;
use data_manager::DataManager;
use defmt::info;
use hal::dmac;
use hal::gpio::Output;
use hal::gpio::Pins;
use hal::gpio::{Pin, PushPullOutput};
use hal::gpio::{PushPull, PB01, PB10, PB16, PB17};
use hal::prelude::*;
use hal::sercom::{spi, IoSet2, Sercom4};
use mcan::messageram::SharedMemory;
use messages::sensor::Sensor;
use messages::*;
use panic_halt as _;
use sbg_manager::{sbg_dma, sbg_handle_data, sbg_sd_task, SBGManager};
//use sbg_manager::{sbg_dma, sbg_handle_data, SBGManager};

use sbg_rs::sbg::{CallbackData, SBG_BUFFER_SIZE};

use systick_monotonic::*;
use types::*;

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        data_manager: DataManager,
        can: communication::CanDevice0,
        sd_manager: SdManager<SdSpi, Pin<PB10, Output<PushPull>>>,
    }

    #[local]
    struct Local {
        led_green: Pin<PB16, PushPullOutput>,
        led_red: Pin<PB17, PushPullOutput>,
        sbg_manager: SBGManager,
        sbg_power_pin: Pin<PB01, PushPullOutput>, // this is here so we do not need to lock sbg_manager.! put into a gpio controller with leds.
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

        let mut sbg_power_pin = pins.pb01.into_push_pull_output();
        sbg_power_pin.set_high().unwrap();

        let mut dmac = DmaController::init(peripherals.DMAC, &mut peripherals.PM);
        let dmaChannels = dmac.split();

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
        let (can, gclk0) = communication::CanDevice0::new(
            pins.pa23.into_mode(),
            pins.pa22.into_mode(),
            pclk_can,
            clocks.ahbs.can0,
            peripherals.CAN0,
            gclk0,
            cx.local.can_memory,
            false,
        );

        /* SD config */
        let (pclk_sd, gclk0) = Pclk::enable(tokens.pclks.sercom4, gclk0);
        let pads_spi = spi::Pads::<Sercom4, IoSet2>::default()
            .sclk(pins.pb09)
            .data_in(pins.pb11)
            .data_out(pins.pb08);

        let sdmmc_spi = spi::Config::new(&mclk, peripherals.SERCOM4, pads_spi, pclk_sd.freq())
            .length::<spi::lengths::U1>()
            .bit_order(spi::BitOrder::MsbFirst)
            .spi_mode(spi::MODE_0)
            .enable();
        let sd_manager = SdManager::new(sdmmc_spi, pins.pb10.into_push_pull_output());

        /* SBG config */
        let (pclk_sbg, gclk0) = Pclk::enable(tokens.pclks.sercom5, gclk0);
        let dmaCh0 = dmaChannels.0.init(dmac::PriorityLevel::LVL3);
        let sbg_manager = SBGManager::new(
            pins.pb03,
            pins.pb02,
            pclk_sbg,
            &mut mclk,
            peripherals.SERCOM5,
            peripherals.RTC,
            dmaCh0,
        );

        // Buzzer should go here. There is complexity using the new clock system with the atsamdhal pwm implementation.

        /* Status LED */
        let led_green = pins.pb16.into_push_pull_output();
        let led_red = pins.pb17.into_push_pull_output();

        /* Spawn tasks */
        sensor_send::spawn().ok();
        blink::spawn().ok();

        /* Monotonic clock */
        let mono = Systick::new(core.SYST, gclk0.freq().to_Hz());

        (
            Shared {
                em: ErrorManager::new(),
                data_manager: DataManager::new(),
                can,
                sd_manager,
            },
            Local {
                led_green,
                led_red,
                sbg_manager,
                sbg_power_pin,
            },
            init::Monotonics(mono),
        )
    }

    /// Idle task for when no other tasks are running.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(local = [sbg_power_pin], shared = [sd_manager, &em])]
    fn sleep_system(mut cx: sleep_system::Context) {
        info!("Power Down");
        // close out sd files.
        cx.shared.sd_manager.lock(|sd| {
            cx.shared.em.run(|| {
                sd.close_current_file()?;
                // sd.close(); // we can also close the root directory and volume.
                // power down sbg
                cx.local.sbg_power_pin.set_low()?; // define hydra error for this error type.
                                                   // Call core.SCB.set_deepsleep for even less power consumption.
                Ok(())
            });
        });
    }

    #[task(priority = 3, binds = CAN0, shared = [can, data_manager])]
    fn can0(mut cx: can0::Context) {
        cx.shared.can.lock(|can| {
            cx.shared
                .data_manager
                .lock(|manager| can.process_data(manager))
        });
    }

    /**
     * Sends a message over CAN.
     */
    #[task(capacity = 10, local = [counter: u16 = 0], shared = [can, &em])]
    fn send_internal(mut cx: send_internal::Context, m: Message) {
        cx.shared.em.run(|| {
            cx.shared.can.lock(|can| can.send_message(m))?;
            Ok(())
        });
    }

    /**
     * Sends information about the sensors.
     */
    #[task(shared = [data_manager, &em])]
    fn sensor_send(mut cx: sensor_send::Context) {
        let sensor_data = cx
            .shared
            .data_manager
            .lock(|data_manager| data_manager.clone_sensors());

        let messages = sensor_data
            .into_iter()
            .flatten()
            .map(|x| Message::new(&monotonics::now(), COM_ID, Sensor::new(x)));

        cx.shared.em.run(|| {
            for msg in messages {
                spawn!(send_internal, msg)?;
            }
            Ok(())
        });
        spawn_after!(sensor_send, ExtU64::millis(100)).ok();
    }

    /**
     * Simple blink task to test the system.
     * Acts as a heartbeat for the system.
     */
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

    extern "Rust" {
        #[task(capacity = 3, shared = [&em, sd_manager])]
        fn sbg_sd_task(context: sbg_sd_task::Context, data: [u8; SBG_BUFFER_SIZE]);

        #[task(binds = DMAC_0, shared = [&em], local = [sbg_manager])]
        fn sbg_dma(context: sbg_dma::Context);

        #[task(capacity = 20, shared = [data_manager])]
        fn sbg_handle_data(context: sbg_handle_data::Context, data: CallbackData);
    }
}
