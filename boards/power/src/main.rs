#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod types;

use atsamd_hal as hal;
use hal::clock::v2::pclk::Pclk;
use hal::clock::v2::Source;
use hal::prelude::*;
use hal::gpio::{PB16, PushPullOutput, Pin, PB17, Pins};
use common_arm::*;
use common_arm::mcan;
use mcan::messageram::SharedMemory;
use communication::Capacities;
use communication::CanDevice0;
use systick_monotonic::*;
use panic_halt as _;
use defmt::info;

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {
    use hal::gpio::{PA10, Input, Floating, Alternate, B, PA08, C, PA09, PA02, AlternateB};

    use crate::data_manager::DataManager;

    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        data_manager: DataManager,
        can0: CanDevice0,
    }

    #[local]
    struct Local {
        led_green: Pin<PB16, PushPullOutput>,
        led_red: Pin<PB17, PushPullOutput>,
        adc_test: hal::adc::Adc<hal::pac::ADC0>,
        adc_pin: Pin<PA10, AlternateB>,
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

        /* Clock setup */
        let (_, clocks, tokens) = hal::clock::v2::clock_system_at_reset(
            peripherals.OSCCTRL,
            peripherals.OSC32KCTRL,
            peripherals.GCLK,
            peripherals.MCLK,
            &mut peripherals.NVMCTRL,
        );
        let gclk0 = clocks.gclk0;

        /* CAN config */
        let (pclk_can, gclk0) = Pclk::enable(tokens.pclks.can0, gclk0);
        let (can0, gclk0) = communication::CanDevice0::new(
            pins.pa23.into_mode(),
            pins.pa22.into_mode(),
            pclk_can,
            clocks.ahbs.can0,
            peripherals.CAN0,
            gclk0,
            cx.local.can_memory,
            false,
        );



        let (pclk_adc0, gclk0) = Pclk::enable(tokens.pclks.adc0, gclk0);
        

        // SAFETY: Misusing the PAC API can break the system.
        // This is safe because we only steal the MCLK.
        let (_, _, gclk, mut mclk) = unsafe { clocks.pac.steal() };


        // setup ADC 
        let mut adc_test = hal::adc::Adc::adc0(peripherals.ADC0, &mut mclk);
    

        // LEDs
        let led_green = pins.pb16.into_push_pull_output();
        let led_red = pins.pb17.into_push_pull_output();

        blink::spawn().ok();
        adc::spawn().ok();

        /* Monotonic clock */
        let mono = Systick::new(core.SYST, gclk0.freq().to_Hz());

        (
            Shared {
                em: ErrorManager::new(),
                data_manager: DataManager::new(),
                can0,
            },
            Local {
                led_green, led_red, adc_test, adc_pin: pins.pa10.into_alternate()
            },
            init::Monotonics(mono),
        )
    }

    /// Idle task for when no other tasks are running.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }
    
    #[task(local = [adc_test, adc_pin], shared = [&em])]
    fn adc(cx: adc::Context) {
        // test adc for 5v PWR sense 
        info!("try adc");
        cx.shared.em.run(||{
            let val = cx.local.adc_test.read(cx.local.adc_pin);
            let x: u16 = match val {
                Ok(v) => {
                    v
                }
                Err(_) => {
                    0
                }
            };
            info!("{}", x);
            Ok(())
        });
        spawn_after!(adc, ExtU64::millis(500)).ok();
    }

    /// Simple blink task to test the system.
    /// Acts as a heartbeat for the system.
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
}