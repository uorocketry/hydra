#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod gpio_manager;
mod state_machine;
mod types;

use atsamd_hal as hal;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::clock::v2::Source;
use atsamd_hal::dmac::DmaController;
use common_arm::hinfo;
use common_arm::*;
use common_arm_atsame::mcan;
use communication::CanCommandManager;
use communication::Capacities;
use data_manager::DataManager;
use defmt::info;
use defmt_rtt as _;
use embedded_hal::spi::FullDuplex;
use gpio_manager::GPIOManager;
use hal::gpio::{Alternate, Pin, Pins, PushPullOutput, D, PA04, PA05, PA07};
use panic_probe as _; // global logger

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use hal::prelude::*;
use hal::rtc::Count32Mode;
use hal::sercom::{spi, spi::Config, spi::Duplex, spi::Pads, spi::Spi, IoSet3, Sercom0};
use mcan::messageram::SharedMemory;
use messages::*;
use ms5611_01ba::{calibration::OversamplingRatio, MS5611_01BA};
use state_machine::{StateMachine, StateMachineContext};
use systick_monotonic::*;
use types::COM_ID;
pub static RTC: Mutex<RefCell<Option<hal::rtc::Rtc<Count32Mode>>>> = Mutex::new(RefCell::new(None));
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

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EVSYS_0, EVSYS_1, EVSYS_2])]
mod app {
    use atsamd_hal::gpio::{PA03, PB04};

    use super::*;

    #[shared]
    struct Shared {
        em: ErrorManager,
        data_manager: DataManager,
        can0: communication::CanDevice0,
        gpio: GPIOManager,
        can_command_manager: CanCommandManager,
    }

    #[local]
    struct Local {
        led_green: Pin<PA03, PushPullOutput>,
        led_red: Pin<PB04, PushPullOutput>,
        state_machine: StateMachine,
        barometer: MS5611_01BA<
            Spi<
                Config<
                    Pads<
                        hal::pac::SERCOM0,
                        IoSet3,
                        Pin<PA07, Alternate<D>>,
                        Pin<PA04, Alternate<D>>,
                        Pin<PA05, Alternate<D>>,
                    >,
                >,
                Duplex,
            >,
        >,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init(local = [
        #[link_section = ".can"]
        can_memory: SharedMemory<Capacities> = SharedMemory::new(),
        #[link_section = ".can_command"]
        can_command_memory: SharedMemory<Capacities> = SharedMemory::new()
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut peripherals = cx.device;
        let core = cx.core;
        let pins = Pins::new(peripherals.PORT);

        HydraLogging::set_ground_station_callback(queue_gs_message);

        let mut dmac = DmaController::init(peripherals.DMAC, &mut peripherals.PM);
        let _dmaChannels = dmac.split();

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

        let (pclk_can_command, gclk0) = Pclk::enable(tokens.pclks.can1, gclk0);
        let (can_command_manager, gclk0) = CanCommandManager::new(
            pins.pb15.into_mode(),
            pins.pb14.into_mode(),
            pclk_can_command,
            clocks.ahbs.can1,
            peripherals.CAN1,
            gclk0,
            cx.local.can_command_memory,
            false,
        );

        /* GPIO config */
        let led_green = pins.pa03.into_push_pull_output();
        let led_red = pins.pb04.into_push_pull_output();
        let gpio = GPIOManager::new(
            // pins switched from schematic
            pins.pb12.into_push_pull_output(),
            pins.pb11.into_push_pull_output(),
        );
        /* State Machine config */
        let state_machine = StateMachine::new();

        //type pads = hal::sercom::spi::PadsFromIds<Sercom0, IoSet3, PA07, PA04, PA05, PA06>;
        //let pads_spi = pads::Pads::new(pins.pa07, pins.pa04, pins.pa05, pins.pa06);

        let (pclk_sd, gclk0) = Pclk::enable(tokens.pclks.sercom0, gclk0);
        let pads_spi = spi::Pads::<Sercom0, IoSet3>::default()
            .sclk(pins.pa05)
            .data_in(pins.pa07)
            .data_out(pins.pa04);

        let baro_spi = spi::Config::new(&mclk, peripherals.SERCOM0, pads_spi, pclk_sd.freq())
            .length::<spi::lengths::U1>()
            .bit_order(spi::BitOrder::MsbFirst)
            .spi_mode(spi::MODE_0)
            .enable();

        let barometer = MS5611_01BA::new(baro_spi, OversamplingRatio::OSR2048);

        info!("RTC");
        let rtc = hal::rtc::Rtc::clock_mode(peripherals.RTC, 1024.Hz(), &mut mclk);
        let mut rtc = rtc.into_count32_mode(); // hal bug this must be done
        rtc.set_count32(0);
        cortex_m::interrupt::free(|cs| {
            RTC.borrow(cs).replace(Some(rtc));
        });
        // info!("RTC done");

        /* Spawn tasks */
        run_sm::spawn().ok();
        read_barometer::spawn().ok();
        state_send::spawn().ok();
        blink::spawn().ok();
        // fire_main::spawn_after(ExtU64::secs(1000)).ok();
        // fire_drogue::spawn_after(ExtU64::secs(15)).ok();

        /* Monotonic clock */
        let mono = Systick::new(core.SYST, gclk0.freq().to_Hz());

        (
            Shared {
                em: ErrorManager::new(),
                data_manager: DataManager::new(),
                can0,
                can_command_manager,
                gpio,
            },
            Local {
                led_green,
                led_red,
                state_machine,
                barometer,
            },
            init::Monotonics(mono),
        )
    }

    /// Idle task for when no other tasks are running.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    /// Handles the CAN0 interrupt.
    #[task(priority = 3, binds = CAN1, shared = [can_command_manager, data_manager])]
    fn can_command(mut cx: can_command::Context) {
        info!("CAN1 interrupt");
        cx.shared.can_command_manager.lock(|can| {
            cx.shared
                .data_manager
                .lock(|data_manager| can.process_data(data_manager));
        });
    }

    /// Receives a log message from the custom logger so that it can be sent over the radio.
    pub fn queue_gs_message(d: impl Into<Data>) {
        let message = Message::new(
            cortex_m::interrupt::free(|cs| {
                let mut rc = RTC.borrow(cs).borrow_mut();
                let rtc = rc.as_mut().unwrap();
                rtc.count32()
            }),
            COM_ID,
            d.into(),
        );

        send_internal::spawn(message).ok();
    }

    /**
     * Sends a message over CAN.
     */
    #[task(capacity = 5, shared = [can_command_manager, &em])]
    fn send_command(mut cx: send_command::Context, m: Message) {
        cx.shared.em.run(|| {
            cx.shared
                .can_command_manager
                .lock(|can| can.send_message(m))?;
            Ok(())
        });
    }

    #[task(local = [barometer], shared = [&em])]
    fn read_barometer(cx: read_barometer::Context) {
        cx.shared.em.run(|| {
            let barometer = cx.local.barometer;
            let (p, t) = barometer.get_data()?;
            info!("pressure {} temperature {}", p, t);
            Ok(())
        });
        spawn_after!(read_barometer, ExtU64::secs(1)).ok();
    }

    #[task(priority = 3, local = [fired: bool = false], shared=[gpio, &em])]
    fn fire_drogue(mut cx: fire_drogue::Context) {
        cx.shared.em.run(|| {
            if !(*cx.local.fired) {
                cx.shared.gpio.lock(|gpio| {
                    gpio.fire_drogue();
                    *cx.local.fired = true;
                });
                spawn_after!(fire_drogue, ExtU64::secs(5))?; // this becomes redundant with a proper error manager
            } else {
                cx.shared.gpio.lock(|gpio| {
                    gpio.close_drogue();
                });
            }
            Ok(())
        });
    }

    #[task(priority = 3, local = [fired: bool = false], shared=[gpio, &em])]
    fn fire_main(mut cx: fire_main::Context) {
        cx.shared.em.run(|| {
            if !(*cx.local.fired) {
                cx.shared.gpio.lock(|gpio| {
                    gpio.fire_main();
                    hinfo!(MainDeploy);
                    *cx.local.fired = true;
                });
                spawn_after!(fire_main, ExtU64::secs(5))?; // this becomes redundant with a proper error manager
            } else {
                cx.shared.gpio.lock(|gpio| {
                    gpio.close_main();
                });
            }
            Ok(())
        });
    }

    /// Runs the state machine.
    /// This takes control of the shared resources.
    #[task(priority = 3, local = [state_machine], shared = [can0, gpio, data_manager, &em])]
    fn run_sm(mut cx: run_sm::Context) {
        cx.local.state_machine.run(&mut StateMachineContext {
            shared_resources: &mut cx.shared,
        });
        cx.shared.data_manager.lock(|data| {
            data.set_state(cx.local.state_machine.get_state());
        });
        spawn_after!(run_sm, ExtU64::millis(500)).ok();
    }

    /// Handles the CAN0 interrupt.
    #[task(binds = CAN0, shared = [can0, data_manager, &em])]
    fn can0(mut cx: can0::Context) {
        info!("CAN0 interrupt");
        cx.shared.can0.lock(|can| {
            cx.shared.data_manager.lock(|data_manager| {
                cx.shared.em.run(|| {
                    can.process_data(data_manager)?;
                    Ok(())
                });
            });
        });
    }

    /// Sends a message over CAN.
    #[task(capacity = 10, shared = [can0, &em])]
    fn send_internal(mut cx: send_internal::Context, m: Message) {
        cx.shared.em.run(|| {
            cx.shared.can0.lock(|can| can.send_message(m))?;
            Ok(())
        });
    }

    /// Sends info about the current state of the system.
    #[task(shared = [data_manager, &em])]
    fn state_send(mut cx: state_send::Context) {
        cx.shared.em.run(|| {
            let rocket_state = cx
                .shared
                .data_manager
                .lock(|data| data.current_state.clone());
            let state = if let Some(rocket_state) = rocket_state {
                rocket_state
            } else {
                // This isn't really an error, we just don't have data yet.
                return Ok(());
            };
            let board_state = messages::state::State { data: state.into() };
            let message = Message::new(
                cortex_m::interrupt::free(|cs| {
                    let mut rc = RTC.borrow(cs).borrow_mut();
                    let rtc = rc.as_mut().unwrap();
                    // info!("RTC: {:?}", rtc.count32());
                    rtc.count32()
                }),
                COM_ID,
                board_state,
            );
            spawn!(send_internal, message)?;
            spawn_after!(state_send, ExtU64::secs(2))?; // I think this is fine here.
            Ok(())
        });
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
