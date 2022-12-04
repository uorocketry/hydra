#![no_std]
#![no_main]

use atsamd_hal as hal;
use atsamd_hal::prelude::*;
use core::marker::PhantomData;
use cortex_m::asm;
use cortex_m_rt::entry;
use defmt::{info, warn, panic};
use defmt_rtt as _;
use embedded_sdmmc as sd;
use hal::gpio::Pins;
use hal::pac;
use pac::Peripherals;
use panic_halt as _;

// Could make a 1hz clock and use ticks
struct TimeSink {
    _marker: PhantomData<*const ()>,
}
impl TimeSink {
    fn new() -> Self {
        TimeSink {
            _marker: PhantomData,
        }
    }
}

impl sd::TimeSource for TimeSink {
    fn get_timestamp(&self) -> sd::Timestamp {
        sd::Timestamp {
            year_since_1970: 52,
            zero_indexed_month: 10,
            zero_indexed_day: 28,
            hours: 13,
            minutes: 56,
            seconds: 0,
        }
    }
}

#[entry]
fn main() -> ! {
    asm::nop(); // To not have main optimize to abort in release mode, remove when you add code

    let mut peripherals = Peripherals::take().unwrap();
    let p2 = cortex_m::peripheral::Peripherals::take().unwrap();
    let pins = Pins::new(peripherals.PORT);
    let mut led = pins.pa14.into_push_pull_output();

    // External 32KHz clock for stability
    let mut clock = hal::clock::GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );

    clock.configure_gclk_divider_and_source(
        pac::gclk::pchctrl::GEN_A::GCLK2,
        1,
        pac::gclk::genctrl::SRC_A::DFLL,
        false,
    );
    let gclk2 = clock
        .get_gclk(pac::gclk::pchctrl::GEN_A::GCLK2)
        .expect("Could not get gclk 2.");

    let mut delay = hal::delay::Delay::new(p2.SYST, &mut clock);

    /* Start UART CDC config */
    let uart_clk = clock
        .sercom5_core(&gclk2)
        .expect("Could not configure sercom 5 clock.");

    let pads = hal::sercom::uart::Pads::<hal::sercom::Sercom5, _>::default()
        .rx(pins.pb17)
        .tx(pins.pb16);
    let mut uart = hal::sercom::uart::Config::new(
        &peripherals.MCLK,
        peripherals.SERCOM5,
        pads,
        uart_clk.freq(),
    )
    .baud(
        9600.hz(),
        hal::sercom::uart::BaudMode::Fractional(hal::sercom::uart::Oversampling::Bits16),
    )
    .enable();
    /* End UART CDC config */

    /* Start SPI config */
    let cs = pins.pa18.into_push_pull_output();
    let sck = pins.pa17.into_push_pull_output();
    let miso = pins.pa19.into_push_pull_output();
    let mosi = pins.pa16.into_push_pull_output();
    clock.configure_gclk_divider_and_source(
        pac::gclk::pchctrl::GEN_A::GCLK3,
        3,
        pac::gclk::genctrl::SRC_A::DFLL,
        false,
    ); // 16MHz clock
    let gclk3 = clock
        .get_gclk(pac::gclk::pchctrl::GEN_A::GCLK3)
        .expect("Cannot get gclk 3.");
    let spi_clk = clock
        .sercom1_core(&gclk3)
        .expect("Cannot configure sercom 1 clock.");
    let pads_spi = hal::sercom::spi::Pads::<hal::sercom::Sercom1, hal::sercom::IoSet1>::default()
        .sclk(sck)
        .data_in(miso)
        .data_out(mosi);

    let sdmmc_spi = hal::sercom::spi::Config::new(
        &peripherals.MCLK,
        peripherals.SERCOM1,
        pads_spi,
        spi_clk.freq(),
    )
    .length::<hal::sercom::spi::lengths::U1>()
    .bit_order(hal::sercom::spi::BitOrder::MsbFirst)
    .spi_mode(hal::sercom::spi::MODE_0)
    .enable();
    /* End SPI config */

    /* Start sd controller config */
    let time_sink: TimeSink = TimeSink::new();
    let mut sd_cont = sd::Controller::new(sd::SdMmcSpi::new(sdmmc_spi, cs), time_sink);
    match sd_cont.device().init() {
        Ok(_) => match sd_cont.device().card_size_bytes() {
            Ok(size) => info!("Card is {} bytes", size),
            Err(_) => warn!("Cannot get card size"),
        },
        Err(_) => {
            warn!("Cannot get SD card");
            panic!("Cannot get SD card.");
        }
    }

    let mut volume = match sd_cont.get_volume(sd::VolumeIdx(0)) {
        Ok(volume) => volume,
        Err(_) => {
            warn!("Cannot get volume 0");
            panic!("Cannot get volume 0");
        }
    };

    let root_directory = match sd_cont.open_root_dir(&volume) {
        Ok(root_directory) => root_directory,
        Err(_) => {
            warn!("Cannot get root");
            panic!("Cannot get root");
        }
    };

    let file = sd_cont.open_file_in_dir(
        &mut volume,
        &root_directory,
        "test.txt",
        sd::Mode::ReadWriteCreateOrTruncate,
    );
    let mut file = match file {
        Ok(file) => file,
        Err(_) => {
            warn!("Cannot create file.");
            panic!("Cannot create file.");
        }
    };
    /* End sd controller config */

    let bytes_written = match sd_cont.write(&mut volume, &mut file, b"Testing file write.") {
        Ok(bytes_written) => bytes_written,
        Err(_) => {
            warn!("Cannot write file.");
            panic!("Cannot write file.");
        }
    };

    /* Write to sd */
    info!("Bytes written: {}", bytes_written);

    /* Close sd */
    sd_cont.close_file(&volume, file).unwrap();
    sd_cont.close_dir(&volume, root_directory);

    loop {
        led.set_high().unwrap();
        delay.delay_ms(1000_u16);
        info!("Test");
        led.set_low().unwrap();
        delay.delay_ms(1000_u16);
    }
}
