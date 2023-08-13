use crate::types::SdController;
use core::marker::PhantomData;
use atsamd_hal::gpio::{Output, Pin, PushPull, PA16, PA17, PA18, PA19, PB14, PB13, PB15, PB12};
use atsamd_hal::pac;
use atsamd_hal::sercom::{spi, IoSet1, Sercom1, Sercom4};
use atsamd_hal::time::Hertz;
use defmt::{info, warn};
use embedded_sdmmc as sd;

/// Time source for `[SdInterface]`. It doesn't return any useful information for now, and will
/// always return an arbitrary time.
pub struct TimeSink {
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
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/// Wrapper for the SD Card. For now, the pins are hard-coded.
pub struct SdManager {
    pub sd_controller: SdController,
    pub volume: sd::Volume,
    pub root_directory: sd::Directory,
    pub file: sd::File,
}

impl SdManager {
    pub fn new(
        mclk: &pac::MCLK,
        sercom: pac::SERCOM4,
        freq: Hertz,
        cs: Pin<PB14, Output<PushPull>>,
        sck: Pin<PB13, Output<PushPull>>,
        miso: Pin<PB15, Output<PushPull>>,
        mosi: Pin<PB12, Output<PushPull>>,
    ) -> Self {
        let pads_spi = spi::Pads::<Sercom4, IoSet1>::default()
            .sclk(sck)
            .data_in(miso)
            .data_out(mosi);
        let sdmmc_spi = spi::Config::new(mclk, sercom, pads_spi, freq)
            .length::<spi::lengths::U1>()
            .bit_order(spi::BitOrder::MsbFirst)
            .spi_mode(spi::MODE_0)
            .enable();
        let time_sink: TimeSink = TimeSink::new(); // Need to give this a DateTime object for actual timing.
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
            "log.txt",
            sd::Mode::ReadWriteCreateOrTruncate,
        );
        let file = match file {
            Ok(file) => file,
            Err(_) => {
                warn!("Cannot create file.");
                panic!("Cannot create file.");
            }
        };

        SdManager {
            sd_controller: sd_cont,
            volume,
            root_directory,
            file,
        }
    }
    pub fn write(
        &mut self,
        // file: &mut sd::File,
        buffer: &[u8],
    ) -> Result<usize, sd::Error<sd::SdMmcError>> {
        self.sd_controller.write(&mut self.volume, &mut self.file, buffer)
    }
    pub fn write_str(
        &mut self,
        file: &mut sd::File,
        msg: &str,
    ) -> Result<usize, sd::Error<sd::SdMmcError>> {
        let buffer: &[u8] = msg.as_bytes();
        self.sd_controller.write(&mut self.volume, file, buffer)
    }
    pub fn open_file(&mut self, file_name: &str) -> Result<sd::File, sd::Error<sd::SdMmcError>> {
        self.sd_controller.open_file_in_dir(
            &mut self.volume,
            &self.root_directory,
            file_name,
            sd::Mode::ReadWriteCreateOrTruncate,
        )
    }
    pub fn close_file(&mut self, file: sd::File) -> Result<(), sd::Error<sd::SdMmcError>> {
        self.sd_controller.close_file(&self.volume, file)
    }
    pub fn close(mut self) {
        self.sd_controller
            .close_dir(&self.volume, self.root_directory);
    }
}

unsafe impl Send for SdManager {}