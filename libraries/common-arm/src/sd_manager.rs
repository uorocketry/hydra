use core::{fmt::Debug, marker::PhantomData};
use defmt::info;
use defmt::panic;
use embedded_hal as hal;
use embedded_sdmmc as sd;
use hal::spi::FullDuplex;

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
pub struct SdManager<SPI, CS>
where
    SPI: hal::spi::FullDuplex<u8>,
    <SPI as FullDuplex<u8>>::Error: Debug,
    CS: hal::digital::v2::OutputPin,
{
    pub sd_controller: sd::Controller<sd::SdMmcSpi<SPI, CS>, TimeSink>,
    pub volume: sd::Volume,
    pub root_directory: sd::Directory,
    pub file: Option<sd::File>,
}

impl<SPI, CS> SdManager<SPI, CS>
where
    SPI: hal::spi::FullDuplex<u8>,
    <SPI as FullDuplex<u8>>::Error: Debug,
    CS: hal::digital::v2::OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Self {
        let time_sink: TimeSink = TimeSink::new(); // Need to give this a DateTime object for actual timing.
        info!("Initializing SD card");
        let mut sd_cont = sd::Controller::new(sd::SdMmcSpi::new(spi, cs), time_sink);
        match sd_cont.device().init() {
            Ok(_) => match sd_cont.device().card_size_bytes() {
                Ok(size) => info!("Card is {} bytes", size),
                Err(_) => panic!("Cannot get card size"),
            },
            Err(_) => {
                panic!("Cannot get SD card.");
            }
        }

        let mut volume = match sd_cont.get_volume(sd::VolumeIdx(0)) {
            Ok(volume) => volume,
            Err(_) => {
                panic!("Cannot get volume 0");
            }
        };

        let root_directory = match sd_cont.open_root_dir(&volume) {
            Ok(root_directory) => root_directory,
            Err(_) => {
                panic!("Cannot get root");
            }
        };
        let file = sd_cont.open_file_in_dir(
            &mut volume,
            &root_directory,
            "lc24.txt",
            sd::Mode::ReadWriteCreateOrTruncate,
        );
        let file = match file {
            Ok(file) => file,
            Err(_) => {
                panic!("Cannot create file.");
            }
        };

        SdManager {
            sd_controller: sd_cont,
            volume,
            root_directory,
            file: Some(file),
        }
    }
    pub fn write(
        &mut self,
        file: &mut sd::File,
        buffer: &[u8],
    ) -> Result<usize, sd::Error<sd::SdMmcError>> {
        self.sd_controller.write(&mut self.volume, file, buffer)
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
    pub fn close_current_file(&mut self) -> Result<(), sd::Error<sd::SdMmcError>> {
        if let Some(file) = self.file.take() {
            return self.close_file(file);
        }
        Ok(())
    }
    pub fn close_file(&mut self, file: sd::File) -> Result<(), sd::Error<sd::SdMmcError>> {
        self.sd_controller.close_file(&self.volume, file)
    }
    pub fn close(mut self) {
        self.sd_controller
            .close_dir(&self.volume, self.root_directory);
    }
}

unsafe impl<SPI, CS> Send for SdManager<SPI, CS>
where
    SPI: hal::spi::FullDuplex<u8>,
    <SPI as FullDuplex<u8>>::Error: Debug,
    CS: hal::digital::v2::OutputPin,
{
}
