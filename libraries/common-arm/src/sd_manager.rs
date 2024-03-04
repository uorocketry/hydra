use core::{fmt::Debug, marker::PhantomData};
use defmt::{info, error};
use embedded_hal as hal;
use embedded_sdmmc as sd;
use hal::spi::FullDuplex;

use messages::ErrorContext;
use crate::herror;

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
    pub volume: Option<sd::Volume>,
    pub root_directory: Option<sd::Directory>,
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
        let mut sd_controller = sd::Controller::new(sd::SdMmcSpi::new(spi, cs), time_sink);
        match sd_controller.device().init() {
            Ok(_) => match sd_controller.device().card_size_bytes() {
                Ok(size) => info!("Card is {} bytes", size),
                Err(_) => error!("Cannot get card size"),
            },
            Err(_) => {
                herror!(Error, ErrorContext::SDCardNotConnected);
            }
        }

        let mut volume = match sd_controller.get_volume(sd::VolumeIdx(0)) {
            Ok(volume) => volume,
            Err(_) => {
                error!("Cannot get volume 0");
                None
            }
        };

        let root_directory = match sd_controller.open_root_dir(&volume) {
            Ok(root_directory) => root_directory,
            Err(_) => {
                error!("Cannot get root");
                None
            }
        };
        let file = sd_controller.open_file_in_dir(
            &mut volume,
            &root_directory,
            "log.txt",
            sd::Mode::ReadWriteCreateOrTruncate,
        );
        let file = match file {
            Ok(file) => file,
            Err(_) => {
                error!("Cannot create file.");
                None
            }
        };

        SdManager {
            sd_controller: sd_controller,
            volume: Some(volume),
            root_directory: Some(root_directory),
            file: Some(file),
        }
    }
    pub fn write(
        &mut self,
        file: &mut sd::File,
        buffer: &[u8],
    ) -> Result<usize, sd::Error<sd::SdMmcError>> {
        if !self.is_mounted() {
            return sd::Error::DeviceError(());
        }
        self.sd_controller.write(&mut self.volume, file, buffer)
    }
    pub fn write_str(
        &mut self,
        file: &mut sd::File,
        msg: &str,
    ) -> Result<usize, sd::Error<sd::SdMmcError>> {
        if !self.is_mounted() {
            return sd::Error::DeviceError(());
        }
        let buffer: &[u8] = msg.as_bytes();
        self.sd_controller.write(&mut self.volume, file, buffer)
    }
    pub fn open_file(&mut self, file_name: &str) -> Result<sd::File, sd::Error<sd::SdMmcError>> {
        if !self.is_mounted() {
            return sd::Error::DeviceError(());
        }
        self.sd_controller.open_file_in_dir(
            &mut self.volume,
            &self.root_directory,
            file_name,
            sd::Mode::ReadWriteCreateOrTruncate,
        )
    }
    pub fn close_current_file(&mut self) -> Result<(), sd::Error<sd::SdMmcError>> {
        if !self.is_mounted() {
            return sd::Error::DeviceError(());
        }
        if let Some(file) = self.file.take() {
            return self.close_file(file);
        }
        Ok(())
    }
    pub fn close_file(&mut self, file: sd::File) -> Result<(), sd::Error<sd::SdMmcError>> {
        if !self.is_mounted() {
            return sd::Error::DeviceError(());
        }
        self.sd_controller.close_file(&self.volume, file)
    }
    pub fn close(mut self) {
        if !self.is_mounted() {
            return sd::Error::DeviceError(());
        }
        self.sd_controller
            .close_dir(&self.volume, self.root_directory);
    }
    pub fn is_mounted(mut self) {
        // Sd crate doesn't have a check method for SD card being still mounted
        // Use `card_size_bytes()` as indicator if device is still connected or not
        match self.sd_controller.device().card_size_bytes() {
            Ok(size) => true,
            Err(_) => false,
        }
    }
}

unsafe impl<SPI, CS> Send for SdManager<SPI, CS>
where
    SPI: hal::spi::FullDuplex<u8>,
    <SPI as FullDuplex<u8>>::Error: Debug,
    CS: hal::digital::v2::OutputPin,
{
}
