
use core::{marker::PhantomData};

use defmt::{warn, info};

use crate::{
    hal,
    pac,
    sd,
};

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

pub struct SdInterface
{
    pub sd_controller: sd::Controller<sd::SdMmcSpi<hal::sercom::spi::Spi<hal::sercom::spi::Config<hal::sercom::spi::Pads<hal::sercom::Sercom1, hal::sercom::IoSet1, hal::gpio::Pin<hal::gpio::PA19, hal::gpio::Alternate<hal::gpio::C>>, hal::gpio::Pin<hal::gpio::PA16, hal::gpio::Alternate<hal::gpio::C>>, hal::gpio::Pin<hal::gpio::PA17, hal::gpio::Alternate<hal::gpio::C>>>>, hal::sercom::spi::Duplex>, hal::gpio::Pin<hal::gpio::PA18, hal::gpio::Output<hal::gpio::PushPull>>>, TimeSink>,
    pub volume: sd::Volume,
    pub root_directory: sd::Directory,
    pub file: sd::File,

}

impl SdInterface {
    pub fn new(mclk: &pac::MCLK, sercom: pac::SERCOM1, spi_clk: hal::clock::Sercom1CoreClock, cs: hal::gpio::Pin<hal::gpio::PA18, hal::gpio::Output<hal::gpio::PushPull>>, sck: hal::gpio::Pin<hal::gpio::PA17, hal::gpio::Output<hal::gpio::PushPull>>, miso: hal::gpio::Pin<hal::gpio::PA19, hal::gpio::Output<hal::gpio::PushPull>>, mosi: hal::gpio::Pin<hal::gpio::PA16, hal::gpio::Output<hal::gpio::PushPull>>) -> Self {
        let pads_spi = hal::sercom::spi::Pads::<hal::sercom::Sercom1, hal::sercom::IoSet1>::default()
            .sclk(sck)
            .data_in(miso)
            .data_out(mosi);
        let sdmmc_spi = hal::sercom::spi::Config::new(
            mclk,
            sercom,
            pads_spi,
            spi_clk.freq(),
        )
        .length::<hal::sercom::spi::lengths::U1>()
        .bit_order(hal::sercom::spi::BitOrder::MsbFirst)
        .spi_mode(hal::sercom::spi::MODE_0)
        .enable();
        let time_sink: TimeSink = TimeSink::new(); // Need to give this a DateTime object for actual timing.
        let mut sd_cont = sd::Controller::new(sd::SdMmcSpi::new(sdmmc_spi, cs), time_sink);
        match sd_cont.device().init() {
            Ok(_) => {
                match sd_cont.device().card_size_bytes() {
                    Ok(size) => info!("Card is {} bytes", size),
                    Err(_) => warn!("Cannot get card size"),
                }
            }
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
            "test2.txt",
            sd::Mode::ReadWriteCreateOrTruncate,
        );
        let file = match file {
            Ok(file) => file,
            Err(_) => {
                warn!("Cannot create file.");
                panic!("Cannot create file.");
            }
        };
        // create interface
        SdInterface {
            sd_controller: sd_cont,
            volume: volume,
            root_directory: root_directory,
            file: file
        }
    }
    pub fn write(&mut self, file: &mut sd::File, buffer: &[u8]) {
        match self.sd_controller.write(&mut self.volume, file, buffer) {
            Ok(_) => info!("successful write."),
            Err(_) => {
                warn!("Cannot write.")
            }
        };
    } 
    pub fn open_file(&mut self, file_name: &str) -> sd::File {
        let file = self.sd_controller.open_file_in_dir(
            &mut self.volume,
            &self.root_directory,
            file_name,
            sd::Mode::ReadWriteCreateOrTruncate,
        );
        let file = match file {
            Ok(file) => file,
            Err(_) => {
                warn!("Cannot open file.");
                panic!("Cannot create file.");
            }
        };
        file
    }
    pub fn close_file(&mut self, file: sd::File) {
        self.sd_controller.close_file(&self.volume, file).unwrap();
    }
    pub fn close(mut self) {
        self.sd_controller.close_dir(&self.volume, self.root_directory);
    }
}
