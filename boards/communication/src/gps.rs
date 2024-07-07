use crate::types::*;
use atsamd_hal::clock::v2::gclk::Gclk0Id;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::clock::v2::pclk::PclkToken;
use atsamd_hal::clock::v2::Source;
use atsamd_hal::gpio::{Disabled, Floating, Pin, PA12, PA13};
use atsamd_hal::pac::{MCLK, SERCOM2};
use atsamd_hal::typelevel::Increment;
use atsamd_hal::{
    sercom,
    sercom::{uart, uart::Duplex, uart::Uart},
};
use systick_monotonic::fugit::RateExtU32;

pub struct GpsDevice {
    pub uart: Uart<GpsUartConfig, Duplex>,
}

impl GpsDevice {
    pub fn new<S>(
        gps_token: PclkToken<SERCOM2>,
        mclk: &MCLK,
        sercom: SERCOM2,
        rx_pin: Pin<PA13, Disabled<Floating>>,
        tx_pin: Pin<PA12, Disabled<Floating>>,
        gclk0: S,
    ) -> (Self, S::Inc)
    where
        S: Source<Id = Gclk0Id> + Increment,
    {
        let (pclk_gps, gclk0) = Pclk::enable(gps_token, gclk0);
        let pads = uart::Pads::<sercom::Sercom2, _>::default()
            .rx(rx_pin)
            .tx(tx_pin);
        let uart = GpsUartConfig::new(mclk, sercom, pads, pclk_gps.freq())
            .baud(
                57600.Hz(), // check this
                uart::BaudMode::Fractional(uart::Oversampling::Bits16),
            )
            .enable();
        (GpsDevice { uart }, gclk0)
    }
}
