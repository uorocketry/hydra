use crate::{
    hal,
    pac,
};


pub struct Cdc {
    // pub uart: hal::sercom::uart:Uart<>
}

impl Cdc {
    pub fn _new(_mclk: &pac::MCLK, _sercom: hal::sercom::Sercom5, _uart_clk: hal::clock::Sercom5CoreClock) -> Self {
        // Pads<SERCOM5, IoSet1, Pin<PB17, Alternate<C>>, Pin<PB16, Alternate<C>>>
        Cdc {
            
        }
    }
}
// let pads = hal::sercom::uart::Pads::<hal::sercom::Sercom5, _>::default()
// .rx(pins.pb17)
// .tx(pins.pb16);
// let _uart = hal::sercom::uart::Config::new(
// &peripherals.MCLK,
// peripherals.SERCOM5,
// pads,
// uart_clk.freq(),
// )
// .baud(
// 9600.hz(),
// hal::sercom::uart::BaudMode::Fractional(hal::sercom::uart::Oversampling::Bits16),
// )
// .enable();