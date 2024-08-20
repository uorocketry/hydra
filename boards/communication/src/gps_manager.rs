use crate::app::gps_dma;
use crate::types::{
    GPSTransfer, GpsPads, GpsUart, GpsUartConfig, GpsUartTx, GroundStationPads,
    GroundStationUartConfig, GPSBUFFER,
};
use atsamd_hal::dmac::Transfer;
use atsamd_hal::dmac::{self, transfer};
use atsamd_hal::gpio::{Output, Pin, PushPull, PB07, PB09, Alternate, C, PA12, PA13};
use atsamd_hal::sercom::uart;
use atsamd_hal::sercom::uart::Uart;
use atsamd_hal::sercom::Sercom;
use atsamd_hal::time::Hertz;
use defmt::info;
use ublox::{
    CfgMsgAllPortsBuilder, CfgPrtUartBuilder, DataBits, InProtoMask, NavPvt, OutProtoMask, Parity,
    StopBits, UartMode, UartPortId, UbxPacketRequest,
};
// use atsamd_hal::prelude::nb;
use atsamd_hal::fugit::ExtU64;
use atsamd_hal::fugit::RateExtU32;
use atsamd_hal::prelude::*;
use rtic::Mutex;

pub static mut BUF_DST: GPSBUFFER = &mut [0; 256];

/// GPS needs reset only once on startup and gps enable otherwise they are unused. This should not be in the constructor incase we are
/// We are using options right now because of a board issue and usage of two sercoms.
pub struct GpsManager {
    pub gps_uart: Option<GpsUart>,
    pub gps_uart_tx: Option<GpsUartTx>,
    pub transfer: Option<GPSTransfer>,
    gps_enable: Pin<PB09, Output<PushPull>>,
    gps_reset: Pin<PB07, Output<PushPull>>,
    pub gps_pads: Option<GpsPads>,
    pub sercom: Option<atsamd_hal::pac::SERCOM2>,
    pub uart_clk_freq: Hertz,
    pub rx: Option<Pin<PA13, Alternate<C>>>,
    pub tx: Option<Pin<PA12, Alternate<C>>>,
}

impl GpsManager {
    pub fn new(
        mut gps_uart: GpsUart, // cannot be an option in this constructor because we want to configure it before the radio.
        gps_pads: Option<GpsPads>,
        sercom: Option<atsamd_hal::pac::SERCOM2>,
        mut gps_reset: Pin<PB07, Output<PushPull>>,
        mut gps_enable: Pin<PB09, Output<PushPull>>,
        mut dma_channel: dmac::Channel<dmac::Ch0, dmac::Ready>,
        pclk_sercom2_freq: Hertz,
    ) -> Self {
        gps_reset.set_low().ok();
        cortex_m::asm::delay(300_000);
        gps_reset.set_high().ok();
        gps_enable.set_low().ok();

        // let (mut gps_rx, mut gps_tx) = gps_uart.split();

        let packet: [u8; 28] = CfgPrtUartBuilder {
            portid: UartPortId::Uart1,
            reserved0: 0,
            tx_ready: 0,
            mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
            baud_rate: 9600,
            in_proto_mask: InProtoMask::all(),
            out_proto_mask: OutProtoMask::UBLOX,
            flags: 0,
            reserved5: 0,
        }
        .into_packet_bytes();

        for byte in packet {
            nb::block!(gps_uart.write(byte)).unwrap();
        }

        // cortex_m::asm::delay(300_000);

        // let packet_two = CfgMsgAllPortsBuilder::set_rate_for::<NavPvt>([1, 0, 0, 0, 0, 0]).into_packet_bytes();
        // for byte in packet_two {
        //     nb::block!(gps_uart.write(byte)).unwrap();
        // }

        // cortex_m::asm::delay(300_000);
        // let request = UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
        // for byte in request {
        //     nb::block!(gps_uart.write(byte)).unwrap();
        // }

        let (mut rx, mut tx) = gps_uart.split();
        dma_channel
            .as_mut()
            .enable_interrupts(dmac::InterruptFlags::new().with_tcmpl(true));
        let mut gps_dma_transfer = Transfer::new(dma_channel, rx, unsafe { &mut *BUF_DST }, false)
            .expect("DMA err")
            .begin(
                atsamd_hal::sercom::Sercom2::DMA_RX_TRIGGER,
                dmac::TriggerAction::BURST,
            );

        // let mut gps_dma_transfer = rx.receive_with_dma( unsafe { &mut *BUF_DST },dma_channel, |_| {
            // info!("DMA Transfer Complete");
        // });

        // gps_dma_transfer.wait(); 

        loop {
            if gps_dma_transfer.complete() {
                info!("DMA Transfer Complete");
                break;
            }
        }

        Self {
            gps_uart: None,
            gps_uart_tx: Some(tx),
            gps_reset,
            gps_enable,
            transfer: None,
            gps_pads,
            sercom,
            uart_clk_freq: pclk_sercom2_freq,
            rx: None, // consumed by pads first
            tx: None,
        }
    }
}

pub fn gps_dma(mut cx: crate::app::gps_dma::Context) {
    info!("GPS DMA");
    cx.shared.gps_manager.lock(|mut gps_manager| {
        let mut gps_dma_transfer = gps_manager.transfer.take().unwrap();
        // let mut gps_tx = gps_manager.gps_tx.take().unwrap();
        if gps_dma_transfer.complete() {
            let (chan0, mut gps_rx, buf) = gps_dma_transfer.stop();

            // get the radio manager from the shared resources
            // take the gps_tx from the gps_manager and gps_rx from the dma transfer and then join them to create the uart object.
            // disable the gps uart giving back the config.
            // free the config giving back the pads and sercom.
            let mut gps_tx = gps_manager.gps_uart_tx.take().unwrap();
            let mut gps_uart = atsamd_hal::sercom::uart::Uart::join(gps_rx, gps_tx);
            let mut config = gps_uart.disable();
            let (mut sercom, mut pads) = config.free();
            let (rx, tx, _, _) = pads.free();
            gps_manager.rx = Some(rx);
            gps_manager.tx = Some(tx);

            cx.shared.radio_manager.lock(|mut radio_manager| {
                let radio_uart = cortex_m::interrupt::free(|cs| {
                    let mut x = unsafe { crate::MCLK.borrow(cs).borrow_mut() };
                    let mclk = x.as_mut().unwrap();
                    let pads = atsamd_hal::sercom::uart::Pads::<atsamd_hal::sercom::Sercom2, atsamd_hal::sercom::IoSet3>::default()
                    .rx(radio_manager.rx.take().unwrap())
                    .tx(radio_manager.tx.take().unwrap());
                    GroundStationUartConfig::new(
                        &mclk,
                        sercom,
                        pads,
                        radio_manager.uart_clk_freq,
                    )
                    .baud(
                        RateExtU32::Hz(57600),
                        uart::BaudMode::Fractional(uart::Oversampling::Bits16),
                    )
                    .enable()
                });
                radio_manager.radio = Some(radio_uart);
            });

            // gps_dma_transfer = dmac::Transfer::new(chan0, source, unsafe { &mut *BUF_DST }, false)
            //     .unwrap()
            //     .begin(
            //         atsamd_hal::sercom::Sercom2::DMA_RX_TRIGGER,
            //         dmac::TriggerAction::BURST,
            //     );

            let buf_clone = buf.clone();
            // let mut uart = Uart::join(source, gps_tx);
            unsafe { BUF_DST.copy_from_slice(&[0; 256]) };
            // gps_dma_transfer.block_transfer_interrupt();

            // let request = UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
            // for byte in request {
            //     nb::block!(gps_uart.write(byte)).unwrap();
            // }
            // cortex_m::asm::delay(300_000);
            let mut buf: [u8; 256] = [0; 256];
            let mut bytes: [u8; 256] = [0; 256];
            let buf = ublox::FixedLinearBuffer::new(&mut buf[..]);
            let mut parser = ublox::Parser::new(buf);
            let mut msgs = parser.consume(&buf_clone);
            while let Some(msg) = msgs.next() {
                match msg {
                    Ok(msg) => match msg {
                        ublox::PacketRef::NavPosLlh(x) => {
                            let message_data = messages::sensor::NavPosLlh {
                                height_msl: x.height_msl(),
                                longitude: x.lon_degrees(),
                                latitude: x.lat_degrees(),
                            };
                            info!(
                                "GPS latitude: {:?}, longitude {:?}",
                                x.lat_degrees(),
                                x.lon_degrees()
                            );
                        }
                        ublox::PacketRef::NavStatus(x) => {
                            info!("GPS fix stat: {:?}", x.fix_stat_raw());
                        }
                        ublox::PacketRef::NavDop(x) => {
                            info!("GPS geometric drop: {:?}", x.geometric_dop());
                        }
                        ublox::PacketRef::NavSat(x) => {
                            info!("GPS num sats used: {:?}", x.num_svs());
                        }
                        ublox::PacketRef::NavVelNed(x) => {
                            info!("GPS velocity north: {:?}", x.vel_north());
                        }
                        ublox::PacketRef::NavPvt(x) => {
                            info!("GPS nun sats PVT: {:?}", x.num_satellites());
                        }
                        _ => {
                            info!("GPS Message not handled.");
                        }
                    },
                    Err(e) => {
                        info!("GPS parse Error");
                    }
                }
            }
        }
    });
}
