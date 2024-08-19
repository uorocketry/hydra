use crate::app::gps_dma;
use atsamd_hal::dmac;
use atsamd_hal::gpio::{Output, Pin, PushPull, PB07, PB09};
use atsamd_hal::sercom::uart::Uart;
use defmt::info;
use crate::types::{
    GPSTransfer, GpsUart, GpsUartConfig, GpsUartTx, GroundStationPads, GroundStationUartConfig,
    GPSBUFFER,
};
use ublox::{
    CfgPrtUartBuilder, DataBits, InProtoMask, OutProtoMask, Parity, StopBits, UartMode, UartPortId, UbxPacketRequest
};
// use atsamd_hal::prelude::nb;
use rtic::Mutex;
use atsamd_hal::prelude::*;

pub static mut BUF_DST: GPSBUFFER = &mut [0; 256];

/// GPS needs reset only once on startup and gps enable otherwise they are unused. This should not be in the constructor incase we are
/// We are using options right now because of a board issue and usage of two sercoms.
pub struct GpsManager {
    pub gps_tx: Option<GpsUartTx>,
    pub transfer: Option<GPSTransfer>,
    gps_enable: Pin<PB09, Output<PushPull>>,
    gps_reset: Pin<PB07, Output<PushPull>>,
}

impl GpsManager {
    pub fn new(
        gps_tx: Option<GpsUartTx>,
        gps_reset: Pin<PB07, Output<PushPull>>,
        gps_enable: Pin<PB09, Output<PushPull>>,
    ) -> Self {
        Self {
            gps_tx,
            gps_reset,
            gps_enable,
            transfer: None,
        }
    }
}

pub fn gps_dma(mut cx: crate::app::gps_dma::Context) {
    cx.shared.gps_manager.lock(|gps_manager| {
        let mut gps_manager = gps_manager.take().unwrap();
        let mut gps_dma_transfer = gps_manager.transfer.take().unwrap();
        let mut gps_tx = gps_manager.gps_tx.take().unwrap();
        if gps_dma_transfer.complete() {
            let (chan0, source, buf) = gps_dma_transfer.stop();
            // gps_dma_transfer = dmac::Transfer::new(chan0, source, unsafe { &mut *BUF_DST }, false)
            //     .unwrap()
            //     .begin(
            //         atsamd_hal::sercom::Sercom2::DMA_RX_TRIGGER,
            //         dmac::TriggerAction::BURST,
            //     );
            
            let buf_clone = buf.clone();
            let mut uart = Uart::join(source, gps_tx);
            unsafe { BUF_DST.copy_from_slice(&[0; 256]) };
            // gps_dma_transfer.block_transfer_interrupt();

            let request = UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
            for byte in request {
                nb::block!(uart.write(byte)).unwrap();
            }
            cortex_m::asm::delay(300_000);
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
                            // info!("GPS latitude: {:?}, longitude {:?}", x.lat_degrees(), x.lon_degrees());
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
