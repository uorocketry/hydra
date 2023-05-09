use core::sync::atomic::AtomicU8;

use mavlink;
use heapless::Vec;

static MAVLINK_SEQUENCE: AtomicU8 = AtomicU8::new(0);

pub fn get_mav_header() -> mavlink::MavHeader {
    mavlink::MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: MAVLINK_SEQUENCE.fetch_add(1, core::sync::atomic::Ordering::Relaxed),
    }
}

/**
 * Creates a Mavlink message with the given data.
 * Assumes the message is postcard encoded.
 */
pub fn mavlink_postcard_message(m: Vec <u8,255>) -> mavlink::uorocketry::MavMessage{
    mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(
        mavlink::uorocketry::POSTCARD_MESSAGE_DATA{
            message: m,
        }
    )
}