use mavlink;
use heapless::Vec;

pub const MAV_HEADER_MAIN: mavlink::MavHeader = mavlink::MavHeader {
    system_id: 1,
    component_id: 1,
    sequence: 42,
};

pub fn mavlink_postcard_message(m: Vec <u8,255>) -> mavlink::uorocketry::MavMessage{
    mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(
        mavlink::uorocketry::POSTCARD_MESSAGE_DATA{
            message: m,
        }
    )
}
