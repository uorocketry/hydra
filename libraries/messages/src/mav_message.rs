use crate::mavlink;

pub fn mavlink_header_main() -> mavlink::MavHeader {
    mavlink::MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 42,
    }
}

pub fn mavlink_postcard_message(m: &[u8]) -> mavlink::uorocketry::MavMessage{
    mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(
        mavlink::uorocketry::MavMessage::Postcard_Message_DATA{
            message: m
        }
    )
}
