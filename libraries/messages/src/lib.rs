#![cfg_attr(all(not(feature = "std"), not(test)), no_std)]

//! # HYDRA Messages
//!
//! This crate contains all the message definitions that will be used for inter-board communication
//! and ground-station communication.

use crate::command::Command;
use crate::health::Health;
use crate::sender::Sender;
use crate::sensor::Sensor;
use crate::state::State;
use derive_more::From;
use fugit::Instant;
/// This is to help control versions.
pub use mavlink;
use messages_proc_macros_lib::common_derives;

pub mod command;
pub mod health;
mod logging;
pub mod sender;
pub mod sensor;
pub mod sensor_status;
pub mod state;

pub const MAX_SIZE: usize = 64;

pub use logging::{ErrorContext, Event, Log, LogLevel};

/// Topmost message. Encloses all the other possible messages, and is the only thing that should
/// be sent over the wire.
#[common_derives]
pub struct Message {
    /// Time in milliseconds since epoch. Note that the epoch here can be arbitrary and is not the
    /// Unix epoch.
    pub timestamp: u64,

    /// The original sender of this message.
    pub sender: Sender,

    /// The data contained in this message.
    pub data: Data,
}

#[common_derives]
#[derive(From)]
#[serde(rename_all = "lowercase")]
pub enum Data {
    State(State),
    Sensor(Sensor),
    Log(Log),
    Command(Command),
    Health(Health),
}

impl Message {
    pub fn new<const NOM: u32, const DENOM: u32>(
        timestamp: &Instant<u64, NOM, DENOM>,
        sender: Sender,
        data: impl Into<Data>,
    ) -> Self {
        Message {
            timestamp: timestamp.duration_since_epoch().to_millis(),
            sender,
            data: data.into(),
        }
    }
}

#[cfg(test)]
mod test {
    use crate::Message;
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn message_size(msg: Message) {
            let bytes = postcard::to_allocvec(&msg).unwrap();

            dbg!(&msg);
            // The size of the message should be less than or equal the maximum size.
            match msg.data {
                crate::Data::State(_) => {
                    assert!(dbg!(bytes.len()) <= 13);
                }
                crate::Data::Sensor(_) => {
                    assert!(dbg!(bytes.len()) <= 53);
                }
                crate::Data::Log(_) => {
                    assert!(dbg!(bytes.len()) <= 15);
                }
                crate::Data::Command(_) => {
                    assert!(dbg!(bytes.len()) <= 15);
                }
                crate::Data::Health(_) => {
                    assert!(dbg!(bytes.len()) <= 47);
                }
            }
        }
    }
}
