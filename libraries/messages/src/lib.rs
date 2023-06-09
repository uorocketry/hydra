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
use defmt::Format;
use derive_more::From;
use fugit::Instant;
/// This is to help control versions.
pub use mavlink;
use serde::{Deserialize, Serialize};

#[cfg(any(feature = "std", test))]
use proptest_derive::Arbitrary;

#[cfg(feature = "ts")]
use ts_rs::TS;

pub mod command;
pub mod health;
mod logging;
pub mod sender;
pub mod sensor;
pub mod state;
pub mod sensor_status;

pub const MAX_SIZE: usize = 64;

pub use logging::{ErrorContext, Event, Log, LogLevel};

/// Topmost message. Encloses all the other possible messages, and is the only thing that should
/// be sent over the wire.
#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
#[cfg_attr(test, derive(Arbitrary))]
pub struct Message {
    /// Time in milliseconds since epoch. Note that the epoch here can be arbitrary and is not the
    /// Unix epoch.
    pub timestamp: u64,

    /// The original sender of this message.
    pub sender: Sender,

    /// The data contained in this message.
    pub data: Data,
}

#[derive(Serialize, Deserialize, Clone, Debug, From, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
#[cfg_attr(test, derive(Arbitrary))]
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
    use crate::{Message, MAX_SIZE};
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn message_size(msg: Message) {
            let bytes = postcard::to_allocvec(&msg).unwrap();

            dbg!(msg);
            assert!(dbg!(bytes.len()) <= MAX_SIZE);
        }
    }
}
