#![cfg_attr(all(not(feature = "std"), not(test)), no_std)]

//! # HYDRA Messages
//!
//! This crate contains all the message definitions that will be used for inter-board communication
//! and ground-station communication.

use crate::sender::Sender;
use crate::sensor::Sensor;
use defmt::Format;
use derive_more::From;
use fugit::Instant;
use serde::{Deserialize, Serialize};

/// This is to help control versions.
pub use mavlink;

#[cfg(feature = "ts")]
use ts_rs::TS;

pub mod mav_message;
pub mod sender;
pub mod sensor;

pub const MAX_SIZE: usize = 85;

/// Topmost message. Encloses all the other possible messages, and is the only thing that should
/// be sent over the wire.
#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
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
#[serde(rename_all = "lowercase")]
pub enum Data {
    State(State),
    Sensor(Sensor),
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum Status {
    Uninitialized,
    Initializing,
    Running,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct State {
    pub status: Status,
    pub has_error: bool,
    pub voltage: f32,
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
