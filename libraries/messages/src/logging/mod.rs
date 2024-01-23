mod log;
mod macros;

use core::fmt::Formatter;
use macros::{display_context, display_event};
use serde::{Deserialize, Serialize};

#[cfg(feature = "ts")]
use ts_rs::TS;

#[cfg(test)]
use proptest_derive::Arbitrary;

pub use log::{Log, LogLevel};

/// Custom events for Hydra. These are used to send logging information to the ground-station in
/// a space efficient way.
#[derive(Serialize, Deserialize, Clone, Debug)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
#[cfg_attr(test, derive(Arbitrary))]
pub enum Event {
    Initialized(),
    MainDeploy(),
    Error(ErrorContext),
}

display_event!(
    [Initialized, "Initialized"],
    [Error, "Encountered error: {}", e],
    [MainDeploy, "Main parachute deployed"]
);

/// This is optionally used to add extra context to any errors. This information can then be sent
/// to the ground station to have a more informative error message.
#[derive(Serialize, Deserialize, Clone, Debug, Copy)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
#[cfg_attr(test, derive(Arbitrary))]
pub enum ErrorContext {
    GroundStation,
    UnkownCanMessage,
    UnknownRadioMessage,
    UnkownPostcardMessage,
    NoRadioTransfer,
}

display_context!(
    [GroundStation, "Error sending ground station message"],
    [UnkownCanMessage, "Unknown CAN message received"],
    [UnknownRadioMessage, "Unknown radio message received"],
    [NoRadioTransfer, "No radio transfer available"],
    [UnkownPostcardMessage, "Unknown postcard message received"]
);
