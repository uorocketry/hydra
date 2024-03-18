mod log;
mod macros;

use core::fmt::Formatter;
use derive_more::From;
use macros::{display_context, display_event};
use messages_proc_macros_lib::common_derives;

pub use log::{Log, LogLevel};

/// Custom events for Hydra. These are used to send logging information to the ground-station in
/// a space efficient way.
#[common_derives(NoFormat)]
#[derive(Copy, From)]
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
#[common_derives(NoFormat)]
#[derive(Copy, From)]
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
