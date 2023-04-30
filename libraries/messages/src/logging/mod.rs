mod log;
mod macros;

use core::fmt::Formatter;
use macros::{display_context, display_event};
use serde::{Deserialize, Serialize};

pub use log::{Log, LogLevel};

/// Custom events for Hydra. These are used to send logging information to the ground-station in
/// a space efficient way.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum Event {
    Initialized(),
    Error(ErrorContext),
}

display_event!(
    [Initialized, "Initialized"],
    [Error, "Encountered error: {}", e]
);

/// This is optionally used to add extra context to any errors. This information can then be sent
/// to the ground station to have a more informative error message.
#[derive(Serialize, Deserialize, Clone, Debug, Copy)]
pub enum ErrorContext {
    GroundStation,
}

display_context!([GroundStation, "Error sending ground station message"]);
