use crate::logging::Event;
use defmt::Format;
use serde::{Deserialize, Serialize};

#[cfg(feature = "ts")]
use ts_rs::TS;

#[cfg(any(feature = "std", test))]
use proptest_derive::Arbitrary;

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub struct Log {
    pub level: LogLevel,
    pub event: Event,
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(any(feature = "std", test), derive(Arbitrary))]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
pub enum LogLevel {
    Info,
    Warning,
    Error,
}

impl Log {
    pub fn new(level: LogLevel, event: Event) -> Self {
        Log { level, event}
    }
}