use crate::logging::Event;
use defmt::Format;
use serde::{Deserialize, Serialize};

#[cfg(feature = "ts")]
use ts_rs::TS;

#[cfg(feature = "ts")]
use ts_rs::TS;

// #[cfg(any(feature = "std", test))]
#[cfg(test)]
use proptest_derive::Arbitrary;

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
#[cfg_attr(test, derive(Arbitrary))]
pub struct Log {
    pub level: LogLevel,
    pub event: Event,
}

impl Log {
    pub fn new(level: LogLevel, event: Event) -> Self {
        Log { level, event }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(feature = "ts", derive(TS))]
#[cfg_attr(feature = "ts", ts(export))]
#[cfg_attr(test, derive(Arbitrary))]
pub enum LogLevel {
    Info,
    Warning,
    Error,
}
