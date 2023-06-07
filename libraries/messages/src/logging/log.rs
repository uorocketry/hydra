use crate::logging::Event;
use defmt::Format;
use serde::{Deserialize, Serialize};

#[cfg(test)]
use proptest_derive::Arbitrary;

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(test, derive(Arbitrary))]
pub struct Log {
    level: LogLevel,
    event: Event,
}

impl Log {
    pub fn new(level: LogLevel, event: Event) -> Self {
        Log { level, event }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
#[cfg_attr(test, derive(Arbitrary))]
pub enum LogLevel {
    Info,
    Warning,
    Error,
}
