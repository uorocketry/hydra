use crate::logging::Event;
use defmt::Format;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
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
pub enum LogLevel {
    Info,
    Warning,
    Error,
}
