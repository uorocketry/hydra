use crate::logging::Event;
use messages_proc_macros_lib::common_derives;

#[common_derives]
pub struct Log {
    pub level: LogLevel,
    pub event: Event,
}

#[common_derives]
pub enum LogLevel {
    Info,
    Warning,
    Error,
}

impl Log {
    pub fn new(level: LogLevel, event: Event) -> Self {
        Log { level, event }
    }
}
