use messages::{Event, Log, LogLevel};

static mut GROUND_STATION_CALLBACK: Option<fn(Log)> = None;

/// Log an informational message. This message will be logged using defmt, and if configured, sent
/// to the ground station. As arguments, it takes an event from the [`Event`] enum, along with its
/// fields if needed.
#[macro_export]
macro_rules! hinfo {
    ($e:ident$(,)? $($p:expr),*) => {
        $crate::HydraLogging::log(messages::LogLevel::Info, messages::Event::$e($($p),*));
        defmt::info!("{}", messages::Event::$e($($p),*));
    };
}

/// Log an warning message. This message will be logged using defmt, and if configured, sent
/// to the ground station. As arguments, it takes an event from the [`Event`] enum, along with its
/// fields if needed.
#[macro_export]
macro_rules! hwarning {
    ($e:ident$(,)? $($p:expr),*) => {
        $crate::HydraLogging::log(messages::LogLevel::Warning, messages::Event::$e($($p),*));
        defmt::warning!("{}", messages::Event::$e($($p),*));
    };
}

/// Log an error message. This message will be logged using defmt, and if configured, sent
/// to the ground station. As arguments, it takes an event from the [`Event`] enum, along with its
/// fields if needed.
#[macro_export]
macro_rules! herror {
    ($e:ident$(,)? $($p:expr),*) => {
        $crate::HydraLogging::log(messages::LogLevel::Error, messages::Event::$e($($p),*));
        defmt::error!("{}", messages::Event::$e($($p),*));
    };
}

pub struct HydraLogging {}

impl HydraLogging {
    /// Set the function for sending log messages the ground station. This should be called during
    /// ONCE during init, and NEVER after to avoid any race conditions.
    pub fn set_ground_station_callback(cb: fn(Log)) {
        // SAFETY:
        // This is called once during init, so any race conditions should not be a concern.
        unsafe { GROUND_STATION_CALLBACK = Some(cb) }
    }

    /// Log a message using the callback set in [`HydraLogging::set_ground_station_callback`].
    /// While this function can be called directly, usually the [`hinfo`] and similar macros would
    /// be used instead.
    pub fn log(level: LogLevel, event: Event) {
        // SAFETY:
        // Since the static mut should only be written once during init and never after, reading
        // this variable is fine.
        if let Some(x) = unsafe { GROUND_STATION_CALLBACK } {
            x(Log::new(level, event))
        }
    }
}
