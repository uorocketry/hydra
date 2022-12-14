use core::convert::Infallible;
use defmt::write;
use derive_more::From;

/// Standard HYDRA error. Add variants to this enum for any errors that can occur in the codebase.
#[derive(From)]
pub enum HydraError {
    /// An Infallible error. This error should never happen.
    Infallible(Infallible),
    /// Error from the Postcard serialization library.
    PostcardError(postcard::Error),
    /// Error that occurred while spawning an RTIC task. Contains the name of the failed task.
    SpawnError(&'static str),
}

impl defmt::Format for HydraError {
    fn format(&self, f: defmt::Formatter) {
        match self {
            HydraError::Infallible(_) => {
                write!(f, "Infallible error encountered!")
            }
            HydraError::PostcardError(e) => {
                write!(f, "Postcard error: {}", e)
            }
            HydraError::SpawnError(e) => {
                write!(f, "Could not spawn task '{}'", e);
            }
        }
    }
}

/// Utility trait for implementing an easy way to convert a RTIC spawn error to a [`HydraError`].
/// This is necessary because RTIC doesn't have a standard error type.
pub trait SpawnError {
    fn spawn_error(self, task: &'static str) -> Result<(), HydraError>;
}

impl<T, E> SpawnError for Result<T, E> {
    /// Converts an RTIC spawn error into a [`HydraError`]. While this function can be used on any
    /// `Result`, this should only be called on a `Result` from a `spawn` or `spawn_after` operation.
    fn spawn_error(self, task: &'static str) -> Result<(), HydraError> {
        match self {
            Ok(_) => Ok(()),
            Err(_) => Err(HydraError::SpawnError(task)),
        }
    }
}
