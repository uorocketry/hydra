// use atsamd_hal::dmac;
use core::convert::Infallible;
use defmt::{write, Format};
use derive_more::From;
use embedded_sdmmc as sd;
use messages::ErrorContext;

/// Open up atsamd hal errors without including the whole crate.


/// Contains all the various error types that can be encountered in the Hydra codebase. Extra errors
/// types should be added to this list whenever needed.
#[derive(From)]
pub enum HydraErrorType {
    /// An Infallible error. This error should never happen.
    Infallible(Infallible),
    /// Error from the Postcard serialization library.
    PostcardError(postcard::Error),
    /// Error that occurred while spawning an RTIC task. Contains the name of the failed task.
    SpawnError(&'static str),
    /// Error from the SD card library.
    SdCardError(sd::Error<sd::SdMmcError>),
    /// Error from the Mavlink library.
    MavlinkError(messages::mavlink::error::MessageWriteError),
    /// DMA error.
    // DmaError(dmac::Error),
    /// CAN send error.
    CanError(nb::Error<mcan::tx_buffers::Error>),
    /// CAN message build error.
    CanMessageError(mcan::message::TooMuchData),
}

impl defmt::Format for HydraErrorType {
    fn format(&self, f: defmt::Formatter) {
        match self {
            HydraErrorType::Infallible(_) => {
                write!(f, "Infallible error encountered!")
            }
            HydraErrorType::PostcardError(e) => {
                write!(f, "Postcard error: {}", e)
            }
            HydraErrorType::SpawnError(e) => {
                write!(f, "Could not spawn task '{}'", e);
            }
            HydraErrorType::SdCardError(_) => {
                write!(f, "SD card error!");
            }
            HydraErrorType::MavlinkError(_) => {
                write!(f, "Mavlink error!");
            }
            // HydraErrorType::DmaError(_) => {
            //     write!(f, "DMA error!");
            // }
            HydraErrorType::CanError(_) => {
                write!(f, "CAN error!");
            }
            HydraErrorType::CanMessageError(_) => {
                write!(f, "CAN message error!");
            }
        }
    }
}

/// Standard HYDRA error. This type should be used as the return type for most functions that can
/// fail and that returns a `Result`.
#[derive(Format)]
pub struct HydraError {
    error: HydraErrorType,
    context: Option<ErrorContext>,
}

impl HydraError {
    pub fn get_context(&self) -> Option<ErrorContext> {
        self.context
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
            Err(_) => Err(HydraError {
                error: HydraErrorType::SpawnError(task),
                context: None,
            }),
        }
    }
}

/// Allow the HydraErrorType to convert into an HydraError.
impl<E> From<E> for HydraError
where
    E: Into<HydraErrorType>,
{
    fn from(value: E) -> Self {
        HydraError {
            error: value.into(),
            context: None,
        }
    }
}

/// Trait to allow converting some errors to a HydraError, while also adding a ErrorContext to it.
pub trait ErrorContextTrait<T> {
    fn error_context(self, context: ErrorContext) -> Result<T, HydraError>;
}

impl<T, E> ErrorContextTrait<T> for Result<T, E>
where
    E: Into<HydraErrorType>,
{
    fn error_context(self, context: ErrorContext) -> Result<T, HydraError> {
        self.map_err(|e| HydraError {
            error: e.into(),
            context: Some(context),
        })
    }
}
