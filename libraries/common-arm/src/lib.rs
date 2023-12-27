#![no_std]
#![no_main]

//!
//! This crate contains common code for HYDRA. Any code that is not platform specific should be put in
//! here.
//!

pub use mcan;

mod error;
mod logging;
mod sd_manager;
mod health;

pub use crate::error::error_manager::ErrorManager;
pub use crate::error::hydra_error::{ErrorContextTrait, HydraError, SpawnError};
pub use crate::logging::HydraLogging;
pub use crate::sd_manager::SdManager;

use defmt_rtt as _; // global logger