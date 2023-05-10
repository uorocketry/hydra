#![no_std]
#![no_main]

//!
//! This crate contains common code for HYDRA. Any code that is not board specific should be put in
//! here.
//!

mod error;
mod sd;

pub use crate::error::error_manager::ErrorManager;
pub use crate::error::hydra_error::SpawnError;
pub use crate::sd::SdInterface;

use defmt_rtt as _; // global logger
