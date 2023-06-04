#![no_std]
#![no_main]

//!
//! This crate contains common code for HYDRA. Any code that is not board specific should be put in
//! here.
//!

mod error;

pub use mcan;

pub use crate::error::error_manager::ErrorManager;
pub use crate::error::hydra_error::HydraError;
pub use crate::error::hydra_error::SpawnError;

use defmt_rtt as _; // global logger
