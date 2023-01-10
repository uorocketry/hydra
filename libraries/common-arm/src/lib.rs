#![no_std]

//!
//! This crate contains common code for HYDRA. Any code that is not board specific should be put in
//! here.
//!

pub mod error;
pub mod sd;

pub use crate::error::error_manager::ErrorManager;
pub use crate::error::hydra_error::SpawnError;
pub use crate::sd::SdInterface;
