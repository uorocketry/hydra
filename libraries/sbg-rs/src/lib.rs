#![no_std]
#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

/// Required bindings for the SBGECom C library.
#[allow(clippy::all)]
mod bindings;
mod data_conversion;
/// This modules contains the Rust API for the SBGECom C library.
/// Covers the implementation of the SBGECom struct and its related functions.
pub mod sbg;
