//! # HYDRA Error Management
//! Using errors in a non-std environment can be non-trivial. This module contains various
//! structs and macros to simplify and standardize error management in HYDRA.
//!
//! ## HydraError
//! [`HydraError`] is the standard error type to be used throughout the HYDRA codebase. It is an enum
//! that wraps all possible error types that can be encountered. This allows functions to have a
//! standard `Result<_, HydraError` that can be returned. The `From` trait is also automatically
//! implemented for all error variants, allowing the use of the `?` operator. For example:
//! ```ignore
//! fn send_message() -> Result<(), HydraError> {
//!     let payload: Vec<u8, 64> = to_vec_cobs(&m)?;
//!
//!     for x in payload {
//!         // Note that this error is different from the error type above
//!         block!(uart.write(x))?;
//!     }
//!
//!     Ok(())
//! }
//! ```
//!
//! ## ErrorManager
//! While [`HydraError`] standardizes the error type, something must be done when an error happens.
//! The [`ErrorManager`] struct provides central management for all errors. A single instance should
//! be created in an application, and all errors should be sent to it. It will then handle those
//! errors as needed, and store them for possible debugging.
//!
//! There are two mains ways to call the error manager. For simple and short code, a closure can
//! be passed directly to it:
//! ```
//! # use common_arm::ErrorManager;
//! let em = ErrorManager::new();
//!
//! em.run(|| {
//!     // Do something that can throw an error
//!     # Ok(())
//! });
//! ```
//!
//! For more complex code, it may be better to simply pass the result directly to it:
//! ```
//! # use common_arm::{ErrorManager, HydraError};
//! fn foo() -> Result<(), HydraError> {
//!     // Do something that can throw an error
//!     # Ok(())
//! }
//!
//! let em = ErrorManager::new();
//!
//! em.handle(foo());
//! ```
//!
//! ### RTIC
//! [`ErrorManager`] is designed to be easily used with RTIC, and uses interior mutability to handle
//! the errors. This makes it possible to share and access an instance without using locks:
//! ```ignore
//! #[task(shared = [&em])]
//! fn foo(cx: foo::Context, m: Message) {
//!     cx.shared.em.run(|| {
//!         // ...
//!     });
//! }
//! ```
//!
//! ## RTIC Task Spawning
//! Spawning software tasks in RTIC can fail for various reasons. However, the `Result` returned
//! by spawning a task is not standard, making it challenging to transform it into a
//! [`HydraError`] while preserving useful info like the task name that failed to spawn.
//!
//! The macros [`spawn`](crate::spawn!) and [`spawn_after`](crate::spawn_after!) are there to facilitate
//! handling the errors that can be returned while spawning. These simply spawn the given task,
//! converting the returned `Result` into a `Result<(), HydraError>`, adding in the process some
//! useful info to the error like the task name.
//!
//! Here is an example of those macros used with the error manager:
//! ```ignore
//! em.run(|| {
//!     spawn!(send_message, message);
//!     spawn_after!(sensor_send, 2.secs());
//! });
//! ```
//!

pub mod error_manager;
pub mod hydra_error;

/// Calls `spawn` on a RTIC's task. Transforms the returned `Result` into a `Result<_, HydraError>`.
#[macro_export]
macro_rules! spawn {
    ($task:ident$(,)? $($arg:expr),*) => {
        $crate::SpawnError::spawn_error($task::spawn($($arg),*), stringify!($task))
    };
}

/// Calls `spawn_after` on a RTIC's task. Transforms the returned `Result` into a `Result<_, HydraError>`.
#[macro_export]
macro_rules! spawn_after {
    ($task:ident$(,)? $($arg:expr),*) => {
        $crate::SpawnError::spawn_error($task::spawn_after($($arg),*), stringify!($task))
    };
}
