use crate::error::hydra_error::HydraError;
use crate::herror;
use core::cell::RefCell;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::Relaxed;
use cortex_m::interrupt;
use cortex_m::interrupt::Mutex;
use defmt::error;
use heapless::HistoryBuffer;

/// Central error management for HYDRA. A single instance of this should be created for each board.
pub struct ErrorManager {
    has_error: AtomicBool,
    error_history: Mutex<RefCell<HistoryBuffer<HydraError, 8>>>,
}

impl Default for ErrorManager {
    fn default() -> Self {
        ErrorManager::new()
    }
}

impl ErrorManager {
    pub fn new() -> Self {
        ErrorManager {
            has_error: false.into(),
            error_history: Mutex::new(RefCell::new(HistoryBuffer::new())),
        }
    }

    /// Runs the given closure. [`ErrorManager::handle()`] will be called on the closure's result.
    pub fn run<F>(&self, callback: F)
    where
        F: FnOnce() -> Result<(), HydraError>,
    {
        let result = callback();
        self.handle(result);
    }

    /// Handles any possible errors. This will store the error and log it using defmt.
    pub fn handle(&self, result: Result<(), HydraError>) {
        if let Err(e) = result {
            self.has_error.store(true, Relaxed);

            if let Some(c) = e.get_context() {
                error!("{}", e);
                herror!(Error, c);
            }

            interrupt::free(|cs| {
                self.error_history.borrow(cs).borrow_mut().write(e);
            });
        }
    }

    /// Returns if any error has been raised.
    pub fn has_error(&self) -> bool {
        self.has_error.load(Relaxed)
    }
}
