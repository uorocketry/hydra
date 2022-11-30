use core::ffi::c_char;
use core::ptr;
use core::ptr::{null, null_mut};
use crate::bindings;
use crate::bindings::{_SbgInterface, SbgInterfaceHandle};

struct UARTSBGInterface {
    interface: bindings::SbgInterface
}

struct SBG {
    UARTSBGInterface: UARTSBGInterface
}

impl SBG {
    fn new() -> Self {
        let interface = UARTSBGInterface {
            interface: _SbgInterface {
                handle: null_mut(),
                type_: 0,
                name: [0; 48],
                pDestroyFunc: None,
                pWriteFunc: None,
                pReadFunc: None,
                pFlushFunc: None,
                pSetSpeedFunc: None,
                pGetSpeedFunc: None,
                pDelayFunc: None,
            },
        };

        SBG {
            UARTSBGInterface: interface,
        }
    }

    fn read() {
    }
}