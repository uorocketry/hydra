#![no_std]

pub trait CommandHandler {

}

pub struct HandlerTest {}

impl CommandHandler for HandlerTest {}

pub struct HotFireStateMachine {
}

impl HotFireStateMachine {
    pub fn new() -> Self {
        HotFireStateMachine {}
    }

    pub fn test(&self, handler: &impl CommandHandler) {

    }
}
