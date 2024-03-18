use messages_proc_macros_lib::common_derives;

#[common_derives]
pub struct State {
    pub data: StateData,
}

#[common_derives]
pub enum StateData {
    Initializing,
    WaitForTakeoff,
    Ascent,
    Descent,
    TerminalDescent,
    WaitForRecovery,
    Abort,
}

impl State {
    pub fn new(data: impl Into<StateData>) -> Self {
        State { data: data.into() }
    }
}
