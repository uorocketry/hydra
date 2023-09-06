use atsamd_hal::gpio::{Pin, PushPullOutput, PA09, PA06};
use atsamd_hal::prelude::*;
use messages::sender::Sender;
use messages::sender::Sender::RecoveryBoard;

// -------
// Sender ID
// -------
pub static COM_ID: Sender = RecoveryBoard;