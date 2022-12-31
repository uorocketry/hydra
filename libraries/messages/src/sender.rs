use defmt::Format;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Debug, Format)]
pub enum Sender {
    GroundStation,
    MainBoard,
}
