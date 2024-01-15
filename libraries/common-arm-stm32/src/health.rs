use common_arm::HealthMonitorChannels;

struct HealthManagerSTM32;

impl HealthMonitorChannels for HealthManagerSTM32 {
    fn get_3v3(&self) -> Option<u16> {
        Some(0)
    }
    fn get_5v(&self) -> Option<u16> {
        Some(0)
    }
    fn get_pyro(&self) -> Option<u16> {
        Some(0)
    }
    fn get_vcc(&self) -> Option<u16> {
        Some(0)
    }
    fn get_int_5v(&self) -> Option<u16> {
        Some(0)
    }
    fn get_int_3v3(&self) -> Option<u16> {
        Some(0)
    }
    fn get_ext_5v(&self) -> Option<u16> {
        Some(0)
    }
    fn get_ext_3v3(&self) -> Option<u16> {
        Some(0)
    }
    fn get_failover(&self) -> Option<u16> {
        Some(0)
    }
}
