use embedded_hal::adc::OneShot;
use embedded_hal::digital::v2::OutputPin;
use crate::health::health_monitor::HealthMonitor;

/// Manages and reports the health of the system.
/// I think OutputPin trait is the right one that should be good for atsame and stm32. 
pub struct HealthManager
{
    monitor: HealthMonitor
}


/// The health status of a component.
/// Nomial: everything is fine  
/// Warning: something is wrong, but it's not critical
/// Error: something is wrong, and it's critical
/// We need some way to quantify this concept of good and bad health.
/// Could be current range or voltage range. If failover happens on the regulators that is warning. 
pub enum HealthStatus {
    Nominal,
    Warning,
    Error,
}


