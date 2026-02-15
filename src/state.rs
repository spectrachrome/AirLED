//! Shared application state accessible from async tasks.
//!
//! Uses [`embassy_sync::mutex::Mutex`] with [`CriticalSectionRawMutex`] so the
//! state can live in a `static` and be shared across embassy tasks.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

/// Current flight / arming mode of the drone.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum FlightMode {
    /// Motors cannot be armed (pre-flight / disarmed).
    ArmingForbidden,
    /// Arming is allowed but motors are not yet spinning.
    ArmingAllowed,
    /// Motors are armed and active.
    Armed,
    /// Communication lost — failsafe behaviour active.
    Failsafe,
}

/// LED-related runtime configuration.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct LedState {
    /// Global brightness (0–255).
    pub brightness: u8,
    /// Current flight mode (drives LED pattern selection).
    pub flight_mode: FlightMode,
}

impl Default for LedState {
    fn default() -> Self {
        Self {
            brightness: 32,
            flight_mode: FlightMode::ArmingForbidden,
        }
    }
}

/// Global shared state protected by an async mutex.
///
/// Lock with `STATE.lock().await` from any embassy task.
pub static STATE: Mutex<CriticalSectionRawMutex, LedState> = Mutex::new(LedState {
    brightness: 32,
    flight_mode: FlightMode::ArmingForbidden,
});
