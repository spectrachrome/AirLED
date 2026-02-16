//! Shared application state accessible from async tasks.
//!
//! Uses [`embassy_sync::mutex::Mutex`] with [`CriticalSectionRawMutex`] so the
//! state can live in a `static` and be shared across embassy tasks.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

/// Active LED pattern.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum PatternMode {
    /// Green/red split sinusoidal pulse.
    SplitPulse,
    /// Green sinusoidal pulse (arming allowed).
    GreenPulse,
    /// Expanding ring ripples on black background.
    Ripple,
    /// Full rainbow cycle.
    Rainbow,
}

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
    /// Number of active LEDs in the strip (1–200).
    pub num_leds: u16,
    /// Target frames per second (1–100).
    pub fps: u8,
    /// Maximum allowed strip current in milliamps.
    pub max_current_ma: u32,
    /// Current flight mode (drives LED pattern selection).
    pub flight_mode: FlightMode,
    /// Active LED pattern.
    pub pattern: PatternMode,
}

impl Default for LedState {
    fn default() -> Self {
        Self {
            brightness: 128,
            num_leds: 180,
            fps: 100,
            max_current_ma: 2000,
            flight_mode: FlightMode::ArmingForbidden,
            pattern: PatternMode::SplitPulse,
        }
    }
}

/// Global shared state protected by an async mutex.
///
/// Lock with `STATE.lock().await` from any embassy task.
pub static STATE: Mutex<CriticalSectionRawMutex, LedState> = Mutex::new(LedState {
    brightness: 128,
    num_leds: 180,
    fps: 100,
    max_current_ma: 2000,
    flight_mode: FlightMode::ArmingForbidden,
    pattern: PatternMode::SplitPulse,
});
