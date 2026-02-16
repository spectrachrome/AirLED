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

/// Per-pattern tunable parameters.
///
/// Uses integer representations to stay `Copy` and avoid float parsing on the
/// embedded side.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum PatternParams {
    /// Parameters for [`PatternMode::SplitPulse`].
    SplitPulse {
        /// Phase increment per frame (higher = faster pulse). Typical range 100–2000.
        speed: u16,
        /// Minimum brightness floor as a percentage (0–100, maps to 0.0–1.0).
        min_intensity_pct: u8,
    },
    /// Parameters for [`PatternMode::GreenPulse`].
    GreenPulse {
        /// Phase increment per frame (higher = faster pulse). Typical range 100–2000.
        speed: u16,
        /// Minimum brightness floor as a percentage (0–100, maps to 0.0–1.0).
        min_intensity_pct: u8,
    },
    /// Parameters for [`PatternMode::Ripple`].
    Ripple {
        /// Ripple expansion speed in fixed-point ×10 (e.g. 15 means 1.5 LEDs/frame).
        speed_x10: u8,
        /// Ring wavefront half-width in fixed-point ×10 (e.g. 190 means 19.0 LEDs).
        width_x10: u8,
        /// Per-frame amplitude decay as a percentage (90–99, maps to 0.90–0.99).
        decay_pct: u8,
    },
    /// Parameters for [`PatternMode::Rainbow`].
    Rainbow {
        /// Hue increment per frame (1–10). Higher = faster color cycling.
        hue_speed: u8,
    },
}

impl PatternParams {
    /// Return sensible defaults for the given pattern mode.
    pub fn default_for(mode: PatternMode) -> Self {
        match mode {
            PatternMode::SplitPulse => PatternParams::SplitPulse {
                speed: 600,
                min_intensity_pct: 40,
            },
            PatternMode::GreenPulse => PatternParams::GreenPulse {
                speed: 600,
                min_intensity_pct: 30,
            },
            PatternMode::Ripple => PatternParams::Ripple {
                speed_x10: 15,
                width_x10: 190,
                decay_pct: 97,
            },
            PatternMode::Rainbow => PatternParams::Rainbow { hue_speed: 1 },
        }
    }
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
    /// Per-pattern tunable parameters.
    pub params: PatternParams,
}

impl Default for LedState {
    fn default() -> Self {
        Self {
            brightness: 255,
            num_leds: 180,
            fps: 100,
            max_current_ma: 2000,
            flight_mode: FlightMode::ArmingForbidden,
            pattern: PatternMode::SplitPulse,
            params: PatternParams::default_for(PatternMode::SplitPulse),
        }
    }
}

/// Global shared state protected by an async mutex.
///
/// Lock with `STATE.lock().await` from any embassy task.
pub static STATE: Mutex<CriticalSectionRawMutex, LedState> = Mutex::new(LedState {
    brightness: 255,
    num_leds: 180,
    fps: 100,
    max_current_ma: 2000,
    flight_mode: FlightMode::ArmingForbidden,
    pattern: PatternMode::SplitPulse,
    params: PatternParams::SplitPulse {
        speed: 600,
        min_intensity_pct: 40,
    },
});
