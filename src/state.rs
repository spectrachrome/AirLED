//! Shared application state accessible from async tasks.
//!
//! Uses [`embassy_sync::mutex::Mutex`] with [`CriticalSectionRawMutex`] so the
//! state can live in a `static` and be shared across embassy tasks.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

/// Active color scheme.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum ColorMode {
    /// Solid green.
    SolidGreen,
    /// Solid red.
    SolidRed,
    /// Green/red split (port / starboard).
    Split,
    /// Rainbow HSV gradient.
    Rainbow,
}

/// Active animation type.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum AnimMode {
    /// Static fill (no motion).
    Static,
    /// Sinusoidal breathing pulse.
    Pulse,
    /// Expanding ring ripples.
    Ripple,
}

/// Color-scheme-specific parameters.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct ColorModeParams {
    /// Hue increment per frame (only used by `Rainbow`, ignored otherwise).
    pub hue_speed: u8,
}

impl Default for ColorModeParams {
    fn default() -> Self {
        Self { hue_speed: 1 }
    }
}

/// Animation-specific parameters.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum AnimModeParams {
    /// No extra parameters.
    Static,
    /// Pulse parameters.
    Pulse {
        /// Phase increment per frame (higher = faster pulse). Typical range 100–2000.
        speed: u16,
        /// Minimum brightness floor as a percentage (0–100).
        min_intensity_pct: u8,
    },
    /// Ripple parameters.
    Ripple {
        /// Ripple expansion speed in fixed-point x10 (e.g. 15 = 1.5 LEDs/frame).
        speed_x10: u8,
        /// Ring wavefront half-width in fixed-point x10 (e.g. 190 = 19.0 LEDs).
        width_x10: u8,
        /// Per-frame amplitude decay as a percentage (90–99).
        decay_pct: u8,
    },
}

impl AnimModeParams {
    /// Return sensible defaults for the given animation mode.
    pub fn default_for(mode: AnimMode) -> Self {
        match mode {
            AnimMode::Static => AnimModeParams::Static,
            AnimMode::Pulse => AnimModeParams::Pulse {
                speed: 600,
                min_intensity_pct: 40,
            },
            AnimMode::Ripple => AnimModeParams::Ripple {
                speed_x10: 15,
                width_x10: 190,
                decay_pct: 97,
            },
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
    /// Active color scheme.
    pub color_mode: ColorMode,
    /// Color-scheme-specific parameters.
    pub color_params: ColorModeParams,
    /// Active animation type.
    pub anim_mode: AnimMode,
    /// Animation-specific parameters.
    pub anim_params: AnimModeParams,
}

impl Default for LedState {
    fn default() -> Self {
        Self {
            brightness: 255,
            num_leds: 180,
            fps: 100,
            max_current_ma: 2000,
            flight_mode: FlightMode::ArmingForbidden,
            color_mode: ColorMode::Split,
            color_params: ColorModeParams::default(),
            anim_mode: AnimMode::Pulse,
            anim_params: AnimModeParams::default_for(AnimMode::Pulse),
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
    color_mode: ColorMode::Split,
    color_params: ColorModeParams { hue_speed: 1 },
    anim_mode: AnimMode::Pulse,
    anim_params: AnimModeParams::Pulse {
        speed: 600,
        min_intensity_pct: 40,
    },
});
