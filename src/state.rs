//! Shared application state accessible from async tasks.
//!
//! Uses [`embassy_sync::mutex::Mutex`] with [`CriticalSectionRawMutex`] so the
//! state can live in a `static` and be shared across embassy tasks.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;

use crate::dither::DitherMode;

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
                min_intensity_pct: 42,
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
    /// Whether a flight controller is connected via MSP.
    pub fc_connected: bool,
    /// Current flight mode (drives LED pattern selection).
    pub flight_mode: FlightMode,
    /// Active color scheme.
    pub color_mode: ColorMode,
    /// Color-scheme-specific parameters.
    pub color_params: ColorModeParams,
    /// Per-channel color balance: red (0–255).
    pub color_bal_r: u8,
    /// Per-channel color balance: green (0–255).
    pub color_bal_g: u8,
    /// Per-channel color balance: blue (0–255).
    pub color_bal_b: u8,
    /// Use HSI color space instead of HSV for rainbow (more uniform perceived brightness).
    pub use_hsi: bool,
    /// Active animation type.
    pub anim_mode: AnimMode,
    /// Animation-specific parameters.
    pub anim_params: AnimModeParams,
    /// Raw MSP flight mode flags for LED-based debugging (0 = no debug data).
    pub debug_flags: u32,
    /// Index of the ARM box in the BOXNAMES map (255 = not found).
    pub debug_arm_box: u8,
    /// Index of the FAILSAFE box in the BOXNAMES map (255 = not found).
    pub debug_failsafe_box: u8,
    /// AUX strobe intensity (0 = off, nonzero = peak brightness).
    pub aux_strobe: u8,
    /// When true, strobe uses position-light colours (red port / green starboard).
    pub strobe_split: bool,
    /// Whether the RC transmitter has an active link (RSSI > 0).
    pub tx_linked: bool,
    /// Temporal dithering method (Off, ErrorDiffusion, Ordered, Hybrid).
    pub dither_mode: DitherMode,
    /// Dither refresh rate in Hz (100–960). Only used when dither_mode != Off.
    /// Animation updates still happen at `fps` rate; the strip is refreshed
    /// at this rate with dithered sub-frames between animation updates.
    pub dither_fps: u16,
    /// Remaining frames for a BLE test pattern (0 = inactive).
    pub test_pattern_frames: u32,
    /// Color mode for the active test pattern.
    pub test_color: ColorMode,
    /// Animation mode for the active test pattern.
    pub test_anim: AnimMode,
}

impl Default for LedState {
    fn default() -> Self {
        Self {
            brightness: 255,
            num_leds: 180,
            fps: 100,
            max_current_ma: 2000,
            fc_connected: false,
            flight_mode: FlightMode::ArmingForbidden,
            color_mode: ColorMode::Split,
            color_params: ColorModeParams::default(),
            color_bal_r: 255,
            color_bal_g: 180,
            color_bal_b: 240,
            use_hsi: false,
            anim_mode: AnimMode::Pulse,
            anim_params: AnimModeParams::default_for(AnimMode::Pulse),
            debug_flags: 0,
            debug_arm_box: 255,
            debug_failsafe_box: 255,
            aux_strobe: 0,
            strobe_split: false,
            tx_linked: false,
            dither_mode: DitherMode::Off,
            dither_fps: 300,
            test_pattern_frames: 0,
            test_color: ColorMode::Split,
            test_anim: AnimMode::Static,
        }
    }
}

/// BLE flash request: number of flashes (1 = connect, 2 = disconnect).
///
/// The LED task picks this up and plays a blue flash sequence over 750 ms.
pub static BLE_FLASH: Signal<CriticalSectionRawMutex, u8> = Signal::new();

/// Signal to notify the BLE task that state has changed.
///
/// Any task that modifies state can signal this to trigger a BLE push notification.
/// The signal carries no data — the BLE task reads the current state when woken.
pub static STATE_CHANGED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Global shared state protected by an async mutex.
///
/// Lock with `STATE.lock().await` from any embassy task.
pub static STATE: Mutex<CriticalSectionRawMutex, LedState> = Mutex::new(LedState {
    brightness: 255,
    num_leds: 180,
    fps: 100,
    max_current_ma: 2000,
    fc_connected: false,
    flight_mode: FlightMode::ArmingForbidden,
    color_mode: ColorMode::Split,
    color_params: ColorModeParams { hue_speed: 1 },
    color_bal_r: 255,
    color_bal_g: 180,
    color_bal_b: 240,
    use_hsi: false,
    anim_mode: AnimMode::Pulse,
    anim_params: AnimModeParams::Pulse {
        speed: 600,
        min_intensity_pct: 42,
    },
    debug_flags: 0,
    debug_arm_box: 255,
    debug_failsafe_box: 255,
    aux_strobe: 0,
    strobe_split: false,
    tx_linked: false,
    dither_mode: DitherMode::Off,
    dither_fps: 300,
    test_pattern_frames: 0,
    test_color: ColorMode::Split,
    test_anim: AnimMode::Static,
});
