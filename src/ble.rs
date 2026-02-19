//! BLE Nordic UART Service (NUS) protocol layer.
//!
//! Defines the JSON command/response protocol used over BLE NUS.
//! Commands are received as JSON on the RX characteristic, responses
//! are sent as JSON (or plain strings) on the TX characteristic.

use heapless::String as HString;
use serde::{Deserialize, Serialize};

use crate::state::{AnimMode, AnimModeParams, ColorMode, FlightMode, LedState};

/// Maximum number of active LEDs (mirrors `MAX_LEDS` in main).
const MAX_NUM_LEDS: u16 = 200;

// ---------------------------------------------------------------------------
// Command (app → ESP, deserialize from JSON)
// ---------------------------------------------------------------------------

/// Commands received from the app over BLE NUS RX characteristic.
///
/// Uses serde's default externally-tagged representation:
/// `{"GetState":null}` or `{"SetBrightness":{"value":128}}`.
#[derive(Deserialize)]
pub enum Command {
    GetState,
    SetBrightness { value: u8 },
    SetNumLeds { value: u16 },
    SetFps { value: u8 },
    SetMaxCurrent { value: u32 },
    SetColorMode { mode: HString<16> },
    SetAnimMode { mode: HString<16> },
    SetColorBalance { r: u8, g: u8, b: u8 },
    SetUseHsi { value: bool },
    SetHueSpeed { value: u8 },
    SetPulseSpeed { value: u16 },
    SetPulseMinBrightness { value: u8 },
    SetRippleSpeed { value: u8 },
    SetRippleWidth { value: u8 },
    SetRippleDecay { value: u8 },
}

// ---------------------------------------------------------------------------
// Response (ESP → app, serialize to JSON)
// ---------------------------------------------------------------------------

/// Full state snapshot sent to the app.
#[derive(Serialize)]
pub struct StateResponse {
    pub brightness: u8,
    pub num_leds: u16,
    pub fps: u8,
    pub max_current_ma: u32,
    pub color_mode: &'static str,
    pub anim_mode: &'static str,
    pub bal_r: u8,
    pub bal_g: u8,
    pub bal_b: u8,
    pub use_hsi: bool,
    pub hue_speed: u8,
    pub pulse_speed: u16,
    pub pulse_min_brightness: u8,
    pub ripple_speed: u8,
    pub ripple_width: u8,
    pub ripple_decay: u8,
    pub fc_connected: bool,
    pub flight_mode: &'static str,
    pub tx_linked: bool,
}

// ---------------------------------------------------------------------------
// Mapping helpers
// ---------------------------------------------------------------------------

/// Map a [`ColorMode`] to its wire-format string key.
pub fn color_mode_str(mode: ColorMode) -> &'static str {
    match mode {
        ColorMode::SolidGreen => "solid_green",
        ColorMode::SolidRed => "solid_red",
        ColorMode::Split => "split",
        ColorMode::Rainbow => "rainbow",
    }
}

/// Map a [`AnimMode`] to its wire-format string key.
pub fn anim_mode_str(mode: AnimMode) -> &'static str {
    match mode {
        AnimMode::Static => "static",
        AnimMode::Pulse => "pulse",
        AnimMode::Ripple => "ripple",
    }
}

/// Map a [`FlightMode`] to its wire-format string key.
fn flight_mode_str(mode: FlightMode) -> &'static str {
    match mode {
        FlightMode::ArmingForbidden => "arming_forbidden",
        FlightMode::ArmingAllowed => "arming_allowed",
        FlightMode::Armed => "armed",
        FlightMode::Failsafe => "failsafe",
    }
}

/// Parse a color mode string into a [`ColorMode`].
fn parse_color_mode(s: &str) -> Option<ColorMode> {
    match s {
        "solid_green" => Some(ColorMode::SolidGreen),
        "solid_red" => Some(ColorMode::SolidRed),
        "split" => Some(ColorMode::Split),
        "rainbow" => Some(ColorMode::Rainbow),
        _ => None,
    }
}

/// Parse an animation mode string into an [`AnimMode`].
fn parse_anim_mode(s: &str) -> Option<AnimMode> {
    match s {
        "static" => Some(AnimMode::Static),
        "pulse" => Some(AnimMode::Pulse),
        "ripple" => Some(AnimMode::Ripple),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// State snapshot
// ---------------------------------------------------------------------------

/// Build a [`StateResponse`] from the current [`LedState`].
pub fn build_state_response(state: &LedState) -> StateResponse {
    let (pulse_speed, pulse_min_brightness) = match state.anim_params {
        AnimModeParams::Pulse {
            speed,
            min_intensity_pct,
        } => (speed, min_intensity_pct),
        _ => (600, 42),
    };
    let (ripple_speed, ripple_width, ripple_decay) = match state.anim_params {
        AnimModeParams::Ripple {
            speed_x10,
            width_x10,
            decay_pct,
        } => (speed_x10, width_x10, decay_pct),
        _ => (15, 190, 97),
    };

    StateResponse {
        brightness: state.brightness,
        num_leds: state.num_leds,
        fps: state.fps,
        max_current_ma: state.max_current_ma,
        color_mode: color_mode_str(state.color_mode),
        anim_mode: anim_mode_str(state.anim_mode),
        bal_r: state.color_bal_r,
        bal_g: state.color_bal_g,
        bal_b: state.color_bal_b,
        use_hsi: state.use_hsi,
        hue_speed: state.color_params.hue_speed,
        pulse_speed,
        pulse_min_brightness,
        ripple_speed,
        ripple_width,
        ripple_decay,
        fc_connected: state.fc_connected,
        flight_mode: flight_mode_str(state.flight_mode),
        tx_linked: state.tx_linked,
    }
}

// ---------------------------------------------------------------------------
// Command handling
// ---------------------------------------------------------------------------

/// Result of handling a command.
pub enum HandleResult {
    /// Send the full state as JSON.
    SendState,
    /// Send a simple "ok\n" ack.
    Ack,
    /// Send an error string.
    Error(&'static str),
}

/// Apply a [`Command`] to the shared [`LedState`], returning what response to send.
pub fn handle_command(cmd: &Command, state: &mut LedState) -> HandleResult {
    match cmd {
        Command::GetState => HandleResult::SendState,
        Command::SetBrightness { value } => {
            state.brightness = *value;
            HandleResult::Ack
        }
        Command::SetNumLeds { value } => {
            state.num_leds = (*value).clamp(1, MAX_NUM_LEDS);
            HandleResult::Ack
        }
        Command::SetFps { value } => {
            state.fps = (*value).clamp(1, 150);
            HandleResult::Ack
        }
        Command::SetMaxCurrent { value } => {
            state.max_current_ma = (*value).clamp(100, 2500);
            HandleResult::Ack
        }
        Command::SetColorMode { mode } => match parse_color_mode(mode.as_str()) {
            Some(m) => {
                state.color_mode = m;
                HandleResult::Ack
            }
            None => HandleResult::Error("err:unknown_color_mode\n"),
        },
        Command::SetAnimMode { mode } => match parse_anim_mode(mode.as_str()) {
            Some(m) => {
                if m != state.anim_mode {
                    state.anim_mode = m;
                    state.anim_params = AnimModeParams::default_for(m);
                }
                HandleResult::Ack
            }
            None => HandleResult::Error("err:unknown_anim_mode\n"),
        },
        Command::SetColorBalance { r, g, b } => {
            state.color_bal_r = *r;
            state.color_bal_g = *g;
            state.color_bal_b = *b;
            HandleResult::Ack
        }
        Command::SetUseHsi { value } => {
            state.use_hsi = *value;
            HandleResult::Ack
        }
        Command::SetHueSpeed { value } => {
            state.color_params.hue_speed = (*value).clamp(1, 10);
            HandleResult::Ack
        }
        Command::SetPulseSpeed { value } => {
            if let AnimModeParams::Pulse { speed, .. } = &mut state.anim_params {
                *speed = (*value).clamp(100, 2000);
            }
            HandleResult::Ack
        }
        Command::SetPulseMinBrightness { value } => {
            if let AnimModeParams::Pulse {
                min_intensity_pct, ..
            } = &mut state.anim_params
            {
                *min_intensity_pct = (*value).min(80);
            }
            HandleResult::Ack
        }
        Command::SetRippleSpeed { value } => {
            if let AnimModeParams::Ripple { speed_x10, .. } = &mut state.anim_params {
                *speed_x10 = (*value).clamp(5, 50);
            }
            HandleResult::Ack
        }
        Command::SetRippleWidth { value } => {
            if let AnimModeParams::Ripple { width_x10, .. } = &mut state.anim_params {
                *width_x10 = (*value).clamp(10, 255);
            }
            HandleResult::Ack
        }
        Command::SetRippleDecay { value } => {
            if let AnimModeParams::Ripple { decay_pct, .. } = &mut state.anim_params {
                *decay_pct = (*value).clamp(90, 99);
            }
            HandleResult::Ack
        }
    }
}

// ---------------------------------------------------------------------------
// Serialization helpers
// ---------------------------------------------------------------------------

/// Parse a JSON command from a byte slice.
///
/// Returns the parsed command or `None` on failure.
pub fn parse_command(data: &[u8]) -> Option<Command> {
    // Strip trailing newline if present
    let data = if data.last() == Some(&b'\n') {
        &data[..data.len() - 1]
    } else {
        data
    };
    serde_json_core::from_slice::<Command>(data).ok().map(|(cmd, _)| cmd)
}

/// Serialize a [`StateResponse`] into a buffer, appending a newline delimiter.
///
/// Returns the number of bytes written, or `None` if the buffer is too small.
pub fn serialize_state(resp: &StateResponse, buf: &mut [u8]) -> Option<usize> {
    let n = serde_json_core::to_slice(resp, buf).ok()?;
    if n < buf.len() {
        buf[n] = b'\n';
        Some(n + 1)
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_state() -> LedState {
        LedState::default()
    }

    #[test]
    fn parse_get_state() {
        let cmd = parse_command(b"{\"GetState\":null}").unwrap();
        assert!(matches!(cmd, Command::GetState));
    }

    #[test]
    fn parse_get_state_with_newline() {
        let cmd = parse_command(b"{\"GetState\":null}\n").unwrap();
        assert!(matches!(cmd, Command::GetState));
    }

    #[test]
    fn parse_set_brightness() {
        let cmd = parse_command(b"{\"SetBrightness\":{\"value\":128}}").unwrap();
        assert!(matches!(cmd, Command::SetBrightness { value: 128 }));
    }

    #[test]
    fn parse_set_color_mode() {
        let cmd = parse_command(b"{\"SetColorMode\":{\"mode\":\"rainbow\"}}").unwrap();
        if let Command::SetColorMode { mode } = cmd {
            assert_eq!(mode.as_str(), "rainbow");
        } else {
            panic!("expected SetColorMode");
        }
    }

    #[test]
    fn parse_invalid_json() {
        assert!(parse_command(b"not json").is_none());
    }

    #[test]
    fn handle_get_state_returns_send_state() {
        let mut state = default_state();
        let result = handle_command(&Command::GetState, &mut state);
        assert!(matches!(result, HandleResult::SendState));
    }

    #[test]
    fn handle_set_brightness() {
        let mut state = default_state();
        let result = handle_command(&Command::SetBrightness { value: 42 }, &mut state);
        assert!(matches!(result, HandleResult::Ack));
        assert_eq!(state.brightness, 42);
    }

    #[test]
    fn handle_set_num_leds_clamps() {
        let mut state = default_state();
        handle_command(&Command::SetNumLeds { value: 999 }, &mut state);
        assert_eq!(state.num_leds, MAX_NUM_LEDS);
    }

    #[test]
    fn handle_set_anim_mode_resets_params() {
        let mut state = default_state();
        state.anim_mode = AnimMode::Pulse;
        state.anim_params = AnimModeParams::Pulse {
            speed: 1234,
            min_intensity_pct: 77,
        };
        let mode: HString<16> = HString::try_from("ripple").unwrap();
        handle_command(&Command::SetAnimMode { mode }, &mut state);
        assert_eq!(state.anim_mode, AnimMode::Ripple);
        assert!(matches!(state.anim_params, AnimModeParams::Ripple { .. }));
    }

    #[test]
    fn serialize_state_response() {
        let state = default_state();
        let resp = build_state_response(&state);
        let mut buf = [0u8; 512];
        let n = serialize_state(&resp, &mut buf).unwrap();
        let json = core::str::from_utf8(&buf[..n]).unwrap();
        assert!(json.ends_with('\n'));
        assert!(json.contains("\"brightness\""));
        assert!(json.contains("\"color_mode\""));
    }
}
