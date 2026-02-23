//! BLE Nordic UART Service (NUS) binary protocol layer.
//!
//! Implements the AirLED Binary BLE Protocol v1 (see `docs/binary_protocol.md`).
//! Commands are received as compact binary frames on the RX characteristic,
//! responses are sent as binary on the TX characteristic.

use crate::dither::DitherMode;
use crate::state::{AnimMode, AnimModeParams, ColorMode, FlightMode, LedState};

/// Maximum number of active LEDs (mirrors `MAX_LEDS` in main).
const MAX_NUM_LEDS: u16 = 200;

/// Current protocol version.
const PROTOCOL_VERSION: u8 = 1;

/// Firmware version (from Cargo.toml).
const FW_MAJOR: u8 = 0;
const FW_MINOR: u8 = 1;
const FW_PATCH: u8 = 0;

// ---------------------------------------------------------------------------
// Command IDs (app → device)
// ---------------------------------------------------------------------------

const CMD_GET_STATE: u8 = 0x01;
const CMD_GET_VERSION: u8 = 0x02;
const CMD_SET_BRIGHTNESS: u8 = 0x10;
const CMD_SET_NUM_LEDS: u8 = 0x11;
const CMD_SET_FPS: u8 = 0x12;
const CMD_SET_MAX_CURRENT: u8 = 0x13;
const CMD_SET_COLOR_MODE: u8 = 0x14;
const CMD_SET_ANIM_MODE: u8 = 0x15;
const CMD_SET_COLOR_BALANCE: u8 = 0x16;
const CMD_SET_USE_HSI: u8 = 0x17;
const CMD_SET_HUE_SPEED: u8 = 0x18;
const CMD_SET_PULSE_SPEED: u8 = 0x19;
const CMD_SET_PULSE_MIN_BRT: u8 = 0x1A;
const CMD_SET_RIPPLE_SPEED: u8 = 0x1B;
const CMD_SET_RIPPLE_WIDTH: u8 = 0x1C;
const CMD_SET_RIPPLE_DECAY: u8 = 0x1D;
const CMD_SET_DITHER_MODE: u8 = 0x1E;
const CMD_SET_DITHER_FPS: u8 = 0x1F;
const CMD_DISPLAY_TEST_PATTERN: u8 = 0xF0;
const CMD_CANCEL_TEST_PATTERN: u8 = 0xF1;

// ---------------------------------------------------------------------------
// Response codes (device → app)
// ---------------------------------------------------------------------------

/// Command accepted.
pub const RSP_OK: u8 = 0x00;
/// State snapshot response ID.
pub const RSP_STATE: u8 = 0x01;
/// Version response ID.
pub const RSP_VERSION: u8 = 0x02;
/// Unknown or malformed command.
pub const RSP_ERR_PARSE: u8 = 0xE0;
/// Value out of valid range.
pub const RSP_ERR_RANGE: u8 = 0xE1;

// ---------------------------------------------------------------------------
// Result type
// ---------------------------------------------------------------------------

/// Result of handling a binary command.
pub enum HandleResult {
    /// Send the full state snapshot (25 bytes).
    SendState,
    /// Send the version response (5 bytes).
    SendVersion,
    /// Send a 1-byte ack/error code.
    Ack(u8),
}

// ---------------------------------------------------------------------------
// Command parsing + handling (combined)
// ---------------------------------------------------------------------------

/// Parse and apply a binary command to the shared [`LedState`].
///
/// Returns what response to send, or a 1-byte error code on failure.
pub fn handle_binary_command(data: &[u8], state: &mut LedState) -> HandleResult {
    if data.is_empty() {
        return HandleResult::Ack(RSP_ERR_PARSE);
    }

    match data[0] {
        CMD_GET_STATE => HandleResult::SendState,
        CMD_GET_VERSION => HandleResult::SendVersion,

        CMD_SET_BRIGHTNESS if data.len() >= 2 => {
            state.brightness = data[1];
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_NUM_LEDS if data.len() >= 3 => {
            let val = u16::from_le_bytes([data[1], data[2]]);
            if !(1..=MAX_NUM_LEDS).contains(&val) {
                return HandleResult::Ack(RSP_ERR_RANGE);
            }
            state.num_leds = val;
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_FPS if data.len() >= 2 => {
            let val = data[1];
            if !(1..=150).contains(&val) {
                return HandleResult::Ack(RSP_ERR_RANGE);
            }
            state.fps = val;
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_MAX_CURRENT if data.len() >= 3 => {
            let val = u16::from_le_bytes([data[1], data[2]]);
            if !(100..=2500).contains(&val) {
                return HandleResult::Ack(RSP_ERR_RANGE);
            }
            state.max_current_ma = val as u32;
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_COLOR_MODE if data.len() >= 2 => {
            let mode = match data[1] {
                0 => ColorMode::SolidGreen,
                1 => ColorMode::SolidRed,
                2 => ColorMode::Split,
                3 => ColorMode::Rainbow,
                _ => return HandleResult::Ack(RSP_ERR_RANGE),
            };
            state.color_mode = mode;
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_ANIM_MODE if data.len() >= 2 => {
            let mode = match data[1] {
                0 => AnimMode::Static,
                1 => AnimMode::Pulse,
                2 => AnimMode::Ripple,
                _ => return HandleResult::Ack(RSP_ERR_RANGE),
            };
            if mode != state.anim_mode {
                state.anim_mode = mode;
                state.anim_params = AnimModeParams::default_for(mode);
            }
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_COLOR_BALANCE if data.len() >= 4 => {
            state.color_bal_r = data[1];
            state.color_bal_g = data[2];
            state.color_bal_b = data[3];
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_USE_HSI if data.len() >= 2 => {
            state.use_hsi = data[1] != 0;
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_HUE_SPEED if data.len() >= 2 => {
            let val = data[1];
            if !(1..=10).contains(&val) {
                return HandleResult::Ack(RSP_ERR_RANGE);
            }
            state.color_params.hue_speed = val;
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_PULSE_SPEED if data.len() >= 3 => {
            let val = u16::from_le_bytes([data[1], data[2]]);
            if !(100..=2000).contains(&val) {
                return HandleResult::Ack(RSP_ERR_RANGE);
            }
            if let AnimModeParams::Pulse { speed, .. } = &mut state.anim_params {
                *speed = val;
            }
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_PULSE_MIN_BRT if data.len() >= 2 => {
            let val = data[1];
            if val > 80 {
                return HandleResult::Ack(RSP_ERR_RANGE);
            }
            if let AnimModeParams::Pulse {
                min_intensity_pct, ..
            } = &mut state.anim_params
            {
                *min_intensity_pct = val;
            }
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_RIPPLE_SPEED if data.len() >= 2 => {
            let val = data[1];
            if !(5..=50).contains(&val) {
                return HandleResult::Ack(RSP_ERR_RANGE);
            }
            if let AnimModeParams::Ripple { speed_x10, .. } = &mut state.anim_params {
                *speed_x10 = val;
            }
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_RIPPLE_WIDTH if data.len() >= 2 => {
            let val = data[1];
            if val < 10 {
                return HandleResult::Ack(RSP_ERR_RANGE);
            }
            if let AnimModeParams::Ripple { width_x10, .. } = &mut state.anim_params {
                *width_x10 = val;
            }
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_RIPPLE_DECAY if data.len() >= 2 => {
            let val = data[1];
            if !(90..=99).contains(&val) {
                return HandleResult::Ack(RSP_ERR_RANGE);
            }
            if let AnimModeParams::Ripple { decay_pct, .. } = &mut state.anim_params {
                *decay_pct = val;
            }
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_DITHER_MODE if data.len() >= 2 => {
            let mode = match data[1] {
                0 => DitherMode::Off,
                1 => DitherMode::ErrorDiffusion,
                2 => DitherMode::Ordered,
                3 => DitherMode::Hybrid,
                _ => return HandleResult::Ack(RSP_ERR_RANGE),
            };
            state.dither_mode = mode;
            HandleResult::Ack(RSP_OK)
        }
        CMD_SET_DITHER_FPS if data.len() >= 3 => {
            let val = u16::from_le_bytes([data[1], data[2]]);
            if !(100..=960).contains(&val) {
                return HandleResult::Ack(RSP_ERR_RANGE);
            }
            state.dither_fps = val;
            HandleResult::Ack(RSP_OK)
        }

        // DisplayTestPattern: [color_mode, anim_mode, duration_ms(u16 LE)]
        CMD_DISPLAY_TEST_PATTERN if data.len() >= 5 => {
            let color = match data[1] {
                0 => ColorMode::SolidGreen,
                1 => ColorMode::SolidRed,
                2 => ColorMode::Split,
                3 => ColorMode::Rainbow,
                _ => return HandleResult::Ack(RSP_ERR_RANGE),
            };
            let anim = match data[2] {
                0 => AnimMode::Static,
                1 => AnimMode::Pulse,
                2 => AnimMode::Ripple,
                _ => return HandleResult::Ack(RSP_ERR_RANGE),
            };
            let duration_ms = u16::from_le_bytes([data[3], data[4]]);
            let frames = (state.fps as u32 * duration_ms as u32) / 1000;
            state.test_color = color;
            state.test_anim = anim;
            state.test_pattern_frames = frames.max(1);
            HandleResult::Ack(RSP_OK)
        }
        CMD_CANCEL_TEST_PATTERN => {
            state.test_pattern_frames = 0;
            HandleResult::Ack(RSP_OK)
        }

        // Known command ID but insufficient payload bytes
        CMD_SET_BRIGHTNESS | CMD_SET_FPS | CMD_SET_COLOR_MODE | CMD_SET_ANIM_MODE
        | CMD_SET_USE_HSI | CMD_SET_HUE_SPEED | CMD_SET_PULSE_MIN_BRT
        | CMD_SET_RIPPLE_SPEED | CMD_SET_RIPPLE_WIDTH | CMD_SET_RIPPLE_DECAY
        | CMD_SET_DITHER_MODE | CMD_DISPLAY_TEST_PATTERN => {
            HandleResult::Ack(RSP_ERR_PARSE)
        }
        CMD_SET_NUM_LEDS | CMD_SET_MAX_CURRENT | CMD_SET_PULSE_SPEED
        | CMD_SET_COLOR_BALANCE | CMD_SET_DITHER_FPS => HandleResult::Ack(RSP_ERR_PARSE),

        _ => HandleResult::Ack(RSP_ERR_PARSE),
    }
}

// ---------------------------------------------------------------------------
// State encoding
// ---------------------------------------------------------------------------

/// Encode the full state snapshot into `buf` (must be >= 25 bytes).
///
/// Returns the number of bytes written (always 25).
pub fn encode_state(state: &LedState, buf: &mut [u8]) -> usize {
    let (pulse_speed, pulse_min_brt) = match state.anim_params {
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

    let color_mode: u8 = match state.color_mode {
        ColorMode::SolidGreen => 0,
        ColorMode::SolidRed => 1,
        ColorMode::Split => 2,
        ColorMode::Rainbow => 3,
    };
    let anim_mode: u8 = match state.anim_mode {
        AnimMode::Static => 0,
        AnimMode::Pulse => 1,
        AnimMode::Ripple => 2,
    };
    let dither_mode: u8 = match state.dither_mode {
        DitherMode::Off => 0,
        DitherMode::ErrorDiffusion => 1,
        DitherMode::Ordered => 2,
        DitherMode::Hybrid => 3,
    };

    let flags: u8 = (state.fc_connected as u8)
        | ((state.tx_linked as u8) << 1)
        | ((matches!(state.flight_mode, FlightMode::Armed) as u8) << 2)
        | ((matches!(state.flight_mode, FlightMode::Failsafe) as u8) << 3)
        | ((matches!(state.flight_mode, FlightMode::ArmingAllowed) as u8) << 4)
        | (((state.test_pattern_frames > 0) as u8) << 5)
        | ((state.strobe_split as u8) << 6);

    let num_leds = state.num_leds.to_le_bytes();
    let max_current = (state.max_current_ma as u16).to_le_bytes();
    let pulse_speed_le = pulse_speed.to_le_bytes();
    let dither_fps_le = state.dither_fps.to_le_bytes();

    buf[0] = RSP_STATE;
    buf[1] = PROTOCOL_VERSION;
    buf[2] = state.brightness;
    buf[3] = num_leds[0];
    buf[4] = num_leds[1];
    buf[5] = state.fps;
    buf[6] = max_current[0];
    buf[7] = max_current[1];
    buf[8] = color_mode;
    buf[9] = anim_mode;
    buf[10] = state.color_bal_r;
    buf[11] = state.color_bal_g;
    buf[12] = state.color_bal_b;
    buf[13] = state.use_hsi as u8;
    buf[14] = state.color_params.hue_speed;
    buf[15] = pulse_speed_le[0];
    buf[16] = pulse_speed_le[1];
    buf[17] = pulse_min_brt;
    buf[18] = ripple_speed;
    buf[19] = ripple_width;
    buf[20] = ripple_decay;
    buf[21] = flags;
    buf[22] = dither_mode;
    buf[23] = dither_fps_le[0];
    buf[24] = dither_fps_le[1];

    25
}

/// Encode the version response into `buf` (must be >= 5 bytes).
///
/// Returns the number of bytes written (always 5).
pub fn encode_version(buf: &mut [u8]) -> usize {
    buf[0] = RSP_VERSION;
    buf[1] = PROTOCOL_VERSION;
    buf[2] = FW_MAJOR;
    buf[3] = FW_MINOR;
    buf[4] = FW_PATCH;
    5
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
    fn get_state_returns_send_state() {
        let mut state = default_state();
        let result = handle_binary_command(&[CMD_GET_STATE], &mut state);
        assert!(matches!(result, HandleResult::SendState));
    }

    #[test]
    fn get_version_returns_send_version() {
        let mut state = default_state();
        let result = handle_binary_command(&[CMD_GET_VERSION], &mut state);
        assert!(matches!(result, HandleResult::SendVersion));
    }

    #[test]
    fn set_brightness() {
        let mut state = default_state();
        let result = handle_binary_command(&[CMD_SET_BRIGHTNESS, 42], &mut state);
        assert!(matches!(result, HandleResult::Ack(RSP_OK)));
        assert_eq!(state.brightness, 42);
    }

    #[test]
    fn set_num_leds() {
        let mut state = default_state();
        let val: u16 = 100;
        let bytes = val.to_le_bytes();
        let result =
            handle_binary_command(&[CMD_SET_NUM_LEDS, bytes[0], bytes[1]], &mut state);
        assert!(matches!(result, HandleResult::Ack(RSP_OK)));
        assert_eq!(state.num_leds, 100);
    }

    #[test]
    fn set_num_leds_out_of_range() {
        let mut state = default_state();
        let val: u16 = 999;
        let bytes = val.to_le_bytes();
        let result =
            handle_binary_command(&[CMD_SET_NUM_LEDS, bytes[0], bytes[1]], &mut state);
        assert!(matches!(result, HandleResult::Ack(RSP_ERR_RANGE)));
    }

    #[test]
    fn set_color_mode_rainbow() {
        let mut state = default_state();
        let result = handle_binary_command(&[CMD_SET_COLOR_MODE, 3], &mut state);
        assert!(matches!(result, HandleResult::Ack(RSP_OK)));
        assert_eq!(state.color_mode, ColorMode::Rainbow);
    }

    #[test]
    fn set_color_mode_invalid() {
        let mut state = default_state();
        let result = handle_binary_command(&[CMD_SET_COLOR_MODE, 99], &mut state);
        assert!(matches!(result, HandleResult::Ack(RSP_ERR_RANGE)));
    }

    #[test]
    fn set_anim_mode_resets_params() {
        let mut state = default_state();
        state.anim_mode = AnimMode::Pulse;
        state.anim_params = AnimModeParams::Pulse {
            speed: 1234,
            min_intensity_pct: 77,
        };
        let result = handle_binary_command(&[CMD_SET_ANIM_MODE, 2], &mut state);
        assert!(matches!(result, HandleResult::Ack(RSP_OK)));
        assert_eq!(state.anim_mode, AnimMode::Ripple);
        assert!(matches!(state.anim_params, AnimModeParams::Ripple { .. }));
    }

    #[test]
    fn set_color_balance() {
        let mut state = default_state();
        let result =
            handle_binary_command(&[CMD_SET_COLOR_BALANCE, 0xFF, 0xC8, 0xDC], &mut state);
        assert!(matches!(result, HandleResult::Ack(RSP_OK)));
        assert_eq!(state.color_bal_r, 255);
        assert_eq!(state.color_bal_g, 200);
        assert_eq!(state.color_bal_b, 220);
    }

    #[test]
    fn set_dither_mode() {
        let mut state = default_state();
        let result = handle_binary_command(&[CMD_SET_DITHER_MODE, 3], &mut state);
        assert!(matches!(result, HandleResult::Ack(RSP_OK)));
        assert_eq!(state.dither_mode, DitherMode::Hybrid);
    }

    #[test]
    fn set_dither_fps() {
        let mut state = default_state();
        let val: u16 = 480;
        let bytes = val.to_le_bytes();
        let result =
            handle_binary_command(&[CMD_SET_DITHER_FPS, bytes[0], bytes[1]], &mut state);
        assert!(matches!(result, HandleResult::Ack(RSP_OK)));
        assert_eq!(state.dither_fps, 480);
    }

    #[test]
    fn display_test_pattern() {
        let mut state = default_state();
        // color=rainbow(3), anim=pulse(1), duration=2000ms LE
        let dur: u16 = 2000;
        let dur_le = dur.to_le_bytes();
        let result = handle_binary_command(
            &[CMD_DISPLAY_TEST_PATTERN, 3, 1, dur_le[0], dur_le[1]],
            &mut state,
        );
        assert!(matches!(result, HandleResult::Ack(RSP_OK)));
        assert_eq!(state.test_color, ColorMode::Rainbow);
        assert_eq!(state.test_anim, AnimMode::Pulse);
        assert!(state.test_pattern_frames > 0);
    }

    #[test]
    fn cancel_test_pattern() {
        let mut state = default_state();
        state.test_pattern_frames = 100;
        let result = handle_binary_command(&[CMD_CANCEL_TEST_PATTERN], &mut state);
        assert!(matches!(result, HandleResult::Ack(RSP_OK)));
        assert_eq!(state.test_pattern_frames, 0);
    }

    #[test]
    fn empty_command_returns_parse_error() {
        let mut state = default_state();
        let result = handle_binary_command(&[], &mut state);
        assert!(matches!(result, HandleResult::Ack(RSP_ERR_PARSE)));
    }

    #[test]
    fn unknown_command_returns_parse_error() {
        let mut state = default_state();
        let result = handle_binary_command(&[0xFF], &mut state);
        assert!(matches!(result, HandleResult::Ack(RSP_ERR_PARSE)));
    }

    #[test]
    fn truncated_command_returns_parse_error() {
        let mut state = default_state();
        // CMD_SET_NUM_LEDS needs 3 bytes, only 2 provided
        let result = handle_binary_command(&[CMD_SET_NUM_LEDS, 0x01], &mut state);
        assert!(matches!(result, HandleResult::Ack(RSP_ERR_PARSE)));
    }

    #[test]
    fn encode_state_snapshot() {
        let state = default_state();
        let mut buf = [0u8; 32];
        let n = encode_state(&state, &mut buf);
        assert_eq!(n, 25);
        assert_eq!(buf[0], RSP_STATE);
        assert_eq!(buf[1], PROTOCOL_VERSION);
        assert_eq!(buf[2], state.brightness);
        // num_leds = 180 = 0x00B4 LE
        assert_eq!(buf[3], 0xB4);
        assert_eq!(buf[4], 0x00);
        assert_eq!(buf[5], state.fps);
        // flags: fc_connected=false → 0x00
        assert_eq!(buf[21], 0x00);
        // dither_mode = Off = 0
        assert_eq!(buf[22], 0x00);
    }

    #[test]
    fn encode_version_response() {
        let mut buf = [0u8; 8];
        let n = encode_version(&mut buf);
        assert_eq!(n, 5);
        assert_eq!(buf[0], RSP_VERSION);
        assert_eq!(buf[1], PROTOCOL_VERSION);
        assert_eq!(buf[2], FW_MAJOR);
        assert_eq!(buf[3], FW_MINOR);
        assert_eq!(buf[4], FW_PATCH);
    }

    #[test]
    fn encode_state_flags() {
        let mut state = default_state();
        state.fc_connected = true;
        state.tx_linked = true;
        state.flight_mode = FlightMode::Armed;
        let mut buf = [0u8; 25];
        encode_state(&state, &mut buf);
        // flags: fc_connected=1, tx_linked=1<<1, armed=1<<2 = 0b00000111 = 0x07
        assert_eq!(buf[21], 0x07);
    }

    #[test]
    fn encode_state_flags_test_active_and_strobe_split() {
        let mut state = default_state();
        state.test_pattern_frames = 50;
        state.strobe_split = true;
        let mut buf = [0u8; 25];
        encode_state(&state, &mut buf);
        // flags: test_active=1<<5, strobe_split=1<<6 = 0b01100000 = 0x60
        assert_eq!(buf[21], 0x60);
    }
}
