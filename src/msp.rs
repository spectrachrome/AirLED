//! MSPv1 protocol codec for Betaflight flight controller communication.
//!
//! Pure protocol logic — no hardware dependency, no heap. Fully unit-testable.
//!
//! Implements frame building, response parsing (state machine), BOXNAMES
//! decoding, and flight mode resolution.

use crate::state::FlightMode;

/// MSP command: status (flight mode flags, cycle time, etc.).
pub const MSP_STATUS: u8 = 101;

/// MSP command: box names (semicolon-separated list of mode names).
pub const MSP_BOXNAMES: u8 = 116;

/// MSP command: box IDs (permanent numeric IDs, one byte each).
pub const MSP_BOXIDS: u8 = 119;

/// Betaflight permanent box ID for ARM.
const PERM_ID_ARM: u8 = 0;

/// Betaflight permanent box ID for FAILSAFE.
const PERM_ID_FAILSAFE: u8 = 27;

/// Maximum payload size for an MSP frame.
const MAX_PAYLOAD: usize = 256;

/// Maximum number of mode boxes supported.
const MAX_BOXES: usize = 48;

// ---------------------------------------------------------------------------
// Frame builder
// ---------------------------------------------------------------------------

/// Build an MSPv1 request frame into `buf`.
///
/// Format: `$M<` + size(1) + cmd(1) + payload(N) + xor_checksum(1)
///
/// Returns the number of bytes written (always `payload.len() + 6`).
///
/// # Panics
///
/// Panics if `buf` is too small to hold the frame.
pub fn build_request(cmd: u8, payload: &[u8], buf: &mut [u8]) -> usize {
    let size = payload.len() as u8;
    let frame_len = payload.len() + 6;
    assert!(buf.len() >= frame_len);

    buf[0] = b'$';
    buf[1] = b'M';
    buf[2] = b'<';
    buf[3] = size;
    buf[4] = cmd;

    let mut crc = size ^ cmd;
    for (i, &b) in payload.iter().enumerate() {
        buf[5 + i] = b;
        crc ^= b;
    }
    buf[5 + payload.len()] = crc;

    frame_len
}

// ---------------------------------------------------------------------------
// Response parser (state machine)
// ---------------------------------------------------------------------------

/// A parsed MSP response frame.
pub struct MspFrame {
    /// Command ID.
    pub cmd: u8,
    /// Payload size.
    pub size: u8,
    /// Payload buffer (only first `size` bytes are valid).
    pub payload: [u8; MAX_PAYLOAD],
}

/// Result of feeding a byte into the parser.
#[allow(clippy::large_enum_variant)] // no_std: can't box, only one instance exists
pub enum ParseResult {
    /// Need more bytes.
    Incomplete,
    /// Complete valid frame.
    Frame(MspFrame),
    /// Protocol error (bad header, checksum mismatch, error frame).
    Error,
}

/// Parser states.
#[derive(Clone, Copy)]
enum State {
    Idle,
    GotDollar,
    GotM,
    GotDir,
    GotSize,
    Payload,
    Checksum,
}

impl Default for MspParser {
    fn default() -> Self {
        Self::new()
    }
}

/// MSPv1 response parser.
///
/// Feed bytes one at a time via [`MspParser::feed`]. The parser tracks its
/// internal state and returns a [`ParseResult`] for each byte.
pub struct MspParser {
    state: State,
    cmd: u8,
    size: u8,
    payload: [u8; MAX_PAYLOAD],
    payload_idx: u8,
    crc: u8,
}

impl MspParser {
    /// Create a new parser in the idle state.
    pub fn new() -> Self {
        Self {
            state: State::Idle,
            cmd: 0,
            size: 0,
            payload: [0; MAX_PAYLOAD],
            payload_idx: 0,
            crc: 0,
        }
    }

    /// Reset the parser to its initial state.
    pub fn reset(&mut self) {
        self.state = State::Idle;
    }

    /// Feed a single byte into the parser.
    pub fn feed(&mut self, byte: u8) -> ParseResult {
        match self.state {
            State::Idle => {
                if byte == b'$' {
                    self.state = State::GotDollar;
                }
                ParseResult::Incomplete
            }
            State::GotDollar => {
                if byte == b'M' {
                    self.state = State::GotM;
                    ParseResult::Incomplete
                } else {
                    self.state = State::Idle;
                    ParseResult::Error
                }
            }
            State::GotM => {
                match byte {
                    b'>' => {
                        self.state = State::GotDir;
                        ParseResult::Incomplete
                    }
                    b'!' => {
                        // Error frame from FC
                        self.state = State::Idle;
                        ParseResult::Error
                    }
                    _ => {
                        self.state = State::Idle;
                        ParseResult::Error
                    }
                }
            }
            State::GotDir => {
                self.size = byte;
                self.crc = byte;
                self.payload_idx = 0;
                self.state = State::GotSize;
                ParseResult::Incomplete
            }
            State::GotSize => {
                self.cmd = byte;
                self.crc ^= byte;
                if self.size > 0 {
                    self.state = State::Payload;
                } else {
                    self.state = State::Checksum;
                }
                ParseResult::Incomplete
            }
            State::Payload => {
                let idx = self.payload_idx as usize;
                if idx < MAX_PAYLOAD {
                    self.payload[idx] = byte;
                }
                self.crc ^= byte;
                self.payload_idx += 1;
                if self.payload_idx >= self.size {
                    self.state = State::Checksum;
                }
                ParseResult::Incomplete
            }
            State::Checksum => {
                self.state = State::Idle;
                if self.crc == byte {
                    let mut payload = [0u8; MAX_PAYLOAD];
                    payload[..self.size as usize]
                        .copy_from_slice(&self.payload[..self.size as usize]);
                    ParseResult::Frame(MspFrame {
                        cmd: self.cmd,
                        size: self.size,
                        payload,
                    })
                } else {
                    ParseResult::Error
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// BOXNAMES decoder
// ---------------------------------------------------------------------------

/// Identified mode box types.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoxId {
    /// ARM mode box.
    Arm,
    /// FAILSAFE mode box.
    Failsafe,
    /// Any other box we don't care about.
    Unknown,
}

/// Parse the MSP_BOXNAMES payload into a fixed-size box map.
///
/// The payload is a semicolon-separated list of box names (e.g. `"ARM;ANGLE;FAILSAFE;..."`).
/// Returns an array mapping box index to [`BoxId`].
pub fn parse_boxnames(payload: &[u8], size: u8) -> [BoxId; MAX_BOXES] {
    let mut map = [BoxId::Unknown; MAX_BOXES];
    let data = &payload[..size as usize];
    let mut box_idx = 0;
    let mut start = 0;

    for i in 0..data.len() {
        if data[i] == b';' {
            if box_idx < MAX_BOXES {
                map[box_idx] = match &data[start..i] {
                    b"ARM" => BoxId::Arm,
                    b"FAILSAFE" => BoxId::Failsafe,
                    _ => BoxId::Unknown,
                };
                box_idx += 1;
            }
            start = i + 1;
        }
    }

    // Handle last entry (no trailing semicolon)
    if start < data.len() && box_idx < MAX_BOXES {
        map[box_idx] = match &data[start..] {
            b"ARM" => BoxId::Arm,
            b"FAILSAFE" => BoxId::Failsafe,
            _ => BoxId::Unknown,
        };
    }

    map
}

/// Parse the MSP_BOXIDS payload into a fixed-size box map.
///
/// Each byte in the payload is a permanent box ID. Betaflight defines
/// ARM = 0 and FAILSAFE = 27.
pub fn parse_boxids(payload: &[u8], size: u8) -> [BoxId; MAX_BOXES] {
    let mut map = [BoxId::Unknown; MAX_BOXES];
    let len = (size as usize).min(MAX_BOXES);
    for i in 0..len {
        map[i] = match payload[i] {
            PERM_ID_ARM => BoxId::Arm,
            PERM_ID_FAILSAFE => BoxId::Failsafe,
            _ => BoxId::Unknown,
        };
    }
    map
}

// ---------------------------------------------------------------------------
// Flight mode resolution
// ---------------------------------------------------------------------------

/// Extract the flight mode bitmask from an MSP_STATUS response payload.
///
/// Bytes 6–9 of the MSP_STATUS payload contain the `u32 LE` flight mode flags.
/// Returns `None` if the payload is too short.
pub fn extract_mode_flags(payload: &[u8], size: u8) -> Option<u32> {
    if size < 10 {
        return None;
    }
    Some(u32::from_le_bytes([
        payload[6],
        payload[7],
        payload[8],
        payload[9],
    ]))
}

/// Resolve a flight mode bitmask to a [`FlightMode`] using the box map.
///
/// Priority: Failsafe > Armed > ArmingAllowed (FC connected but disarmed).
pub fn resolve_flight_mode(flags: u32, box_map: &[BoxId; MAX_BOXES]) -> FlightMode {
    let mut armed = false;
    let mut failsafe = false;

    for (i, box_id) in box_map.iter().enumerate() {
        if i >= 32 {
            break;
        }
        let bit_set = flags & (1 << i) != 0;
        match box_id {
            BoxId::Arm if bit_set => armed = true,
            BoxId::Failsafe if bit_set => failsafe = true,
            _ => {}
        }
    }

    if failsafe {
        FlightMode::Failsafe
    } else if armed {
        FlightMode::Armed
    } else {
        FlightMode::ArmingAllowed
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_request_empty_payload() {
        let mut buf = [0u8; 16];
        let len = build_request(MSP_STATUS, &[], &mut buf);
        assert_eq!(len, 6);
        assert_eq!(&buf[..3], b"$M<");
        assert_eq!(buf[3], 0); // size
        assert_eq!(buf[4], MSP_STATUS); // cmd
        assert_eq!(buf[5], 0 ^ MSP_STATUS); // checksum = size ^ cmd
    }

    #[test]
    fn build_request_with_payload() {
        let mut buf = [0u8; 16];
        let payload = [0xAA, 0x55];
        let len = build_request(42, &payload, &mut buf);
        assert_eq!(len, 8);
        assert_eq!(buf[3], 2); // size
        assert_eq!(buf[4], 42); // cmd
        assert_eq!(buf[5], 0xAA);
        assert_eq!(buf[6], 0x55);
        // checksum = size ^ cmd ^ payload bytes
        assert_eq!(buf[7], 2 ^ 42 ^ 0xAA ^ 0x55);
    }

    #[test]
    fn parser_valid_empty_frame() {
        let mut parser = MspParser::new();
        // Build a response frame: $M> size=0 cmd=101 crc=101
        let frame = [b'$', b'M', b'>', 0, MSP_STATUS, 0 ^ MSP_STATUS];
        for &b in &frame[..5] {
            assert!(matches!(parser.feed(b), ParseResult::Incomplete));
        }
        match parser.feed(frame[5]) {
            ParseResult::Frame(f) => {
                assert_eq!(f.cmd, MSP_STATUS);
                assert_eq!(f.size, 0);
            }
            _ => panic!("expected Frame"),
        }
    }

    #[test]
    fn parser_valid_frame_with_payload() {
        let mut parser = MspParser::new();
        let payload = [0x01, 0x02, 0x03];
        let size = 3u8;
        let cmd = 42u8;
        let crc = size ^ cmd ^ 0x01 ^ 0x02 ^ 0x03;
        let frame = [b'$', b'M', b'>', size, cmd, 0x01, 0x02, 0x03, crc];
        for &b in &frame[..8] {
            assert!(matches!(parser.feed(b), ParseResult::Incomplete));
        }
        match parser.feed(frame[8]) {
            ParseResult::Frame(f) => {
                assert_eq!(f.cmd, cmd);
                assert_eq!(f.size, size);
                assert_eq!(&f.payload[..3], &payload);
            }
            _ => panic!("expected Frame"),
        }
    }

    #[test]
    fn parser_corrupt_checksum() {
        let mut parser = MspParser::new();
        let frame = [b'$', b'M', b'>', 0, MSP_STATUS, 0xFF]; // bad checksum
        for &b in &frame[..5] {
            assert!(matches!(parser.feed(b), ParseResult::Incomplete));
        }
        assert!(matches!(parser.feed(frame[5]), ParseResult::Error));
    }

    #[test]
    fn parser_error_frame() {
        let mut parser = MspParser::new();
        // $M! is an error frame
        assert!(matches!(parser.feed(b'$'), ParseResult::Incomplete));
        assert!(matches!(parser.feed(b'M'), ParseResult::Incomplete));
        assert!(matches!(parser.feed(b'!'), ParseResult::Error));
    }

    #[test]
    fn parser_partial_then_valid() {
        let mut parser = MspParser::new();
        // Feed garbage, then a valid frame
        parser.feed(b'$');
        parser.feed(b'X'); // bad → Error, reset
        // Now feed a valid frame
        let crc = 0 ^ MSP_STATUS;
        let frame = [b'$', b'M', b'>', 0, MSP_STATUS, crc];
        for &b in &frame[..5] {
            assert!(matches!(parser.feed(b), ParseResult::Incomplete));
        }
        assert!(matches!(parser.feed(frame[5]), ParseResult::Frame(_)));
    }

    #[test]
    fn parse_boxnames_basic() {
        let data = b"ARM;ANGLE;FAILSAFE;HORIZON";
        let map = parse_boxnames(data, data.len() as u8);
        assert_eq!(map[0], BoxId::Arm);
        assert_eq!(map[1], BoxId::Unknown); // ANGLE
        assert_eq!(map[2], BoxId::Failsafe);
        assert_eq!(map[3], BoxId::Unknown); // HORIZON
    }

    #[test]
    fn parse_boxnames_trailing_semicolon() {
        let data = b"ARM;FAILSAFE;";
        let map = parse_boxnames(data, data.len() as u8);
        assert_eq!(map[0], BoxId::Arm);
        assert_eq!(map[1], BoxId::Failsafe);
    }

    #[test]
    fn parse_boxnames_empty() {
        let data = b"";
        let map = parse_boxnames(data, 0);
        assert!(map.iter().all(|b| *b == BoxId::Unknown));
    }

    #[test]
    fn extract_mode_flags_valid() {
        // Build a fake MSP_STATUS payload (at least 10 bytes)
        let mut payload = [0u8; 16];
        // Bytes 6-9: flags = 0x0000_0005 (bits 0 and 2 set)
        payload[6] = 0x05;
        payload[7] = 0x00;
        payload[8] = 0x00;
        payload[9] = 0x00;
        assert_eq!(extract_mode_flags(&payload, 16), Some(5));
    }

    #[test]
    fn extract_mode_flags_too_short() {
        let payload = [0u8; 8];
        assert_eq!(extract_mode_flags(&payload, 8), None);
    }

    #[test]
    fn resolve_armed() {
        let mut box_map = [BoxId::Unknown; MAX_BOXES];
        box_map[0] = BoxId::Arm;
        box_map[2] = BoxId::Failsafe;
        // Only ARM bit set
        let mode = resolve_flight_mode(0b001, &box_map);
        assert_eq!(mode, FlightMode::Armed);
    }

    #[test]
    fn resolve_failsafe_overrides_armed() {
        let mut box_map = [BoxId::Unknown; MAX_BOXES];
        box_map[0] = BoxId::Arm;
        box_map[2] = BoxId::Failsafe;
        // Both ARM and FAILSAFE bits set
        let mode = resolve_flight_mode(0b101, &box_map);
        assert_eq!(mode, FlightMode::Failsafe);
    }

    #[test]
    fn resolve_disarmed() {
        let mut box_map = [BoxId::Unknown; MAX_BOXES];
        box_map[0] = BoxId::Arm;
        // No bits set
        let mode = resolve_flight_mode(0, &box_map);
        assert_eq!(mode, FlightMode::ArmingAllowed);
    }
}
