//! Temporal dithering for WS2812B LEDs.
//!
//! WS2812B LEDs have 8-bit per-channel resolution. After gamma correction,
//! low-brightness values get crushed (e.g., input 10 maps to output 0–1),
//! creating visible banding. By refreshing the strip faster than perceptual
//! flicker fusion (~60 Hz) and varying the quantized output frame-to-frame,
//! we represent fractional brightness through time-averaging — effectively
//! adding extra bit depth.
//!
//! All types are fixed-size, no-alloc, suitable for `no_std` embedded use.

use smart_leds::RGB8;

/// Maximum number of LEDs supported (must match `MAX_LEDS` in main).
const MAX_LEDS: usize = 200;

/// 8.8 fixed-point: high byte = integer (0–255), low byte = fraction (0–255).
pub type Fix16 = u16;

// ---------------------------------------------------------------------------
// Compile-time gamma LUTs (8.8 fixed-point output)
// ---------------------------------------------------------------------------

/// Compute gamma correction for a single input value, returning 8.8 fixed-point.
///
/// Uses an integer approximation: `(input/255)^gamma * 255` scaled to 16-bit.
/// The `gamma_x10` parameter is gamma × 10 (e.g., 26 for gamma 2.6).
const fn gamma_fix16(input: u8, gamma_x10: u32) -> Fix16 {
    if input == 0 {
        return 0;
    }
    if input == 255 {
        return 255 << 8;
    }

    // We compute (input/255)^gamma * 255 * 256 using integer math.
    // Strategy: use repeated squaring on a fixed-point representation.
    // Work in 32-bit with 16 fractional bits for intermediate precision.

    // input_norm = input / 255 in 0.16 fixed-point
    let input_fp: u64 = (input as u64) << 16;
    let norm: u64 = input_fp / 255; // 0.16 fixed-point, range [0, 65536]

    // Compute norm^(gamma_x10/10) using logarithms approximated by iteration.
    // For better precision, we use pow by repeated multiplication.
    // gamma_x10/10 = integer_part + fractional_part
    let int_part = gamma_x10 / 10;
    let frac_part = gamma_x10 % 10; // tenths

    // Compute norm^int_part (0.16 fixed-point)
    let mut result: u64 = 1 << 16; // 1.0 in 0.16
    let mut i = 0;
    while i < int_part {
        result = (result * norm) >> 16;
        i += 1;
    }

    // For the fractional part, approximate norm^0.X by linear interpolation
    // between norm^0 (=1) and norm^1 (=norm): norm^frac ≈ 1 + frac*(norm-1)
    // This isn't perfectly accurate but is good enough for gamma LUTs.
    if frac_part > 0 {
        // Compute one more full power for interpolation
        let next_power = (result * norm) >> 16;
        // Interpolate: result + frac/10 * (next_power - result)
        // = result * (10 - frac)/10 + next_power * frac/10
        result = (result * (10 - frac_part as u64) + next_power * frac_part as u64) / 10;
    }

    // Scale from 0.16 to the output: result * 255 * 256 / 65536
    // = result * 255 * 256 >> 16
    // = result * 255 >> 8  (since 256 >> 16 = >> 8)
    let out = (result * 255) >> 8;

    // Clamp to valid Fix16 range
    if out > (255 << 8) {
        255 << 8
    } else {
        out as Fix16
    }
}

/// Build a 256-entry gamma LUT at compile time.
const fn build_gamma_lut(gamma_x10: u32) -> [Fix16; 256] {
    let mut lut = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        lut[i] = gamma_fix16(i as u8, gamma_x10);
        i += 1;
    }
    lut
}

/// Red channel gamma 2.6 correction LUT (8.8 fixed-point output).
const GAMMA_R_FIX16: [Fix16; 256] = build_gamma_lut(26);

/// Green channel gamma 2.7 correction LUT (8.8 fixed-point output).
const GAMMA_G_FIX16: [Fix16; 256] = build_gamma_lut(27);

/// Blue channel gamma 2.5 correction LUT (8.8 fixed-point output).
const GAMMA_B_FIX16: [Fix16; 256] = build_gamma_lut(25);

// ---------------------------------------------------------------------------
// Dither mode
// ---------------------------------------------------------------------------

/// Dithering algorithm selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum DitherMode {
    /// No dithering (bypass, use u8 gamma as before).
    Off,
    /// Temporal error diffusion (smooth gradients, best for slow fades).
    ErrorDiffusion,
    /// Ordered Bayer 4x4 (deterministic, good for strobes/fast changes).
    Ordered,
    /// Hybrid: error diffusion normally, correlated ordered when any channel < 10.
    Hybrid,
}

// ---------------------------------------------------------------------------
// Bayer 4x4 threshold matrix
// ---------------------------------------------------------------------------

/// 4x4 Bayer ordered dither threshold matrix, scaled to 8.8 fixed-point.
///
/// Standard Bayer matrix values (0-15) mapped to range [-128, +112] in
/// 8.8 fixed-point (i.e., -0.5 to +0.44 in the integer domain).
const BAYER_4X4: [i16; 16] = {
    // Standard Bayer 4x4 normalized positions: 0/16, 8/16, 2/16, 10/16, ...
    // Map to [-128, +112] (8.8 fixed-point, covering roughly ±0.5)
    let matrix: [u8; 16] = [
         0,  8,  2, 10,
        12,  4, 14,  6,
         3, 11,  1,  9,
        15,  7, 13,  5,
    ];
    let mut result = [0i16; 16];
    let mut i = 0;
    while i < 16 {
        // Map 0..15 to -128..+112 (step of 16 in 8.8 = step of 1/16 in integer)
        result[i] = (matrix[i] as i16) * 16 - 128;
        i += 1;
    }
    result
};

// ---------------------------------------------------------------------------
// Dither state
// ---------------------------------------------------------------------------

/// Per-LED, per-channel error accumulators and frame counter for temporal dithering.
pub struct DitherState {
    /// Per-LED per-channel error accumulator (signed 8.8 fixed-point).
    error: [[i16; 3]; MAX_LEDS],
    /// Frame counter for ordered dithering.
    frame: u32,
}

impl Default for DitherState {
    fn default() -> Self {
        Self::new()
    }
}

impl DitherState {
    /// Create a new dither state with zeroed accumulators.
    pub const fn new() -> Self {
        Self {
            error: [[0i16; 3]; MAX_LEDS],
            frame: 0,
        }
    }

    /// Reset all error accumulators (call on scene/mode change).
    pub fn reset(&mut self) {
        for e in self.error.iter_mut() {
            *e = [0; 3];
        }
    }

    /// Apply one dither frame, converting Fix16 targets to quantized RGB8 output.
    ///
    /// `targets` contains 8.8 fixed-point gamma-corrected values per LED per channel.
    /// `output` receives the quantized u8 values for this frame.
    pub fn dither_frame(
        &mut self,
        mode: DitherMode,
        targets: &[[Fix16; 3]],
        output: &mut [RGB8],
    ) {
        let len = targets.len().min(output.len()).min(MAX_LEDS);

        match mode {
            DitherMode::Off => {
                // Simple rounding, no dithering
                for i in 0..len {
                    output[i] = RGB8 {
                        r: ((targets[i][0] + 128) >> 8).min(255) as u8,
                        g: ((targets[i][1] + 128) >> 8).min(255) as u8,
                        b: ((targets[i][2] + 128) >> 8).min(255) as u8,
                    };
                }
            }
            DitherMode::ErrorDiffusion => {
                self.dither_error_diffusion(targets, output, len);
            }
            DitherMode::Ordered => {
                self.dither_ordered(targets, output, len, false);
            }
            DitherMode::Hybrid => {
                self.dither_hybrid(targets, output, len);
            }
        }

        self.frame = self.frame.wrapping_add(1);
    }

    /// Error diffusion dithering: accumulates quantization error per LED per channel.
    fn dither_error_diffusion(
        &mut self,
        targets: &[[Fix16; 3]],
        output: &mut [RGB8],
        len: usize,
    ) {
        for i in 0..len {
            let mut rgb = [0u8; 3];
            for ch in 0..3 {
                let target = targets[i][ch] as i16;
                let corrected = target.saturating_add(self.error[i][ch]);
                // Round to nearest u8 (add 0.5 in 8.8 = 128, then shift)
                let quantized = ((corrected.max(0) + 128) >> 8).min(255) as u8;
                // New error = corrected - quantized (in 8.8)
                let new_error = corrected - ((quantized as i16) << 8);
                // Clamp error to prevent runaway accumulation
                self.error[i][ch] = new_error.clamp(-256, 256);
                rgb[ch] = quantized;
            }
            output[i] = RGB8 { r: rgb[0], g: rgb[1], b: rgb[2] };
        }
    }

    /// Ordered Bayer 4x4 dithering using spatial LED index + temporal frame index.
    ///
    /// When `correlated` is true, the same threshold is used for all 3 channels
    /// (preserves hue at very low brightness). Otherwise each channel uses a
    /// different column offset for less visible patterning.
    fn dither_ordered(
        &mut self,
        targets: &[[Fix16; 3]],
        output: &mut [RGB8],
        len: usize,
        correlated: bool,
    ) {
        let frame_row = (self.frame as usize) & 3; // row = frame mod 4
        for i in 0..len {
            let col = i & 3; // column = LED index mod 4
            let idx_base = frame_row * 4 + col;
            let threshold = BAYER_4X4[idx_base & 15];

            let mut rgb = [0u8; 3];
            for ch in 0..3 {
                let t = if correlated {
                    threshold
                } else {
                    // Offset each channel by a different amount to decorrelate
                    BAYER_4X4[(idx_base + ch * 5) & 15]
                };
                let target = targets[i][ch] as i16;
                let dithered = target + t;
                rgb[ch] = ((dithered.max(0)) >> 8).min(255) as u8;
            }
            output[i] = RGB8 { r: rgb[0], g: rgb[1], b: rgb[2] };
        }
    }

    /// Hybrid dithering: error diffusion for normal brightness, correlated ordered
    /// when any channel target is below threshold (integer part < 10).
    fn dither_hybrid(
        &mut self,
        targets: &[[Fix16; 3]],
        output: &mut [RGB8],
        len: usize,
    ) {
        let low_threshold: u16 = 10 << 8;
        let frame_row = (self.frame as usize) & 3;

        for i in 0..len {
            let any_low = targets[i][0] < low_threshold
                || targets[i][1] < low_threshold
                || targets[i][2] < low_threshold;

            if any_low {
                // Use correlated ordered dithering for low-brightness LEDs
                let col = i & 3;
                let idx_base = frame_row * 4 + col;
                let threshold = BAYER_4X4[idx_base & 15];

                let mut rgb = [0u8; 3];
                for ch in 0..3 {
                    let target = targets[i][ch] as i16;
                    let dithered = target + threshold;
                    rgb[ch] = ((dithered.max(0)) >> 8).min(255) as u8;
                }
                // Reset error accumulator since we're not using error diffusion
                self.error[i] = [0; 3];
                output[i] = RGB8 { r: rgb[0], g: rgb[1], b: rgb[2] };
            } else {
                // Use error diffusion for normal brightness
                let mut rgb = [0u8; 3];
                for ch in 0..3 {
                    let target = targets[i][ch] as i16;
                    let corrected = target.saturating_add(self.error[i][ch]);
                    let quantized = ((corrected.max(0) + 128) >> 8).min(255) as u8;
                    let new_error = corrected - ((quantized as i16) << 8);
                    self.error[i][ch] = new_error.clamp(-256, 256);
                    rgb[ch] = quantized;
                }
                output[i] = RGB8 { r: rgb[0], g: rgb[1], b: rgb[2] };
            }
        }

        self.frame = self.frame.wrapping_add(1);
    }
}

// ---------------------------------------------------------------------------
// Gamma conversion (RGB8 → Fix16 targets)
// ---------------------------------------------------------------------------

/// Apply gamma correction to an RGB8 buffer, producing 8.8 fixed-point targets.
///
/// This runs once per animation frame. The dither loop then repeatedly
/// quantizes these targets to produce slightly different u8 outputs each sub-frame.
pub fn gamma_to_fix16(src: &[RGB8], dst: &mut [[Fix16; 3]]) {
    let len = src.len().min(dst.len());
    for i in 0..len {
        dst[i] = [
            GAMMA_R_FIX16[src[i].r as usize],
            GAMMA_G_FIX16[src[i].g as usize],
            GAMMA_B_FIX16[src[i].b as usize],
        ];
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fix16_gamma_endpoints() {
        // Input 0 → output 0 for all channels
        assert_eq!(GAMMA_R_FIX16[0], 0);
        assert_eq!(GAMMA_G_FIX16[0], 0);
        assert_eq!(GAMMA_B_FIX16[0], 0);
        // Input 255 → output 255.0 (= 255 << 8 = 65280)
        assert_eq!(GAMMA_R_FIX16[255], 255 << 8);
        assert_eq!(GAMMA_G_FIX16[255], 255 << 8);
        assert_eq!(GAMMA_B_FIX16[255], 255 << 8);
    }

    #[test]
    fn fix16_gamma_monotonic() {
        // All LUTs should be monotonically non-decreasing
        for lut in [&GAMMA_R_FIX16, &GAMMA_G_FIX16, &GAMMA_B_FIX16] {
            for i in 1..256 {
                assert!(
                    lut[i] >= lut[i - 1],
                    "LUT not monotonic at index {}: {} < {}",
                    i,
                    lut[i],
                    lut[i - 1]
                );
            }
        }
    }

    #[test]
    fn fix16_gamma_green_steeper_than_red() {
        // Green gamma (2.7) is steeper than red (2.6), so mid-range green < red
        assert!(GAMMA_G_FIX16[128] < GAMMA_R_FIX16[128]);
    }

    #[test]
    fn error_diffusion_converges() {
        // A constant target of 0.5 (Fix16 = 128) should produce alternating 0/1
        // and the error should stay bounded.
        let mut state = DitherState::new();
        let targets = [[128u16; 3]; 1]; // 0.5 in 8.8 for all channels
        let mut output = [RGB8 { r: 0, g: 0, b: 0 }; 1];

        let mut sum = [0u32; 3];
        let n = 100;
        for _ in 0..n {
            state.dither_frame(DitherMode::ErrorDiffusion, &targets, &mut output);
            sum[0] += output[0].r as u32;
            sum[1] += output[0].g as u32;
            sum[2] += output[0].b as u32;
        }

        // Average should be close to 0.5 (either 0 or 1 each frame)
        // With error diffusion, target 0.5 should produce ~50% ones
        for ch in 0..3 {
            let avg_x100 = sum[ch] * 100 / n;
            assert!(
                avg_x100 >= 30 && avg_x100 <= 70,
                "channel {} average {}/100 outside expected range",
                ch,
                avg_x100,
            );
        }

        // Error should be bounded
        for ch in 0..3 {
            assert!(
                state.error[0][ch].unsigned_abs() <= 256,
                "error[0][{}] = {} exceeds bounds",
                ch,
                state.error[0][ch],
            );
        }
    }

    #[test]
    fn ordered_dither_deterministic() {
        // Same inputs + same frame counter → same outputs
        let targets = [[512u16; 3]; 4]; // 2.0 in 8.8
        let mut output_a = [RGB8 { r: 0, g: 0, b: 0 }; 4];
        let mut output_b = [RGB8 { r: 0, g: 0, b: 0 }; 4];

        let mut state_a = DitherState::new();
        let mut state_b = DitherState::new();

        state_a.dither_frame(DitherMode::Ordered, &targets, &mut output_a);
        state_b.dither_frame(DitherMode::Ordered, &targets, &mut output_b);

        for i in 0..4 {
            assert_eq!(output_a[i], output_b[i], "mismatch at LED {}", i);
        }
    }

    #[test]
    fn gamma_to_fix16_roundtrip() {
        // Full white should map to [255<<8, 255<<8, 255<<8]
        let src = [RGB8 { r: 255, g: 255, b: 255 }];
        let mut dst = [[0u16; 3]; 1];
        gamma_to_fix16(&src, &mut dst);
        assert_eq!(dst[0], [255 << 8, 255 << 8, 255 << 8]);

        // Black should map to [0, 0, 0]
        let src = [RGB8 { r: 0, g: 0, b: 0 }];
        gamma_to_fix16(&src, &mut dst);
        assert_eq!(dst[0], [0, 0, 0]);
    }

    #[test]
    fn dither_off_rounds_correctly() {
        let mut state = DitherState::new();
        // Target: 2.6 in 8.8 = (2 << 8) + 153 = 665
        let targets = [[665u16; 3]; 1];
        let mut output = [RGB8 { r: 0, g: 0, b: 0 }; 1];
        state.dither_frame(DitherMode::Off, &targets, &mut output);
        // (665 + 128) >> 8 = 793 >> 8 = 3 (rounds 2.6 to 3)
        assert_eq!(output[0].r, 3);
    }

    #[test]
    fn hybrid_uses_ordered_for_low_brightness() {
        // Targets below threshold (< 10 << 8 = 2560) should use ordered dithering
        let mut state = DitherState::new();
        let targets = [[256u16; 3]; 4]; // 1.0 in 8.8 — below threshold
        let mut output = [RGB8 { r: 0, g: 0, b: 0 }; 4];
        state.dither_frame(DitherMode::Hybrid, &targets, &mut output);
        // Should produce valid output without panicking
        for px in &output[..4] {
            assert!(px.r <= 2 && px.g <= 2 && px.b <= 2);
        }
    }

    #[test]
    fn reset_clears_error() {
        let mut state = DitherState::new();
        state.error[0] = [100, -200, 50];
        state.reset();
        assert_eq!(state.error[0], [0, 0, 0]);
    }
}
