//! Post-processing effects for the LED buffer.
//!
//! Effects are represented as enum variants and applied in sequence to the
//! pixel buffer via [`apply_pipeline`]. Everything is zero-allocation, fully
//! static, and `Copy`.

use smart_leds::RGB8;

/// Gamma 2.6 correction look-up table (256 entries, lives in flash).
///
/// Source: Adafruit gamma8 table.
const GAMMA_LUT: [u8; 256] = [
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
      1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
      2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
      5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
     10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
     17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
     25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
     37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
     51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
     69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
     90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
    115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
    144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
    177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
    215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255,
];

/// A single post-processing effect applied to the LED buffer.
#[derive(Clone, Copy)]
pub enum PostEffect {
    /// Scale every channel by `brightness / 256`.
    Brightness(u8),
    /// Clamp total strip current to `max_ma` milliamps.
    CurrentLimit { max_ma: u32 },
    /// Apply gamma 2.2 correction via look-up table.
    Gamma,
}

impl PostEffect {
    /// Apply this effect to the LED buffer in-place.
    pub fn apply(&self, leds: &mut [RGB8]) {
        match self {
            PostEffect::Brightness(n) => {
                let n = *n as u16;
                for px in leds.iter_mut() {
                    px.r = (px.r as u16 * n / 256) as u8;
                    px.g = (px.g as u16 * n / 256) as u8;
                    px.b = (px.b as u16 * n / 256) as u8;
                }
            }
            PostEffect::CurrentLimit { max_ma } => {
                // Model: each WS2812B draws ~1 mA idle + 8 mA per colour
                // channel at full PWM.
                let channel_sum: u32 = leds
                    .iter()
                    .map(|c| c.r as u32 + c.g as u32 + c.b as u32)
                    .sum();

                let num = leds.len() as u32;
                // total_ma = idle + channel_draw
                //   idle         = num_leds * 1 mA
                //   channel_draw = channel_sum * 8 / 255 mA
                let total_ma_x255 = num * 255 + channel_sum * 8;

                let budget_x255 = *max_ma * 255;
                if total_ma_x255 > budget_x255 {
                    // Scale factor = budget / total  (both ×255, cancels)
                    for px in leds.iter_mut() {
                        px.r = (px.r as u32 * budget_x255 / total_ma_x255) as u8;
                        px.g = (px.g as u32 * budget_x255 / total_ma_x255) as u8;
                        px.b = (px.b as u32 * budget_x255 / total_ma_x255) as u8;
                    }
                }
            }
            PostEffect::Gamma => {
                for px in leds.iter_mut() {
                    px.r = GAMMA_LUT[px.r as usize];
                    px.g = GAMMA_LUT[px.g as usize];
                    px.b = GAMMA_LUT[px.b as usize];
                }
            }
        }
    }
}

/// Apply a sequence of post-processing effects to the LED buffer.
pub fn apply_pipeline(leds: &mut [RGB8], effects: &[PostEffect]) {
    for effect in effects {
        effect.apply(leds);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn brightness_scales_channels() {
        let mut leds = [RGB8 { r: 200, g: 100, b: 50 }];
        PostEffect::Brightness(128).apply(&mut leds);
        assert_eq!(leds[0].r, 100);
        assert_eq!(leds[0].g, 50);
        assert_eq!(leds[0].b, 25);
    }

    #[test]
    fn brightness_zero_blacks_out() {
        let mut leds = [RGB8 { r: 255, g: 255, b: 255 }];
        PostEffect::Brightness(0).apply(&mut leds);
        assert_eq!(leds[0], RGB8 { r: 0, g: 0, b: 0 });
    }

    #[test]
    fn gamma_maps_via_lut() {
        let mut leds = [RGB8 { r: 0, g: 128, b: 255 }];
        PostEffect::Gamma.apply(&mut leds);
        assert_eq!(leds[0].r, GAMMA_LUT[0]);
        assert_eq!(leds[0].g, GAMMA_LUT[128]);
        assert_eq!(leds[0].b, GAMMA_LUT[255]);
    }

    #[test]
    fn current_limit_under_budget_is_noop() {
        let mut leds = [RGB8 { r: 10, g: 10, b: 10 }; 2];
        let before = leds;
        PostEffect::CurrentLimit { max_ma: 5000 }.apply(&mut leds);
        assert_eq!(leds, before);
    }

    #[test]
    fn current_limit_over_budget_reduces() {
        let mut leds = [RGB8 { r: 255, g: 255, b: 255 }; 100];
        PostEffect::CurrentLimit { max_ma: 500 }.apply(&mut leds);
        // All pixels should be reduced
        assert!(leds[0].r < 255);
    }

    #[test]
    fn pipeline_applies_in_order() {
        let mut leds = [RGB8 { r: 200, g: 100, b: 50 }];
        apply_pipeline(
            &mut leds,
            &[PostEffect::Brightness(128), PostEffect::Gamma],
        );
        // Brightness first: 200*128/256=100, then gamma LUT[100]
        assert_eq!(leds[0].r, GAMMA_LUT[100]);
    }
}
