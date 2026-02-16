//! LED pattern trait and built-in pattern implementations.

use smart_leds::RGB8;
use smart_leds::hsv::{Hsv, hsv2rgb};

/// A pattern that renders frames into an LED buffer.
pub trait Pattern {
    /// Render the next frame into the provided LED buffer.
    fn render(&mut self, leds: &mut [RGB8]);
}

// ---------------------------------------------------------------------------
// Rainbow cycle
// ---------------------------------------------------------------------------

/// Cycles a full rainbow across the LED strip, shifting each frame.
pub struct RainbowCycle {
    hue: u8,
}

impl RainbowCycle {
    /// Create a new `RainbowCycle` starting at hue 0.
    pub fn new() -> Self {
        Self { hue: 0 }
    }
}

impl Default for RainbowCycle {
    fn default() -> Self {
        Self::new()
    }
}

impl Pattern for RainbowCycle {
    fn render(&mut self, leds: &mut [RGB8]) {
        let len = leds.len();
        for (i, led) in leds.iter_mut().enumerate() {
            *led = hsv2rgb(Hsv {
                hue: self.hue.wrapping_add((i * 256 / len) as u8),
                sat: 255,
                val: 255,
            });
        }
        self.hue = self.hue.wrapping_add(1);
    }
}

// ---------------------------------------------------------------------------
// Ripple effect
// ---------------------------------------------------------------------------

/// Maximum number of concurrently active ripples.
const MAX_RIPPLES: usize = 20;

/// Background color (black).
const BACKGROUND: RGB8 = RGB8 { r: 0, g: 0, b: 0 };

/// A single expanding ring ripple.
#[derive(Clone, Copy)]
struct Ripple {
    /// LED index where the ripple originated.
    origin: u8,
    /// Current ring radius in LEDs (expands each frame).
    radius: f32,
    /// Brightness multiplier, decays each frame.
    amplitude: f32,
    /// Pre-computed RGB color (random hue at spawn).
    color: RGB8,
}

/// Simple xorshift32 PRNG suitable for visual effects.
struct Xorshift32 {
    state: u32,
}

impl Xorshift32 {
    fn new(seed: u32) -> Self {
        // Avoid zero state which would produce all zeros.
        Self { state: if seed == 0 { 1 } else { seed } }
    }

    fn next(&mut self) -> u32 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.state = x;
        x
    }
}

/// Particle-based expanding rings on a ring topology.
///
/// Ripples spawn at random positions, expand outward, fade over time, and
/// composite additively onto a deep navy background. The effect resembles
/// colored stones dropping into dark water.
pub struct RippleEffect {
    ripples: [Ripple; MAX_RIPPLES],
    count: usize,
    rng: Xorshift32,
    /// Probability of spawning a ripple per frame (fixed-point: threshold out of u32::MAX).
    spawn_threshold: u32,
    /// How many LEDs the ring expands per frame.
    speed: f32,
    /// Half-width of the ring wavefront in LEDs.
    width: f32,
    /// Per-frame amplitude multiplier.
    decay: f32,
}

impl RippleEffect {
    /// Create a new `RippleEffect` with the given PRNG seed.
    pub fn new(seed: u32) -> Self {
        // 0.06 probability ≈ 0.06 * u32::MAX
        let spawn_threshold = (0.06_f64 * u32::MAX as f64) as u32;

        Self {
            ripples: [Ripple {
                origin: 0,
                radius: 0.0,
                amplitude: 0.0,
                color: BACKGROUND,
            }; MAX_RIPPLES],
            count: 0,
            rng: Xorshift32::new(seed),
            spawn_threshold,
            speed: 1.5,
            width: 19.0,
            decay: 0.97,
        }
    }

    /// Spawn a new ripple at a random position with a random hue.
    fn spawn(&mut self, num_leds: u8) {
        if self.count >= MAX_RIPPLES {
            return;
        }
        let origin = (self.rng.next() % num_leds as u32) as u8;
        let hue = self.rng.next() as u8;
        let color = hsv2rgb(Hsv { hue, sat: 230, val: 255 }); // S≈0.9, V=1.0
        self.ripples[self.count] = Ripple {
            origin,
            radius: 0.0,
            amplitude: 1.0,
            color,
        };
        self.count += 1;
    }
}

/// Shortest arc distance around a ring of `n` LEDs.
fn wrap_distance(a: u8, b: u8, n: u8) -> f32 {
    let d = a.abs_diff(b) as f32;
    let n_f = n as f32;
    if d < n_f - d { d } else { n_f - d }
}

/// Linearly interpolate a single channel.
fn lerp_u8(a: u8, b: u8, t: f32) -> u8 {
    (a as f32 + (b as f32 - a as f32) * t) as u8
}

/// Linearly interpolate two RGB colors.
fn lerp_rgb(bg: RGB8, fg: RGB8, t: f32) -> RGB8 {
    let t = t.clamp(0.0, 1.0);
    RGB8 {
        r: lerp_u8(bg.r, fg.r, t),
        g: lerp_u8(bg.g, fg.g, t),
        b: lerp_u8(bg.b, fg.b, t),
    }
}

impl Pattern for RippleEffect {
    fn render(&mut self, leds: &mut [RGB8]) {
        let num_leds = leds.len() as u8;

        // Maybe spawn a new ripple
        if self.rng.next() < self.spawn_threshold {
            self.spawn(num_leds);
        }

        // Update existing ripples
        for i in 0..self.count {
            self.ripples[i].radius += self.speed;
            self.ripples[i].amplitude *= self.decay;
        }

        // Remove dead ripples (amplitude < 0.02)
        let mut i = 0;
        while i < self.count {
            if self.ripples[i].amplitude < 0.02 {
                self.ripples[i] = self.ripples[self.count - 1];
                self.count -= 1;
            } else {
                i += 1;
            }
        }

        // Fill background
        for led in leds.iter_mut() {
            *led = BACKGROUND;
        }

        // Composite each ripple onto the buffer
        for (led_idx, led) in leds.iter_mut().enumerate() {
            for ri in 0..self.count {
                let ripple = &self.ripples[ri];
                let d = wrap_distance(led_idx as u8, ripple.origin, num_leds);
                let diff = (d - ripple.radius).abs();
                if diff < self.width {
                    let falloff = 1.0 - diff / self.width;
                    let bright = ripple.amplitude * falloff * falloff;
                    *led = lerp_rgb(*led, ripple.color, bright);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Sinusoidal pulse
// ---------------------------------------------------------------------------

/// Smooth sinusoidal breathing/pulsing of the entire strip in a single color.
///
/// Intensity follows a sine wave from 0 to full value and back.
/// Useful for status indicators (e.g. "arming allowed" in green).
pub struct SinePulse {
    /// Base color at full intensity (hue/saturation preserved, value modulated).
    color: RGB8,
    /// Phase accumulator in fixed-point (0–65535 maps to 0–2π).
    phase: u16,
    /// Phase increment per frame (controls speed).
    speed: u16,
    /// Minimum intensity floor (0.0–1.0). Pulse oscillates between this and 1.0.
    min_intensity: f32,
}

impl SinePulse {
    /// Create a new `SinePulse` with the given color and speed.
    ///
    /// `speed` is the phase increment per frame in 16-bit fixed-point.
    /// At 100 FPS, a speed of 400 gives a ~1.6 s full cycle.
    pub fn new(color: RGB8, speed: u16, min_intensity: f32) -> Self {
        Self {
            color,
            phase: 0,
            speed,
            min_intensity: min_intensity.clamp(0.0, 1.0),
        }
    }

    /// Create a green sinusoidal pulse with default speed and 10% floor.
    pub fn green() -> Self {
        Self::new(RGB8 { r: 0, g: 204, b: 0 }, 600, 0.1)
    }
}

impl Pattern for SinePulse {
    fn render(&mut self, leds: &mut [RGB8]) {
        // Advance phase (wraps naturally at u16::MAX)
        self.phase = self.phase.wrapping_add(self.speed);

        // Approximate sin using a parabolic curve on 0–65535:
        //   triangle: fold phase into 0–32767–0 range
        //   then square for smooth sine-like shape
        let half = if self.phase < 32768 {
            self.phase
        } else {
            65535 - self.phase
        };
        // Normalize to 0.0–1.0 and apply squared curve for smoothness
        let t = half as f32 / 32767.0;
        let intensity = self.min_intensity + (1.0 - self.min_intensity) * t * t;

        let r = (self.color.r as f32 * intensity) as u8;
        let g = (self.color.g as f32 * intensity) as u8;
        let b = (self.color.b as f32 * intensity) as u8;
        let c = RGB8 { r, g, b };

        for led in leds.iter_mut() {
            *led = c;
        }
    }
}

// ---------------------------------------------------------------------------
// Split pulse (two halves, two colors)
// ---------------------------------------------------------------------------

/// Sinusoidal pulse split into two halves, each with its own color.
///
/// The first half of the strip pulses in one color, the second half in another.
/// Both halves share the same phase and speed.
pub struct SplitPulse {
    /// Color for the first half of the strip.
    color_a: RGB8,
    /// Color for the second half of the strip.
    color_b: RGB8,
    /// Phase accumulator in fixed-point (0–65535 maps to 0–2π).
    phase: u16,
    /// Phase increment per frame.
    speed: u16,
    /// Minimum intensity floor (0.0–1.0).
    min_intensity: f32,
}

impl SplitPulse {
    /// Create a new `SplitPulse` with the given colors, speed, and floor.
    pub fn new(color_a: RGB8, color_b: RGB8, speed: u16, min_intensity: f32) -> Self {
        Self {
            color_a,
            color_b,
            phase: 0,
            speed,
            min_intensity: min_intensity.clamp(0.0, 1.0),
        }
    }

    /// Green front half, red rear half, default speed and 10% floor.
    pub fn green_red() -> Self {
        Self::new(
            RGB8 { r: 0, g: 204, b: 0 },
            RGB8 { r: 204, g: 0, b: 0 },
            600,
            0.1,
        )
    }
}

impl Pattern for SplitPulse {
    fn render(&mut self, leds: &mut [RGB8]) {
        self.phase = self.phase.wrapping_add(self.speed);

        let half = if self.phase < 32768 {
            self.phase
        } else {
            65535 - self.phase
        };
        let t = half as f32 / 32767.0;
        let intensity = self.min_intensity + (1.0 - self.min_intensity) * t * t;

        let mid = leds.len() / 2;

        let apply = |color: RGB8| -> RGB8 {
            RGB8 {
                r: (color.r as f32 * intensity) as u8,
                g: (color.g as f32 * intensity) as u8,
                b: (color.b as f32 * intensity) as u8,
            }
        };

        let ca = apply(self.color_a);
        let cb = apply(self.color_b);

        for led in &mut leds[..mid] {
            *led = ca;
        }
        for led in &mut leds[mid..] {
            *led = cb;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rainbow_cycle_advances_hue() {
        let mut pattern = RainbowCycle::new();
        let mut buf = [RGB8 { r: 0, g: 0, b: 0 }; 4];

        pattern.render(&mut buf);
        let first = buf[0];

        pattern.render(&mut buf);
        // Hue shifted by 1, so color should differ
        assert_ne!(buf[0], first);
    }

    #[test]
    fn wrap_distance_symmetric() {
        assert_eq!(wrap_distance(0, 10, 180), 10.0);
        assert_eq!(wrap_distance(10, 0, 180), 10.0);
    }

    #[test]
    fn wrap_distance_wraps_around() {
        // Distance from 0 to 170 on a 180-LED ring should be 10, not 170
        assert_eq!(wrap_distance(0, 170, 180), 10.0);
        assert_eq!(wrap_distance(170, 0, 180), 10.0);
    }

    #[test]
    fn ripple_starts_with_background() {
        let mut pattern = RippleEffect::new(42);
        let mut buf = [RGB8 { r: 0, g: 0, b: 0 }; 10];

        // With spawn probability ~2%, first frame may or may not spawn.
        // But the background should be set.
        pattern.render(&mut buf);
        // At minimum, LEDs without a ripple should be the background color.
        // We verify at least one LED is the background.
        assert!(buf.iter().any(|c| *c == BACKGROUND));
    }

    #[test]
    fn ripple_dead_ripples_removed() {
        let mut pattern = RippleEffect::new(42);
        let mut buf = [RGB8 { r: 0, g: 0, b: 0 }; 20];

        // Force-spawn a ripple with very low amplitude
        pattern.ripples[0] = Ripple {
            origin: 5,
            radius: 10.0,
            amplitude: 0.01,
            color: RGB8 { r: 255, g: 0, b: 0 },
        };
        pattern.count = 1;

        pattern.render(&mut buf);
        // After render, the dead ripple should have been removed
        assert_eq!(pattern.count, 0);
    }

    #[test]
    fn lerp_rgb_endpoints() {
        let black = RGB8 { r: 0, g: 0, b: 0 };
        let white = RGB8 { r: 255, g: 255, b: 255 };

        let result = lerp_rgb(black, white, 0.0);
        assert_eq!(result, black);

        let result = lerp_rgb(black, white, 1.0);
        assert_eq!(result, white);
    }

    #[test]
    fn xorshift_never_zero_with_nonzero_seed() {
        let mut rng = Xorshift32::new(1);
        for _ in 0..1000 {
            assert_ne!(rng.next(), 0);
        }
    }
}
