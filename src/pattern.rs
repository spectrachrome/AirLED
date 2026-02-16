//! Color schemes, animation trait, and built-in animation implementations.
//!
//! The LED system is split into two independent axes:
//! - [`ColorScheme`] determines the base color at each LED position.
//! - [`Animation`] determines how those colors are modulated over time.

use smart_leds::RGB8;
use smart_leds::hsv::{Hsv, hsv2rgb};

// ---------------------------------------------------------------------------
// Color schemes
// ---------------------------------------------------------------------------

/// Base color scheme that maps LED positions to colors.
pub enum ColorScheme {
    /// Single solid color for all LEDs.
    Solid(RGB8),
    /// First half one color, second half another (port / starboard).
    Split(RGB8, RGB8),
    /// HSV rainbow gradient that shifts over time.
    Rainbow {
        /// Current base hue (advances each tick).
        hue: u8,
        /// Hue increment per tick.
        speed: u8,
    },
}

impl ColorScheme {
    /// Return the base color at the given LED index.
    pub fn color_at(&self, index: usize, num_leds: usize) -> RGB8 {
        match self {
            ColorScheme::Solid(c) => *c,
            ColorScheme::Split(a, b) => {
                if index < num_leds / 2 { *a } else { *b }
            }
            ColorScheme::Rainbow { hue, .. } => {
                let offset = if num_leds == 0 {
                    0
                } else {
                    (index * 256 / num_leds) as u8
                };
                hsv2rgb(Hsv {
                    hue: hue.wrapping_add(offset),
                    sat: 255,
                    val: 255,
                })
            }
        }
    }

    /// Advance time-dependent state (e.g. rainbow hue rotation).
    pub fn tick(&mut self) {
        if let ColorScheme::Rainbow { hue, speed } = self {
            *hue = hue.wrapping_add(*speed);
        }
    }

    /// Update the hue rotation speed (only affects `Rainbow`).
    pub fn set_hue_speed(&mut self, new_speed: u8) {
        if let ColorScheme::Rainbow { speed, .. } = self {
            *speed = new_speed;
        }
    }
}

// ---------------------------------------------------------------------------
// Animation trait
// ---------------------------------------------------------------------------

/// An animation that renders frames into an LED buffer using a color scheme.
pub trait Animation {
    /// Render the next frame into the provided LED buffer.
    fn render(&mut self, leds: &mut [RGB8], colors: &mut ColorScheme);
}

// ---------------------------------------------------------------------------
// Static animation (no motion)
// ---------------------------------------------------------------------------

/// Fills LEDs with colors from the scheme, advancing the scheme each frame.
pub struct StaticAnim;

impl Animation for StaticAnim {
    fn render(&mut self, leds: &mut [RGB8], colors: &mut ColorScheme) {
        let num = leds.len();
        for (i, led) in leds.iter_mut().enumerate() {
            *led = colors.color_at(i, num);
        }
        colors.tick();
    }
}

// ---------------------------------------------------------------------------
// Pulse animation (sinusoidal breathing)
// ---------------------------------------------------------------------------

/// Smooth sinusoidal breathing/pulsing of the entire strip.
///
/// Intensity follows a sine-like curve. Works with any color scheme.
pub struct Pulse {
    /// Phase accumulator in fixed-point (0–65535 maps to 0–2π).
    phase: u16,
    /// Phase increment per frame (controls speed).
    speed: u16,
    /// Minimum intensity floor (0.0–1.0). Pulse oscillates between this and 1.0.
    min_intensity: f32,
}

impl Default for Pulse {
    fn default() -> Self {
        Self::new()
    }
}

impl Pulse {
    /// Create a new `Pulse` with default parameters.
    pub fn new() -> Self {
        Self {
            phase: 0,
            speed: 600,
            min_intensity: 0.4,
        }
    }

    /// Update pulse speed and minimum intensity floor.
    pub fn set_params(&mut self, speed: u16, min_intensity: f32) {
        self.speed = speed;
        self.min_intensity = min_intensity.clamp(0.0, 1.0);
    }
}

impl Animation for Pulse {
    fn render(&mut self, leds: &mut [RGB8], colors: &mut ColorScheme) {
        self.phase = self.phase.wrapping_add(self.speed);

        let half = if self.phase < 32768 {
            self.phase
        } else {
            65535 - self.phase
        };
        // Cubic falloff: spends most time dim, snaps quickly to bright
        let t = half as f32 / 32767.0;
        let intensity = self.min_intensity + (1.0 - self.min_intensity) * t * t * t;

        let num = leds.len();
        for (i, led) in leds.iter_mut().enumerate() {
            let c = colors.color_at(i, num);
            *led = RGB8 {
                r: (c.r as f32 * intensity) as u8,
                g: (c.g as f32 * intensity) as u8,
                b: (c.b as f32 * intensity) as u8,
            };
        }
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
    /// Pre-computed RGB color (inherited from color scheme at spawn).
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
/// Ripples spawn at random positions, inherit their color from the active
/// color scheme at their origin position, expand outward, fade over time,
/// and composite additively onto a black background.
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

    /// Update ripple parameters (speed, width, decay).
    pub fn set_params(&mut self, speed: f32, width: f32, decay: f32) {
        self.speed = speed;
        self.width = width;
        self.decay = decay.clamp(0.0, 1.0);
    }

    /// Spawn a new ripple at a random position, inheriting color from the scheme.
    fn spawn(&mut self, num_leds: u8, colors: &ColorScheme) {
        if self.count >= MAX_RIPPLES {
            return;
        }
        let origin = (self.rng.next() % num_leds as u32) as u8;
        let color = colors.color_at(origin as usize, num_leds as usize);
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

impl Animation for RippleEffect {
    fn render(&mut self, leds: &mut [RGB8], colors: &mut ColorScheme) {
        let num_leds = leds.len() as u8;

        // Maybe spawn a new ripple (color inherited from scheme)
        if self.rng.next() < self.spawn_threshold {
            self.spawn(num_leds, colors);
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

        // Advance color scheme (e.g. rainbow rotation) so ripple colors evolve
        colors.tick();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn color_scheme_solid() {
        let c = ColorScheme::Solid(RGB8 { r: 0, g: 204, b: 0 });
        assert_eq!(c.color_at(0, 10), RGB8 { r: 0, g: 204, b: 0 });
        assert_eq!(c.color_at(9, 10), RGB8 { r: 0, g: 204, b: 0 });
    }

    #[test]
    fn color_scheme_split() {
        let green = RGB8 { r: 0, g: 204, b: 0 };
        let red = RGB8 { r: 204, g: 0, b: 0 };
        let c = ColorScheme::Split(green, red);
        assert_eq!(c.color_at(0, 10), green);
        assert_eq!(c.color_at(4, 10), green);
        assert_eq!(c.color_at(5, 10), red);
        assert_eq!(c.color_at(9, 10), red);
    }

    #[test]
    fn color_scheme_rainbow_varies() {
        let c = ColorScheme::Rainbow { hue: 0, speed: 1 };
        let a = c.color_at(0, 10);
        let b = c.color_at(5, 10);
        assert_ne!(a, b);
    }

    #[test]
    fn color_scheme_tick_advances_rainbow() {
        let mut c = ColorScheme::Rainbow { hue: 0, speed: 5 };
        let before = match c { ColorScheme::Rainbow { hue, .. } => hue, _ => unreachable!() };
        c.tick();
        let after = match c { ColorScheme::Rainbow { hue, .. } => hue, _ => unreachable!() };
        assert_eq!(after, before + 5);
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
        let mut colors = ColorScheme::Solid(RGB8 { r: 255, g: 0, b: 0 });

        pattern.render(&mut buf, &mut colors);
        assert!(buf.iter().any(|c| *c == BACKGROUND));
    }

    #[test]
    fn ripple_dead_ripples_removed() {
        let mut pattern = RippleEffect::new(42);
        let mut buf = [RGB8 { r: 0, g: 0, b: 0 }; 20];
        let mut colors = ColorScheme::Solid(RGB8 { r: 255, g: 0, b: 0 });

        // Force-spawn a ripple with very low amplitude
        pattern.ripples[0] = Ripple {
            origin: 5,
            radius: 10.0,
            amplitude: 0.01,
            color: RGB8 { r: 255, g: 0, b: 0 },
        };
        pattern.count = 1;

        pattern.render(&mut buf, &mut colors);
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

    #[test]
    fn pulse_renders_with_color_scheme() {
        let mut pulse = Pulse::new();
        let mut buf = [RGB8 { r: 0, g: 0, b: 0 }; 10];
        let mut colors = ColorScheme::Solid(RGB8 { r: 0, g: 204, b: 0 });

        pulse.render(&mut buf, &mut colors);
        // All LEDs should be the same (solid color + uniform pulse)
        assert!(buf.windows(2).all(|w| w[0] == w[1]));
    }

    #[test]
    fn static_anim_fills_from_scheme() {
        let mut anim = StaticAnim;
        let mut buf = [RGB8 { r: 0, g: 0, b: 0 }; 10];
        let green = RGB8 { r: 0, g: 204, b: 0 };
        let red = RGB8 { r: 204, g: 0, b: 0 };
        let mut colors = ColorScheme::Split(green, red);

        anim.render(&mut buf, &mut colors);
        assert_eq!(buf[0], green);
        assert_eq!(buf[9], red);
    }
}
