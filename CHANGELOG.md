# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Conventional Commits](https://www.conventionalcommits.org/).

## [Unreleased]

### Added

- Temporal dithering module (`src/dither.rs`): adds extra perceived bit depth to WS2812B LEDs by varying quantized output frame-to-frame faster than flicker fusion
- `DitherMode` enum with four modes: `Off`, `ErrorDiffusion` (smooth gradients), `Ordered` (Bayer 4x4, deterministic), `Hybrid` (error diffusion + correlated ordered for low brightness)
- 8.8 fixed-point gamma LUTs (3 x 256 entries, 1536 bytes flash) computed at compile time for high-precision gamma correction in the dithered path
- `SetDitherMode` and `SetDitherFps` BLE commands for runtime control of dithering
- `dither_mode` and `dither_fps` fields in BLE `StateResponse`
- Inner dither loop in LED task: animation renders at `fps` rate, strip refreshes at `dither_fps` rate (100–960 Hz) with different dither patterns between animation frames
- Dither state auto-reset on mode change, strobe activation, and BLE flash sequences
- Unit tests for dither algorithms (error diffusion convergence, ordered determinism, Fix16 gamma roundtrip)
- `DisplayTestPattern` BLE command: temporarily force a color + animation combo for a given duration, overriding FC flight mode patterns
- `CancelTestPattern` BLE command: stop a running test pattern immediately
- `test_active` field in BLE `StateResponse` (true when a test pattern is playing)
- RSSI-based TX link detection via MSP_ANALOG — strobe only activates when RSSI > 0 (replaces unreliable stick-center heuristic)
- `tx_linked` field exposed in BLE `StateResponse` for app display

### Removed

- Wi-Fi AP hotspot, HTTP web UI, and DHCP server (BLE is now the sole control interface)
- `embassy-net`, `smoltcp`, `edge-dhcp`, and `embedded-io` dependencies
- `wifi` and `coex` features from `esp-radio` (no longer needed without Wi-Fi)

### Added

- BLE Nordic UART Service (NUS) for app control via JSON protocol
- `ble` module (`src/ble.rs`): Command/Response types with serde, command handler, state snapshot builder, JSON serialization helpers — all `no_std`, no heap, unit-testable
- `ble_task`: async task advertising as "AirLED", serving NUS GATT service with chunked notifications
- Newline-delimited JSON protocol over BLE: externally-tagged command enums (`{"GetState":null}`, `{"SetBrightness":{"value":128}}`), flat `StateResponse` struct, plain `"ok\n"` / `"err:reason\n"` acks
- Auto-push state on BLE connect and on MSP flight mode change via `STATE_CHANGED` signal
- Chunked BLE notifications (20-byte MTU) for state responses (~250 bytes)
- MSP flight controller integration over UART1 (GPIO20 RX, GPIO21 TX, 115200 baud)
- `msp` module (`src/msp.rs`): MSPv1 frame builder, response parser state machine, BOXNAMES decoder, flight mode resolver — all `no_std`, no heap, fully unit-testable
- `msp_task`: async task polling MSP_STATUS at ~10 Hz, with BOXNAMES discovery at startup
- `fc_connected` field in `LedState` for FC connection tracking
- Armed flight mode override: cyclic rainbow show patterns (static rainbow, rainbow pulse, rainbow chase, rainbow sparkle), auto-advancing every ~10 s
- Failsafe flight mode override: sliding red bars with black gaps
- Disarmed / no FC connected: normal user-selected pattern from web UI (unchanged behavior)
- Two independent axes: `ColorScheme` (solid green, solid red, split, rainbow) × `Animation` (static, pulse, ripple) for 12 combinations
- `ColorScheme` enum in `src/pattern.rs` with `color_at()`, `tick()`, and `set_hue_speed()`
- `Animation` trait replacing `Pattern` trait — `render(&mut self, leds, colors)`
- `StaticAnim` animation (fills LEDs from color scheme, no motion)
- `Pulse` animation (merged `SplitPulse` + `SinePulse` — works with any color scheme)
- Ripples now inherit color from the active color scheme at their origin position
- Two dropdown selectors in web UI: "Color" and "Animation" (independently selectable)
- `ColorMode`, `AnimMode`, `ColorModeParams`, `AnimModeParams` in `src/state.rs`
- Per-axis param slider visibility: `data-color` and `data-anim` attributes checked independently
- `hsi_to_rgb()` function in `src/pattern.rs` — HSI color space conversion for more uniform perceived brightness across hues (ported from [SO#69328218](https://stackoverflow.com/q/69328218), CC BY-SA 4.0)
- Runtime "Use HSI color space" checkbox in web UI (rainbow color mode only), backed by `use_hsi` field in `LedState`
- Per-channel gamma correction LUTs: R γ=2.6, G γ=2.7, B γ=2.5 (768 bytes flash) — steeper green curve tames eye sensitivity and WS2812B green die efficiency
- `PostEffect::ColorBalance { r, g, b }` — per-channel max scaling (0–255), applied after gamma
- Runtime "Balance R/G/B" sliders in web UI (defaults: R=255, G=180, B=240, matching typical WS2812B correction)

### Changed

- Default pulse `min_intensity_pct` raised from 40 to 42

- Animation mode changes in web UI reset animation params to defaults
- `led_task` maintains a `ColorScheme` instance, rebuilds on color mode change
- `parse_query_params()` uses `color=...&anim=...` query keys instead of `mode=...`

### Removed

- `PatternMode` and `PatternParams` enums (replaced by `ColorMode`/`AnimMode` axes)
- `Pattern` trait (replaced by `Animation` trait with color scheme parameter)
- `SplitPulse`, `SinePulse`, `RainbowCycle` structs (merged into `Pulse`/`StaticAnim` + `ColorScheme`)
- Single "Mode" dropdown in web UI (replaced by separate "Color" and "Animation" dropdowns)

- Post-processing pipeline (`src/postfx.rs`) with composable `PostEffect` enum: `Brightness`, `Gamma`, `CurrentLimit`
- Gamma 2.6 correction via 256-byte LUT (Adafruit gamma8 table)
- SPI+DMA LED driver via `ws2812-spi` (replaces RMT, supports 200+ LEDs)
- Interactive web UI with brightness (0–255), LED count (1–200), and FPS (1–150) sliders
- `/set` HTTP endpoint for real-time parameter updates via query params
- Configurable `num_leds` field in `LedState` (default 200, max 200)
- Configurable `fps` field in `LedState` (default 50, range 1–150)
- `Pattern` trait in `src/pattern.rs` for modular LED patterns
- `RainbowCycle` pattern implementing `Pattern` trait
- `RippleEffect` pattern: particle-based expanding rings on ring topology with random hues and deep navy background
- Current-limited brightness clamping (8 mA/channel model, configurable mA budget via shared state)
- Shared `LedState` with `brightness` and `FlightMode` in `src/state.rs`
- `embassy-sync` dependency for async mutex
- `led_task` reads brightness from shared state instead of hardcoded value
- Initial project setup with ESP32-C3 (XIAO) target
- Embassy async runtime with esp-hal
- BLE and WiFi support via esp-wifi
- WS2812 LED support via esp-hal-smartled2 and smart-leds
- Wi-Fi AP hotspot (open, SSID: XIAO-LED-Controller) with static IP 192.168.4.1
- HTTP test page on port 80
- DHCP server via edge-dhcp, assigns IPs in 192.168.4.50–200 range
- esp-bootloader-esp-idf app descriptor for probe-rs flashing
- Project guidelines (CLAUDE.md)
- Changelog

### Changed

- Pulse patterns (`SplitPulse`, `SinePulse`) linearized — gamma LUT now handles perceptual dimming
- Pulse `min_intensity` raised from 0.1 to 0.3 so floor survives gamma correction
- Default brightness raised from 128 to 200 to compensate for gamma darkening
- `led_task` uses `apply_pipeline()` instead of ad-hoc `clamp_brightness()` + `smart_leds::brightness()`
- LED driver switched from RMT (`esp-hal-smartled2`) to SPI+DMA (`ws2812-spi`) on GPIO10 (MOSI)
- `MAX_LEDS` increased from 150 to 200 (no longer constrained by RMT buffer)
- Default `num_leds` changed from 150 to 200
- Compiler `opt-level` changed from `"s"` to `3` for both dev and release profiles
- `NUM_LEDS` renamed to `MAX_LEDS` (200) as compile-time buffer max; active count is now runtime-configurable
- Static HTML test page replaced with interactive control UI
- Web server tx_buffer increased to 4096 to fit HTML page
- Upgraded esp-hal to 1.0.0 (from 1.0.0-beta.0)
- Replaced esp-wifi with esp-radio 0.17.0
- Replaced esp-hal-embassy with esp-rtos 0.2.0
- Bumped embassy-executor to 0.9.1, embassy-time to 0.5.0, embassy-net to 0.8.0
- Bumped esp-alloc to 0.9.0, embedded-io to 0.7.1, embedded-io-async to 0.7.0
- Manual RTOS scheduler init with esp_rtos::start() instead of #[esp_rtos::main] macro

### Removed

- `esp-hal-smartled2` / RMT-based LED driver (replaced by SPI+DMA)
- ws2812-esp32-rmt-driver (incompatible with bare-metal no_std)
