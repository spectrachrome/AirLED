# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Conventional Commits](https://www.conventionalcommits.org/).

## [Unreleased]

### Added

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
