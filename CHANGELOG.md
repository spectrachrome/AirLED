# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Conventional Commits](https://www.conventionalcommits.org/).

## [Unreleased]

### Added

- `Pattern` trait in `src/pattern.rs` for modular LED patterns
- `RainbowCycle` pattern implementing `Pattern` trait
- `RippleEffect` pattern: particle-based expanding rings on ring topology with random hues and deep navy background
- Current-limited brightness clamping (8 mA/channel model, 2A budget) to protect USB power supply
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

- Upgraded esp-hal to 1.0.0 (from 1.0.0-beta.0)
- Replaced esp-wifi with esp-radio 0.17.0
- Replaced esp-hal-embassy with esp-rtos 0.2.0
- Bumped embassy-executor to 0.9.1, embassy-time to 0.5.0, embassy-net to 0.8.0
- Bumped esp-alloc to 0.9.0, embedded-io to 0.7.1, embedded-io-async to 0.7.0
- Manual RTOS scheduler init with esp_rtos::start() instead of #[esp_rtos::main] macro

### Removed

- ws2812-esp32-rmt-driver (incompatible with bare-metal no_std)
