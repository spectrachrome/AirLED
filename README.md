# XIAO Drone LED Controller

Firmware for a Seeed XIAO ESP32-C3 that drives a WS2812 LED strip on an FPV drone. Written in Rust (`no_std`), running on the Embassy async runtime.

## Features

- **WS2812 LED strip** driven via RMT peripheral (150 LEDs, configurable)
- **Modular pattern engine** — `Pattern` trait with swappable implementations
  - `RippleEffect` (default) — expanding ring particles with random hues on a deep navy background
  - `RainbowCycle` — classic hue-shifting rainbow
- **Current-limited brightness** — estimates strip draw (8 mA/channel model) and clamps brightness to a configurable mA budget (default 2000 mA)
- **Wi-Fi access point** — open AP (`XIAO-LED-Controller`) with DHCP server and HTTP status page
- **BLE support** — via bleps (not yet wired up)

## Hardware

- Seeed XIAO ESP32-C3
- WS2812B LED strip on GPIO2
- USB power (2A budget)

## Building

Requires a Rust nightly toolchain with the `riscv32imc-unknown-none-elf` target.

```sh
cargo build
```

## Flashing

Flash with [probe-rs](https://probe.rs/):

```sh
cargo run
```

## Project Structure

```
src/
  bin/main.rs   — entry point, task spawning, LED/Wi-Fi/DHCP/HTTP tasks
  lib.rs        — crate root
  pattern.rs    — Pattern trait and LED pattern implementations
  state.rs      — shared LedState (brightness, flight mode) via async mutex
```

## License

Unlicensed / private project.
