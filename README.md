# XIAO Drone LED Controller

Firmware for a Seeed XIAO ESP32-C3 that drives a WS2812 LED strip on an FPV drone. Written in Rust (`no_std`), running on the Embassy async runtime.

## Features

- **WS2812 LED strip** driven via SPI+DMA (`ws2812-spi`) on GPIO10 (200 LEDs, configurable)
- **Modular pattern engine** — `Pattern` trait with swappable implementations
  - `RippleEffect` (default) — expanding ring particles with random hues on a deep navy background
  - `RainbowCycle` — classic hue-shifting rainbow
- **Current-limited brightness** — estimates strip draw (8 mA/channel model) and clamps brightness to a configurable mA budget (default 2000 mA)
- **Wi-Fi access point** — open AP (`XIAO-LED-Controller`) with DHCP server and interactive web UI
- **BLE support** — via bleps (not yet wired up)

## Hardware

- Seeed XIAO ESP32-C3
- WS2812B LED strip on GPIO10 (SPI2 MOSI)
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

## HTTP API

Connect to the `XIAO-LED-Controller` Wi-Fi network and open `http://192.168.4.1` in a browser for the interactive control page.

### `GET /`

Returns the control page with brightness and LED count sliders. Slider changes are sent to the device in real time.

### `GET /set`

Updates LED parameters via query string. Returns `204 No Content`.

| Parameter    | Type | Range   | Description                  |
|--------------|------|---------|------------------------------|
| `brightness` | int  | 0–255   | Global brightness level      |
| `num_leds`   | int  | 1–200   | Number of active LEDs driven |
| `fps`        | int  | 1–150   | Animation frame rate         |

Both parameters are optional and can be combined.

**Examples:**

```
GET /set?brightness=128
GET /set?num_leds=60
GET /set?brightness=200&num_leds=100
GET /set?fps=30
```

## Project Structure

```
src/
  bin/main.rs   — entry point, task spawning, LED/Wi-Fi/DHCP/HTTP tasks
  lib.rs        — crate root
  pattern.rs    — Pattern trait and LED pattern implementations
  state.rs      — shared LedState (brightness, num_leds, fps, flight mode) via async mutex
```

## License

Unlicensed / private project.
