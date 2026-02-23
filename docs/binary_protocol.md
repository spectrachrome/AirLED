# AirLED Binary BLE Protocol v1

> Compact binary protocol for BLE communication between the AirLED controller
> (ESP32-C3) and a companion app. Replaces the previous JSON-over-NUS protocol.

## Design Goals

- **Compact**: Full state snapshot in a single BLE write (no chunking)
- **Simple**: Fixed-size commands, no framing or escaping needed
- **Extensible**: Version byte + reserved space for future fields
- **Zero-copy friendly**: All multi-byte values are little-endian, naturally aligned

## Transport

| Property          | Value                                      |
|-------------------|--------------------------------------------|
| BLE Service       | Nordic UART Service (NUS) — unchanged      |
| RX Characteristic | `6e400002-...` (app → device, Write)       |
| TX Characteristic | `6e400003-...` (device → app, Notify)      |
| Byte order        | Little-endian throughout                   |
| Integrity         | Delegated to BLE link layer (CRC + retx)   |

## Frame Format

Every frame (command or response) starts with a 1-byte header:

```
┌─────────┐
│ CMD: u8 │  Payload (0–N bytes, size determined by CMD)
└─────────┘
```

Each command ID implies a fixed payload length — no length field needed.

---

## Commands (App → Device)

| ID   | Name               | Payload              | Size | Description                       |
|------|--------------------|----------------------|------|-----------------------------------|
| 0x01 | `GetState`         | —                    | 1    | Request full state snapshot       |
| 0x02 | `GetVersion`       | —                    | 1    | Request firmware/protocol version |
| 0x10 | `SetBrightness`    | `u8`                 | 2    | 0–255                             |
| 0x11 | `SetNumLeds`       | `u16`                | 3    | 1–200                             |
| 0x12 | `SetFps`           | `u8`                 | 2    | 1–150                             |
| 0x13 | `SetMaxCurrent`    | `u16`                | 3    | 100–2500 mA                       |
| 0x14 | `SetColorMode`     | `u8`                 | 2    | See Color Mode table              |
| 0x15 | `SetAnimMode`      | `u8`                 | 2    | See Anim Mode table               |
| 0x16 | `SetColorBalance`  | `u8, u8, u8`         | 4    | R, G, B scaling (0–255 each)      |
| 0x17 | `SetUseHsi`        | `u8`                 | 2    | 0 = HSV, 1 = HSI                  |
| 0x18 | `SetHueSpeed`      | `u8`                 | 2    | 1–10 (rainbow only)               |
| 0x19 | `SetPulseSpeed`    | `u16`                | 3    | 100–2000 ms period                |
| 0x1A | `SetPulseMinBrt`   | `u8`                 | 2    | 0–80 (%)                          |
| 0x1B | `SetRippleSpeed`   | `u8`                 | 2    | 5–50 (×0.1 fixed-point)           |
| 0x1C | `SetRippleWidth`   | `u8`                 | 2    | 10–255 (×0.1 fixed-point)         |
| 0x1D | `SetRippleDecay`   | `u8`                 | 2    | 90–99 (%)                         |
| 0xF0 | `TestPattern`      | `u8`                 | 2    | 5 s solid color (see table below) |

Command IDs are grouped:
- `0x01–0x0F` — queries
- `0x10–0x3F` — setters (user-configurable fields)
- `0xF0–0xFF` — system commands

### Test Pattern Color (`u8`)

| Value | Color  |
|-------|--------|
| 0     | Red    |
| 1     | Green  |
| 2     | Blue   |
| 3     | White  |

Triggers a 5-second solid-color episode at full intensity, bypassing all
post-processing (gamma, color balance, brightness, current limit).
A new `TestPattern` command restarts the 5-second timer.

---

## Responses (Device → App)

### Ack / Error (1 byte)

Sent after every setter or system command:

| ID   | Name         | Meaning                          |
|------|--------------|----------------------------------|
| 0x00 | `Ok`         | Command accepted                 |
| 0xE0 | `ErrParse`   | Unknown or malformed command     |
| 0xE1 | `ErrRange`   | Value out of valid range         |
| 0xE2 | `ErrBusy`    | Device busy (e.g. flash write)   |

### State Snapshot (response to `GetState` or unsolicited push)

Response ID: **0x01**

```
Offset  Size  Field               Notes
─────────────────────────────────────────────────────
 0      1     response_id         0x01
 1      1     protocol_version    1
 2      1     brightness          0–255
 3      2     num_leds            u16 LE, 1–200
 5      1     fps                 1–150
 6      2     max_current_ma      u16 LE, 100–2500
 8      1     color_mode          enum (see table)
 9      1     anim_mode           enum (see table)
10      1     color_bal_r         0–255
11      1     color_bal_g         0–255
12      1     color_bal_b         0–255
13      1     use_hsi             0 or 1
14      1     hue_speed           1–10
15      2     pulse_speed         u16 LE, 100–2000
17      1     pulse_min_brt       0–80
18      1     ripple_speed        5–50
19      1     ripple_width        10–255
20      1     ripple_decay        90–99
21      1     flags               bitfield (see below)
─────────────────────────────────────────────────────
Total: 22 bytes
```

**Flags byte (offset 21)**:

```
Bit  Field
───────────────────
 0   fc_connected
 1   tx_linked
 2   armed
 3   failsafe
 4   arming_allowed
5–7  reserved (0)
```

Flight mode is encoded in bits 2–4:

| armed | failsafe | arming_allowed | Meaning            |
|-------|----------|----------------|--------------------|
| 0     | 0        | 0              | Arming forbidden   |
| 0     | 0        | 1              | Arming allowed     |
| 0     | 1        | x              | Failsafe           |
| 1     | 0        | x              | Armed              |

When `fc_connected = 0`, bits 2–4 should be ignored by the app.

### Version Response (response to `GetVersion`)

Response ID: **0x02**

```
Offset  Size  Field               Notes
─────────────────────────────────────────────────────
 0      1     response_id         0x02
 1      1     protocol_version    1
 2      1     fw_major            Firmware semver major
 3      1     fw_minor            Firmware semver minor
 4      1     fw_patch            Firmware semver patch
─────────────────────────────────────────────────────
Total: 5 bytes
```

---

## Enum Encodings

### Color Mode (`u8`)

| Value | Mode          |
|-------|---------------|
| 0     | `solid_green`  |
| 1     | `solid_red`    |
| 2     | `split`        |
| 3     | `rainbow`      |

### Animation Mode (`u8`)

| Value | Mode     |
|-------|----------|
| 0     | `static` |
| 1     | `pulse`  |
| 2     | `ripple` |

---

## State Push

The device pushes the full 22-byte state snapshot (ID `0x01`) over the TX
characteristic whenever the MSP task detects a flight-mode change. This is
the same format as the `GetState` response — the app does not need to poll.

---

## Size Comparison

| Metric                | JSON protocol | Binary protocol |
|-----------------------|---------------|-----------------|
| State snapshot        | ~300 bytes    | 22 bytes        |
| BLE chunks needed*    | 15            | 2               |
| Setter command        | 20–40 bytes   | 2–4 bytes       |
| Ack response          | 3–30 bytes    | 1 byte          |

*At default 20-byte MTU. With negotiated 23+ byte MTU, binary state fits in 1 ATT packet.

---

## Examples

### Set brightness to 128

```
App  →  Device:   [0x10, 0x80]            (2 bytes)
Device  →  App:   [0x00]                  (1 byte: Ok)
```

### Set color balance to (255, 200, 220)

```
App  →  Device:   [0x16, 0xFF, 0xC8, 0xDC]  (4 bytes)
Device  →  App:   [0x00]                     (1 byte: Ok)
```

### Request full state

```
App  →  Device:   [0x01]                  (1 byte)
Device  →  App:   [0x01, 0x01, 0xFF, 0xB4, 0x00, 0x64, 0xD0, 0x07,
                    0x02, 0x01, 0xFF, 0xB4, 0xF0, 0x00, 0x01, 0x58,
                    0x02, 0x2A, 0x0F, 0xBE, 0x61, 0x00]
                                          (22 bytes)
```

Decoded: brightness=255, num_leds=180, fps=100, max_current=2000,
color_mode=split, anim_mode=pulse, bal=(255,180,240), hsi=off,
hue_speed=1, pulse_speed=600, pulse_min=42, ripple_speed=15,
ripple_width=190, ripple_decay=97, flags=0x00 (FC disconnected).
