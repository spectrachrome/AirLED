# AirLED Flutter BLE Integration Guide

This guide is for building a Flutter companion app that controls the AirLED ESP32-C3 LED controller over Bluetooth Low Energy (BLE).

## Architecture Overview

The device runs on an ESP32-C3 (XIAO form factor) and exposes a **Nordic UART Service (NUS)** over BLE. Communication is newline-delimited JSON over a serial-like BLE pipe. BLE is the primary and only control interface for the app.

## BLE Connection

### Advertising

- **Device name:** `AirLED`
- **Flags:** LE General Discoverable, no BR/EDR

### NUS UUIDs

| Role | UUID |
|------|------|
| Service | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` |
| RX (app → device, write) | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` |
| TX (device → app, notify) | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` |

### Recommended Flutter package

Use `flutter_reactive_ble` or `flutter_blue_plus` for BLE communication.

### Connection flow

1. Scan for devices advertising name `AirLED`
2. Connect and discover services
3. Subscribe to notifications on TX characteristic
4. Send `{"GetState":null}\n` on RX to request initial state
5. The device also auto-pushes state on certain events (see below)

## Wire Protocol

### Sending commands (app → device)

- Write JSON + `\n` to the **RX characteristic**
- If the command exceeds 20 bytes, split into 20-byte chunks and write sequentially
- The device reassembles until it sees `\n`

### Receiving responses (device → app)

- Subscribe to **TX characteristic** notifications
- Responses arrive as 20-byte chunks
- Buffer incoming bytes until you see `\n`
- Then parse the complete message as either JSON or a plain-text ack/error

### Response types

| Type | Format | When |
|------|--------|------|
| State | JSON object + `\n` | Response to `GetState`, or auto-push on state change |
| Ack | `ok\n` | Response to any successful `Set*` command |
| Error | `err:reason\n` | Parse failure or invalid value |

Error reasons: `err:parse`, `err:unknown_color_mode`, `err:unknown_anim_mode`

## Commands (app → device)

Commands use serde's externally-tagged enum format. All commands must be terminated with `\n`.

### GetState

Request the full device state snapshot.

```json
{"GetState":null}
```

### SetBrightness

```json
{"SetBrightness":{"value":128}}
```

- `value`: `u8` (0–255), default **255**

### SetNumLeds

```json
{"SetNumLeds":{"value":180}}
```

- `value`: `u16` (clamped 1–200), default **180**

### SetFps

```json
{"SetFps":{"value":100}}
```

- `value`: `u8` (clamped 1–150), default **100**

### SetMaxCurrent

Maximum LED strip current budget in milliamps.

```json
{"SetMaxCurrent":{"value":2000}}
```

- `value`: `u32` (clamped 100–2500), default **2000**

### SetColorMode

```json
{"SetColorMode":{"mode":"rainbow"}}
```

Valid modes:

| Key | Description |
|-----|-------------|
| `solid_green` | Solid green |
| `solid_red` | Solid red |
| `split` | Green/red split (port/starboard navigation lights) |
| `rainbow` | Rainbow HSV gradient |

Default: **`split`**

### SetAnimMode

Changing animation mode resets animation parameters to defaults.

```json
{"SetAnimMode":{"mode":"pulse"}}
```

Valid modes:

| Key | Description |
|-----|-------------|
| `static` | No animation, static fill |
| `pulse` | Sinusoidal breathing pulse |
| `ripple` | Expanding ring ripples |

Default: **`pulse`**

### SetColorBalance

Per-channel color balance applied after gamma correction.

```json
{"SetColorBalance":{"r":255,"g":180,"b":240}}
```

- `r`, `g`, `b`: `u8` (0–255), defaults **r=255, g=180, b=240**

### SetUseHsi

Toggle HSI color space for rainbow mode (more uniform perceived brightness).

```json
{"SetUseHsi":{"value":true}}
```

- `value`: `bool`, default **false**

### SetHueSpeed

Rainbow hue rotation speed (only affects rainbow color mode).

```json
{"SetHueSpeed":{"value":2}}
```

- `value`: `u8` (clamped 1–10), default **1**

### SetPulseSpeed

Pulse animation phase increment per frame (only applies in pulse mode).

```json
{"SetPulseSpeed":{"value":600}}
```

- `value`: `u16` (clamped 100–2000), default **600**

### SetPulseMinBrightness

Minimum brightness floor for pulse as a percentage (only applies in pulse mode).

```json
{"SetPulseMinBrightness":{"value":42}}
```

- `value`: `u8` (clamped 0–80), default **42**

### SetRippleSpeed

Ripple expansion speed in fixed-point x10 (e.g. 15 = 1.5 LEDs/frame). Only applies in ripple mode.

```json
{"SetRippleSpeed":{"value":15}}
```

- `value`: `u8` (clamped 5–50), default **15**

### SetRippleWidth

Ripple wavefront half-width in fixed-point x10 (e.g. 190 = 19.0 LEDs). Only applies in ripple mode.

```json
{"SetRippleWidth":{"value":190}}
```

- `value`: `u8` (clamped 10–255), default **190**

### SetRippleDecay

Per-frame amplitude decay percentage. Only applies in ripple mode.

```json
{"SetRippleDecay":{"value":97}}
```

- `value`: `u8` (clamped 90–99), default **97**

## StateResponse (device → app)

Full JSON state snapshot. Approximately 250 bytes serialized.

```json
{
  "brightness": 255,
  "num_leds": 180,
  "fps": 100,
  "max_current_ma": 2000,
  "color_mode": "split",
  "anim_mode": "pulse",
  "bal_r": 255,
  "bal_g": 180,
  "bal_b": 240,
  "use_hsi": false,
  "hue_speed": 1,
  "pulse_speed": 600,
  "pulse_min_brightness": 42,
  "ripple_speed": 15,
  "ripple_width": 190,
  "ripple_decay": 97,
  "fc_connected": false,
  "flight_mode": "arming_forbidden",
  "tx_linked": false
}
```

### Field reference

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `brightness` | int | 0–255 | Global LED brightness |
| `num_leds` | int | 1–200 | Active LED count |
| `fps` | int | 1–150 | Target frame rate |
| `max_current_ma` | int | 100–2500 | Current limit (mA) |
| `color_mode` | string | see above | Active color scheme |
| `anim_mode` | string | see above | Active animation |
| `bal_r` | int | 0–255 | Red color balance |
| `bal_g` | int | 0–255 | Green color balance |
| `bal_b` | int | 0–255 | Blue color balance |
| `use_hsi` | bool | | HSI mode for rainbow |
| `hue_speed` | int | 1–10 | Rainbow rotation speed |
| `pulse_speed` | int | 100–2000 | Pulse animation speed |
| `pulse_min_brightness` | int | 0–80 | Pulse min brightness % |
| `ripple_speed` | int | 5–50 | Ripple speed (x10) |
| `ripple_width` | int | 10–255 | Ripple width (x10) |
| `ripple_decay` | int | 90–99 | Ripple decay % |
| `fc_connected` | bool | | Flight controller connected |
| `flight_mode` | string | see below | Current flight mode |
| `tx_linked` | bool | | RC transmitter link active (RSSI > 0) |

### Flight modes

| Value | Description |
|-------|-------------|
| `arming_forbidden` | Pre-flight checks failed, cannot arm |
| `arming_allowed` | Disarmed, ready to arm |
| `armed` | Motors armed and active |
| `failsafe` | Communication lost |

When `fc_connected` is `true`, the device overrides user LED patterns with flight-mode-specific displays. The app should show the flight mode status but can still send `Set*` commands — they apply when the FC is disconnected or disarmed.

## Auto-push behavior

The device automatically sends a full `StateResponse` (without the app requesting it) when:

1. **Flight mode changes** — FC arms/disarms, enters failsafe, etc.
2. **FC connection/disconnection** — `fc_connected` toggles

These are event-driven, not polled. The app should always be ready to receive unsolicited `StateResponse` messages on the TX notification stream.

## UI design notes

### Parameter visibility

Not all parameters are relevant at all times. Show/hide based on current mode:

| Parameter | Visible when |
|-----------|-------------|
| Hue Speed, Use HSI | `color_mode == "rainbow"` |
| Pulse Speed, Min Brightness | `anim_mode == "pulse"` |
| Ripple Speed/Width/Decay | `anim_mode == "ripple"` |
| Brightness, LED count, FPS, Current Limit, Color Balance | Always |

### Recommended controls

| Parameter | Control type |
|-----------|-------------|
| Brightness | Slider (0–255) |
| LED count | Slider (1–200) |
| FPS | Slider (1–150) |
| Current Limit | Slider (100–2500, step 50) |
| Color Mode | Dropdown or segmented control |
| Animation Mode | Dropdown or segmented control |
| Color Balance R/G/B | Three sliders (0–255) |
| Use HSI | Toggle/checkbox |
| Hue Speed | Slider (1–10) |
| Pulse Speed | Slider (100–2000) |
| Min Brightness | Slider (0–80) with % label |
| Ripple Speed | Slider (5–50) |
| Ripple Width | Slider (10–255) |
| Ripple Decay | Slider (90–99) with % label |

### Debouncing

Sliders should debounce sends (e.g. 80ms) to avoid flooding the BLE link. The device handles rapid updates fine, but BLE throughput is limited.

### Connection status

Show a connection indicator. The device flashes its LEDs blue on connect (1 flash) and disconnect (2 flashes), but the app should track connection state independently via the BLE library.

### Flight mode display

When `fc_connected` is `true`, show the flight mode prominently. Consider disabling or dimming the color/animation controls since the device overrides them based on flight mode:

- **Armed:** Rainbow ripple effect (not user-controllable)
- **Failsafe:** Sliding red bars (not user-controllable)
- **Arming forbidden:** Slow red pulse (not user-controllable)
- **Arming allowed:** Green pulse (not user-controllable)

## Example: minimal connection in Dart pseudocode

```dart
// Scan
final device = await ble.scanForDevice(name: 'AirLED');

// Connect
final connection = await ble.connect(device.id);

// Discover NUS service
final rx = QualifiedCharacteristic(
  serviceId: Uuid.parse('6e400001-b5a3-f393-e0a9-e50e24dcca9e'),
  characteristicId: Uuid.parse('6e400002-b5a3-f393-e0a9-e50e24dcca9e'),
  deviceId: device.id,
);
final tx = QualifiedCharacteristic(
  serviceId: Uuid.parse('6e400001-b5a3-f393-e0a9-e50e24dcca9e'),
  characteristicId: Uuid.parse('6e400003-b5a3-f393-e0a9-e50e24dcca9e'),
  deviceId: device.id,
);

// Subscribe to notifications and reassemble
final buffer = StringBuffer();
ble.subscribeToCharacteristic(tx).listen((bytes) {
  buffer.write(utf8.decode(bytes));
  while (buffer.toString().contains('\n')) {
    final str = buffer.toString();
    final idx = str.indexOf('\n');
    final message = str.substring(0, idx);
    buffer.clear();
    buffer.write(str.substring(idx + 1));
    handleMessage(message); // parse JSON or ack/error
  }
});

// Request initial state
await ble.writeCharacteristic(rx,
  value: utf8.encode('{"GetState":null}\n'));

// Send a command
await ble.writeCharacteristic(rx,
  value: utf8.encode('{"SetBrightness":{"value":128}}\n'));
```
