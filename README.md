# MYX Power & Cadence Bridge

[![Release](https://img.shields.io/github/v/release/bnevets27/MYX-Power-Cadence-Bridge)](https://github.com/bnevets27/MYX-Power-Cadence-Bridge/releases/latest)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

An ESP32 firmware that bridges the **BODi MYX** bike's proprietary Bluetooth sensor to standard cycling apps and now also includes:

- sensor tuning / diagnostics
- local fallback correction
- WiFi setup
- LAN status UI
- MQTT / Home Assistant publishing
- OTA firmware updates

This branch uses **one unified firmware image**. The BLE bridge remains the primary function.

The MYX bike sensor (`BKSNSR*`) XOR-masks its BLE data, making it unreadable by third-party apps. This firmware decodes the data in real time and re-broadcasts it as a standard BLE Cycling Power + Speed & Cadence sensor while also serving a shared web UI and optional MQTT output.

---

## Unified Behavior

### AP-only / unprovisioned mode

If no WiFi has been saved yet:

1. Boot and begin BLE recovery
2. Scan for the bike sensor
3. Connect to the bike sensor
4. Start the AP and shared web UI only after the sensor is connected

If the bike sensor sleeps or disconnects:

- the AP shuts down
- the bridge returns to BLE recovery indefinitely
- once the bike sensor reconnects, the AP and UI return automatically

This is intentional. AP availability is tied to the sensor connection so the BLE bridge keeps priority.

### Provisioned LAN mode

If WiFi credentials are saved:

1. Boot and connect to WiFi
2. Serve the same UI on LAN
3. Recover the bike sensor in the background

If the bike sensor sleeps or disconnects:

- LAN UI stays up
- MQTT stays up
- OTA stays up
- cached reconnect is tried first
- BLE scan fallback continues until the sensor is rediscovered

No reboot should be required just because the bike sensor went away.

---

## Features

| Capability | Supported |
|-----------|-----------|
| BLE bridge for Zwift / TrainerRoad / Wahoo / Garmin / Apple Fitness+ | ✅ |
| Multiple simultaneous BLE app connections | ✅ |
| Sensor tune button | ✅ |
| Enhanced tune opcode `0x10` | ✅ |
| Raw diagnostics | ✅ |
| Local fallback correction | ✅ |
| Shared AP/LAN web UI | ✅ |
| Separate WiFi and MQTT settings | ✅ |
| MQTT test connection | ✅ |
| Home Assistant discovery | ✅ |
| OTA firmware upload | ✅ |
| Cached sensor reconnect | ✅ |

---

## Hardware

Any board built on the **original ESP32 chip** works well. Recommended:

- **ESP32-DevKitC**
- **ESP32-WROOM-32**

Also supported:

- **ESP32-S3-DevKitC**

Not supported:

- **ESP32-S2** - no BLE
- **ESP32-C3 / C6** - not currently supported here

Minimum flash: **4 MB**

---

## Building

### ESP32

```bash
python -m platformio run -e esp32
```

### ESP32-S3

```bash
python -m platformio run -e esp32s3
```

### Flashing example

```bash
python -m platformio run --target upload --environment esp32
```

### Serial monitor

```bash
python -m platformio device monitor --environment esp32 --baud 115200
```

---

## First Power-On

1. Place the ESP32 near the MYX bike sensor
2. Pedal briefly to wake the bike sensor
3. Wait for the bridge to connect
4. If no WiFi is saved yet, join:

```text
SSID: MYX-Bridge-Setup
URL:  http://192.168.4.1/
```

5. Use the shared page to:
   - tune the sensor
   - adjust local correction
   - save WiFi
   - save MQTT
   - test MQTT

Once WiFi is saved, the device reboots and the same UI moves to LAN mode.

---

## Shared Web UI

The same page is used in AP and LAN mode and includes:

- sensor status / reconnect status
- tune / diagnostics
- local fallback correction
- WiFi settings
- MQTT settings
- status / log text
- OTA entry point

Important actions:

- `Tune Sensor`
- `Find Sensor Again`
- `Reconnect Sensor`
- `Save WiFi`
- `Save MQTT`
- `Test MQTT`
- `Reset WiFi`
- `Reset MQTT`
- `Full Reset`

### Reset behavior

- `Reset WiFi` clears only WiFi settings
- `Reset MQTT` clears only MQTT settings
- `Full Reset` clears WiFi, MQTT, cached sensor identity, and calibration settings

---

## Tuning and Local Correction

The firmware keeps the current dev-branch tuning and correction behavior.

### Tune flow

1. Pedal for about 30 seconds to wake the sensor
2. Get off the bike
3. Set the crank vertical
4. Press `Tune Sensor`
5. Wait for completion

This firmware uses **enhanced tune opcode `0x10`** as the default and only tune path.

### If tuning is not enough

Use local fallback correction:

- `Local zero offset`
- `Scale factor`
- cadence-specific offsets

Raw diagnostics can also be enabled from the same page.

---

## MQTT / Home Assistant

MQTT is configured separately from WiFi.

- Saving WiFi does not change MQTT settings
- Saving MQTT does not change WiFi settings
- `Test MQTT` attempts a broker connection and reports success/failure in the UI

Published metrics include:

- power
- cadence
- battery
- accumulated energy
- sensor connected state

Home Assistant discovery uses the standard `homeassistant` discovery prefix.

---

## OTA Updates

Use the shared UI and open:

```text
/update
```

Upload a new `.bin` file and the bridge will flash and restart.

The repo uses the OTA partition table in [partitions_ota.csv](partitions_ota.csv).

---

## LED Status

| Pattern | State |
|---------|-------|
| Slow blink (1 s) | Searching / recovering sensor |
| Fast blink (250 ms) | Sensor connected, waiting for app |
| Solid on | Sensor connected and at least one app connected |

---

## How It Works

```text
MYX Bike (BKSNSR*)          ESP32 Bridge                 Destinations
  BLE sensor        ------> XOR decode    ----------->  Zwift / Wahoo / Garmin
  XOR 0xAA masked             re-broadcast ----------->  MQTT / Home Assistant
                                             --------->  AP or LAN web UI
```

The MYX sensor transmits Cycling Power Service packets but XOR-masks every byte with `0xAA`. The bridge decodes those packets and re-publishes the corrected values over BLE and optional MQTT while keeping diagnostics available in the shared UI.

---

## Troubleshooting

**ESP32 can't find the sensor**

Make sure the bike sensor is awake and not already connected to another device. The bridge keeps retrying.

**AP disappeared**

In AP-only mode, that means the bike sensor disconnected or went to sleep. Pedal to wake the bike and the AP will return after sensor reconnect.

**LAN UI stays up but the bike is disconnected**

That is expected in provisioned mode. The bridge will try cached reconnect first and then keep scanning until the bike wakes up again.

**MQTT test fails**

Check that WiFi is connected, the broker host/port are correct, and any username/password is valid.

**Power feels off after tune**

Use local fallback correction for zero offset, scale, and cadence-specific adjustments.

---

## License

MIT - free to use, modify, and distribute.
