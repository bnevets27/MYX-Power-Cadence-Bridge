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

On first boot, the bridge opens a temporary setup network for 10 minutes,
independent of the bike sensor. Join it and choose a password in your browser.
After setup, the password-protected AP is available when the sensor is connected.
When the sensor sleeps, the AP stops and returns when the sensor reconnects.

To reopen setup after it expires, or to recover a forgotten password, release the
board's BOOT button after startup, then hold it for five seconds. Setup is open
for 10 minutes and accepts a new password without erasing existing settings.

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
| BLE bridge for Zwift / TrainerRoad / Wahoo / Garmin / Apple Fitness+ | âœ… |
| Multiple simultaneous BLE app connections | âœ… |
| Sensor tune button | âœ… |
| Enhanced tune opcode `0x10` | âœ… |
| Raw diagnostics | âœ… |
| Local fallback correction | âœ… |
| Shared AP/LAN web UI | âœ… |
| Separate WiFi and MQTT settings | âœ… |
| MQTT test connection | âœ… |
| Home Assistant discovery | âœ… |
| OTA firmware upload | âœ… |
| Cached sensor reconnect | âœ… |

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

1. Power on the ESP32 and join the open `MYX-Bridge-Setup` network.
2. Open **http://192.168.4.1/** in a browser within 10 minutes.
3. Choose and confirm an administrator password (8 to 63 printable ASCII characters).
   Optionally enter your Wi-Fi network and password, then save and restart.
4. Connect to your Wi-Fi network, or join `MYX-Bridge-Setup` using the new
   password when the bike sensor is connected. Sign in as `admin` using that
   password. If setup expires, release and then hold BOOT for five seconds to reopen it.
5. Pedal to wake the bike sensor, then use the web interface for tuning and settings.

During the temporary setup window, anyone near the device can join its open
network and choose credentials. Only open it in a trusted location. Existing
installations from the earlier generated-password firmware must complete this
setup once; saved Wi-Fi and MQTT settings are preserved unless changed.

The web interface uses HTTP Basic authentication over HTTP: credentials are not
encrypted on the LAN. Use a trusted network. If saved Wi-Fi credentials stop
working, a password-protected AP appears after the connection attempt times out.

Password fields are blank when opened; leave them blank to keep the saved value,
or select the corresponding clear checkbox to remove it.

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

The bridge uses enhanced tune opcode `0x10` when the sensor advertises enhanced
offset compensation; otherwise it uses standard opcode `0x0C`. If enhanced tune
is explicitly rejected as unsupported, it retries using standard tune. The observed
MYX standard success response does not include an offset, so the overview shows
**Not reported** rather than a numeric value. See [tuning protocol](docs/tuning-protocol.md)
for response handling and diagnostic details.

Allow up to 30 seconds for the sensor to respond. If tuning times out, use
**Reconnect Sensor** to establish a fresh connection before trying again.
The bridge keeps an active tune from being interrupted by its measurement
watchdog. A timeout still means no valid tune response was received; it does
not mean calibration succeeded.

MQTT configuration is under **Network Settings → MQTT Settings**. The Connections
card links directly to it and provides **Test MQTT** for the saved configuration.

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

## Firmware Files

Releases now publish two firmware types for each supported chip:

- `myx-bridge-esp32-factory.bin`
- `myx-bridge-esp32-ota.bin`
- `myx-bridge-esp32s3-factory.bin`
- `myx-bridge-esp32s3-ota.bin`

Use the `factory` image for first-time USB flashing from tools such as `https://web.esphome.io/`.

Use the `ota` image only when updating an already-installed bridge through the built-in web updater.

The factory image is a merged binary that includes bootloader + partitions + app in the flash layout expected by ESP Web Tools.

---

## OTA Updates

Use the shared UI and open:

```text
/update
```

Upload the matching `*-ota.bin` file for your board and the bridge will flash and restart.

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
