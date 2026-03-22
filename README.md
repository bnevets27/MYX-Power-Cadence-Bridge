# MYX Power & Cadence Bridge — Home Assistant Edition

[![Release](https://img.shields.io/github/v/release/bnevets27/MYX-Power-Cadence-Bridge)](https://github.com/bnevets27/MYX-Power-Cadence-Bridge/releases/latest)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

An ESP32 firmware that bridges the **BODi MYX** bike's proprietary Bluetooth sensor to any standard cycling app **and** publishes live ride data to Home Assistant via MQTT.

The MYX bike's built-in sensor (`BKSNSR*`) XOR-masks its BLE data, making it unreadable by third-party apps. This firmware runs on an ESP32 dev board, decodes the data in real time, re-broadcasts it as a fully compliant BLE Cycling Power + Speed & Cadence sensor, and simultaneously streams metrics to your smart home.

> **Two versions are available** — choose the one that fits your setup:
>
> | | [Basic](https://github.com/bnevets27/MYX-Power-Cadence-Bridge/tree/main) | [**Home Assistant (this branch)**](https://github.com/bnevets27/MYX-Power-Cadence-Bridge/tree/ha-mqtt) |
> |---|---|---|
> | BLE bridge (Zwift, Wahoo, etc.) | ✅ | ✅ |
> | Home Assistant / MQTT | ❌ | ✅ |
> | Wi-Fi captive portal setup | ❌ | ✅ |
> | Live debug web UI | ❌ | ✅ |
> | OTA firmware updates | ❌ | ✅ |
> | Setup complexity | Plug & play | 5-min Wi-Fi setup |
>
> **Not sure?** If you don't use Home Assistant, use the [Basic version](https://github.com/bnevets27/MYX-Power-Cadence-Bridge/tree/main). Download either `.bin` files from the [Releases page](https://github.com/bnevets27/MYX-Power-Cadence-Bridge/releases/latest).

---

## Features

| Metric | BLE Apps | Home Assistant |
|--------|----------|----------------|
| Power (watts) | ✅ | ✅ |
| Cadence (RPM) | ✅ | ✅ |
| Battery level | ✅ | ✅ |
| Accumulated energy (kJ) | ✅ | ✅ |

- Auto-connects to any nearby MYX bike sensor on boot
- Supports multiple simultaneous BLE app connections (Zwift + phone simultaneously)
- **WiFiManager captive portal** — no hardcoded credentials, configure via phone on first boot
- **MQTT + Home Assistant auto-discovery** — sensors appear in HA automatically, no YAML needed
- **Live debug web UI** — view real-time sensor data, connection status, and heap usage from any browser
- **OTA firmware updates** — flash new firmware over Wi-Fi from the web UI, no USB cable needed after initial flash
- LED status indicator for BLE connection state
- Standard BLE CPS (0x1818) + CSC (0x1816) services

---

## Hardware

Any board built on the **original ESP32 chip** (dual-core Xtensa, with both Wi-Fi and BLE) works. Recommended:

- **ESP32-DevKitC** or **ESP32-WROOM-32** or **ESP32-S3** 
- USB-C or Micro USB data cable for the initial flash, then any USB power source

No soldering or extra components required.

> **Compatibility note:** 
> - **ESP32-S2** — no BLE, incompatible
> - **ESP32-C3 / C6** — not currently supported
>
> Minimum flash: **4MB** (standard on virtually all ESP32 dev boards).

---

## Initial Flashing

### Option 1 — Web Flasher (easiest, no software required)

1. Download `esp32-bike-bridge-ha-mqtt-v2.x.x.bin` from the [Releases page](https://github.com/bnevets27/MYX-Power-Cadence-Bridge/releases/latest)
2. Plug your ESP32 into your computer via USB
3. Open **[web.esphome.io](https://web.esphome.io)** in Chrome or Edge
4. Click **Install** → **Browse** → select the `.bin` file → select your COM port → Flash
5. Done — proceed to **First Boot Setup** below

> **Windows**: if no COM port appears you may need a USB driver — [CP2102](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) or [CH340](https://www.wch-ic.com/downloads/CH341SER_EXE.html) depending on your board.

### Option 2 — PlatformIO (for developers)

```bash
git clone -b ha-mqtt https://github.com/bnevets27/MYX-Power-Cadence-Bridge
cd MYX-Power-Cadence-Bridge
# Open in VS Code with PlatformIO extension, then click Upload
```

---

## First Boot Setup

On first boot the ESP32 has no Wi-Fi credentials and starts a **captive portal**:

1. On your phone or laptop, connect to the Wi-Fi network **`MYX-Bridge-Setup`**
2. A configuration page opens automatically (or navigate to `192.168.4.1`)
3. Enter your home Wi-Fi SSID and password
4. Enter your MQTT broker IP, port (default `1883`), and credentials (if required)
5. Click **Save** — the ESP32 reboots and connects

After setup, the device remembers its credentials. To reconfigure, hold the BOOT button for 5 seconds or use the **Reset Config** button in the web UI.

---

## Web UI

Once connected, open your browser to the ESP32's IP address (find it in your router's DHCP list or from the serial monitor):

| Page | URL | Description |
|------|-----|-------------|
| Dashboard | `http://<ip>/` | Live power, cadence, battery, BLE connections, heap |
| OTA Update | `http://<ip>/update` | Upload new firmware `.bin` over Wi-Fi |
| Reset Config | `http://<ip>/reset` | Wipe Wi-Fi + MQTT credentials and restart captive portal |

---

## Home Assistant

This firmware uses **MQTT Discovery** — no manual YAML configuration needed.

Once the ESP32 connects to your MQTT broker, the following entities appear automatically in Home Assistant under a device named **"MYX Bridge"**:

| Entity | Type | Unit | Notes |
|--------|------|------|-------|
| Power | Sensor | W | Instantaneous power output |
| Cadence | Sensor | RPM | Crank cadence |
| Battery | Sensor | % | Bike sensor battery level |
| Energy | Sensor | kJ | Accumulated energy for the session |
| Sensor Connected | Binary Sensor | — | Whether the bridge has an active BLE link to the bike sensor |

> **About Sensor Connected:** This reflects the BLE connection between the ESP32 bridge and the MYX bike sensor (`BKSNSR*`). **Disconnected is completely normal when you're not riding** — the bike sensor sleeps after a few seconds of no pedalling and drops the connection to save battery. Once you start pedalling again it wakes up and the bridge reconnects automatically within a few seconds. You don't need to restart anything.

Make sure MQTT discovery is enabled in your `configuration.yaml`:

```yaml
mqtt:
  discovery: true
  discovery_prefix: homeassistant
```

---

## OTA Firmware Updates

After the initial USB flash you can update over Wi-Fi:

1. Download the new `.bin` from the [Releases page](https://github.com/bnevets27/MYX-Power-Cadence-Bridge/releases/latest)
2. Open `http://<ip>/update` in your browser
3. Click **Choose File**, select the `.bin`, then click **Upload**
4. A progress bar shows upload %, then "Flashing...", then auto-redirects when done

> **First-time OTA note:** If upgrading from the Basic firmware (v1.x) or any pre-v2.0.3 HA firmware, you must do **one USB flash** first to burn the new dual-partition table. After that, all future updates can be done via OTA.

---

## Usage

1. Power on your MYX bike (pedal briefly to wake the sensor)
2. Plug the ESP32 into USB power and place it near the bike
3. Watch the LED — once it blinks fast, the sensor is connected
4. Open your cycling app, scan for Bluetooth sensors, and connect to **"MYX Bridge"**

The bridge appears as both a **Cycling Power Sensor** and a **Speed & Cadence Sensor** — connect to whichever your app prefers (or both).

### LED Status

| Pattern | State |
|---------|-------|
| Slow blink (1 s) | Scanning for bike sensor |
| Fast blink (250 ms) | Sensor found, waiting for app |
| Solid on | Fully active — sensor + app connected |

---

## How It Works

```
MYX Bike (BKSNSR*)          ESP32 Bridge                 Destinations
  BLE sensor        ──────►  XOR decode    ──────────►  Zwift / Wahoo / Garmin
  XOR 0xAA masked             re-broadcast  ──────────►  Home Assistant (MQTT)
                                             ──────────►  Web UI (browser)
```

The MYX sensor transmits standard BLE Cycling Power Service packets but XOR-masks every byte with `0xAA`. This makes the raw power reading appear as `-21846 W` to any app trying to read it directly. The bridge decodes each packet and re-broadcasts correct values over BLE, MQTT, and the web UI simultaneously.

### Decoded Packet Format

| Bytes | Field | Type | Notes |
|-------|-------|------|-------|
| 0–1 | Flags | uint16 LE | `0x0820` — crank data + energy present |
| 2–3 | Instantaneous Power | sint16 LE | Watts |
| 4–5 | Cumulative Crank Revolutions | uint16 LE | Used for cadence |
| 6–7 | Last Crank Event Time | uint16 LE | 1/1024 s resolution |
| 8–9 | Accumulated Energy | uint16 LE | kJ |

---

## Troubleshooting

**ESP32 can't find the sensor**
Make sure the bike sensor is powered on and not already connected to another device — BLE sensors typically allow only one client connection at a time.

**App can't find "MYX Bridge"**
Restart the app (some cache old device lists) and confirm the ESP32 LED is blinking fast. Re-scan for sensors.

**Power reads 0 while pedaling**
The sensor takes a couple of seconds to register movement. Check the web UI dashboard or serial monitor (115200 baud) — you should see live data updating.

**MQTT sensors not appearing in Home Assistant**
Confirm MQTT discovery is enabled, the broker IP/port are correct, and the ESP32 web UI shows "MQTT: Connected". Check the HA MQTT integration logs for incoming discovery messages.

**Multiple MYX bikes nearby**
The ESP32 connects to the first `BKSNSR*` device it finds. For home use this is never a problem. In a gym, position the ESP32 physically close to your bike.

**Need to change Wi-Fi or MQTT settings**
Use the **Reset Config** button at `http://<ip>/reset` or hold the BOOT button for 5 seconds. The device restarts as a captive portal.

---

## License

MIT — free to use, modify, and distribute.
