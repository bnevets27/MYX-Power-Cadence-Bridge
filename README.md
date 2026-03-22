# MYX Power & Cadence Bridge

[![Release](https://img.shields.io/github/v/release/bnevets27/MYX-Power-Cadence-Bridge)](https://github.com/bnevets27/MYX-Power-Cadence-Bridge/releases/latest)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

An ESP32 firmware that bridges the **BODi MYX** bike's proprietary Bluetooth sensor to any standard cycling app — Zwift, TrainerRoad, Wahoo, Garmin, Apple Fitness+, and more.

The MYX bike's built-in sensor (`BKSNSR*`) XOR-masks its BLE data, making it unreadable by third-party apps. This firmware runs on a ESP32 dev board, decodes the data in real time, and re-broadcasts it as a fully compliant BLE Cycling Power + Speed & Cadence sensor.

> **Two versions are available** — choose the one that fits your setup:
>
> | | [**Basic (this branch)**](https://github.com/bnevets27/MYX-Power-Cadence-Bridge/tree/main) | [Home Assistant Edition](https://github.com/bnevets27/MYX-Power-Cadence-Bridge/tree/ha-mqtt) |
> |---|---|---|
> | BLE bridge (Zwift, Wahoo, etc.) | ✅ | ✅ |
> | Home Assistant / MQTT | ❌ | ✅ |
> | Wi-Fi captive portal setup | ❌ | ✅ |
> | Live debug web UI | ❌ | ✅ |
> | OTA firmware updates | ❌ | ✅ |
> | Setup complexity | **Plug & play** | 5-min Wi-Fi setup |
>
> **Not sure?** If you don't use Home Assistant, stay here. Download both `.bin` files from the [Releases page](https://github.com/bnevets27/MYX-Power-Cadence-Bridge/releases/latest).

---

## Features

| Metric | Supported |
|--------|-----------|
| Power (watts) | ✅ |
| Cadence (RPM) | ✅ |

- Auto-connects to any nearby MYX bike sensor on boot — no configuration needed
- Supports multiple simultaneous app connections (Zwift + phone, etc.)
- LED status indicator for connection state
- Standard BLE CPS (0x1818) + CSC (0x1816) services — works with every major cycling app

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

## Flashing

### Option 1 — Web Flasher (easiest, no software required)

1. Download the latest [`firmware.bin`](https://github.com/bnevets27/MYX-Power-Cadence-Bridge/releases/latest) from the Releases page
2. Plug your ESP32 into your computer via USB
3. Open **[web.esphome.io](https://web.esphome.io)** in Chrome or Edge
4. Click **Install** → **Browse** → select `firmware.bin` → select your COM port → Flash
5. Done — unplug and power the ESP32 near your bike

> **Windows**: if no COM port appears you may need a USB driver — [CP2102](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) or [CH340](https://www.wch-ic.com/downloads/CH341SER_EXE.html) depending on your board.

### Option 2 — PlatformIO (for developers)

```bash
git clone https://github.com/bnevets27/MYX-Power-Cadence-Bridge
cd MYX-Power-Cadence-Bridge
# Open in VS Code with PlatformIO extension, then click Upload
```

---

## Usage

1. Power on your MYX bike (wake the sensor by pedaling briefly)
2. Plug the ESP32 into USB power and place it near the bike
3. Watch the LED — once it blinks fast, the sensor is connected
4. Open your app, scan for Bluetooth sensors, and connect to **"MYX Bridge"**

The bridge appears as both a **Cycling Power Sensor** (power + cadence) and a **Speed & Cadence Sensor** (cadence) — connect to whichever your app prefers.

### LED Status

| Pattern | State |
|---------|-------|
| Slow blink (1 s) | Scanning for bike sensor |
| Fast blink (250 ms) | Sensor found, waiting for app |
| Solid on | Fully active — sensor + app connected |

---

## How It Works

```
MYX Bike (BKSNSR*)          ESP32 Bridge             Your App
  BLE sensor        ──────►  XOR decode    ──────►  Zwift / Wahoo
  XOR 0xAA masked             re-broadcast           standard BLE
```

The MYX sensor transmits standard BLE Cycling Power Service packets but XOR-masks every byte with `0xAA`. This makes the raw power reading appear as `-21846 W` to any app trying to read it directly. The bridge decodes each packet and re-broadcasts it with the correct values.

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
Restart the app (some cache old device lists) and confirm the ESP32 LED is blinking. Re-scan for sensors.

**Power reads 0 while pedaling**
The sensor takes a couple of seconds to register movement. Check the serial monitor (115200 baud) — you should see `[DATA]` lines when data is flowing.

**Multiple MYX bikes nearby**
The ESP32 connects to the first `BKSNSR*` device found during its scan. For home use this is never a problem. In a gym, position the ESP32 physically close to your bike.

---

## License

MIT — free to use, modify, and distribute.

