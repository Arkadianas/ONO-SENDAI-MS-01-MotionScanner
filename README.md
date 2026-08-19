# ONO-SENDAI MS-01 — MotionScanner

A handheld wireless device scanner built on ESP32-S3. Detects nearby WiFi devices — phones, routers, laptops — and displays them as dots on a real-time radar display. Inspired by cyberpunk instrumentation aesthetics.

![MotionScanner Display](docs/display.jpg)

---

## What It Does

MotionScanner passively scans the 2.4GHz WiFi spectrum and visualises nearby wireless devices on a radar-style display. It captures 802.11 management frames (probe requests) sent by phones, tablets, and laptops looking for known WiFi networks, as well as beacon frames from access points. Detected devices appear as labelled dots on the radar, with distance estimates derived from RSSI signal strength.

The device also tracks heading via an onboard IMU — rotate the scanner and the radar field rotates with you, keeping world-space device positions stable relative to your orientation.

---

## Hardware

| Component | Specification |
|---|---|
| **Board** | Waveshare ESP32-S3-Touch-AMOLED-1.64 |
| **MCU** | ESP32-S3, dual-core Xtensa LX7 @ 240MHz |
| **Display** | 1.64" AMOLED, 280×456px, SH8601 driver |
| **Touch** | FT3168 capacitive touch controller |
| **IMU** | QMI8658 6-axis (accelerometer + gyroscope) |
| **RAM** | 512KB SRAM + 8MB PSRAM |
| **Flash** | 8MB |
| **Radio** | WiFi 802.11 b/g/n (2.4GHz), Bluetooth 5.0 |
| **Power** | LiPo battery via JST connector |
| **Firmware** | ESP-IDF v5.5.x, LVGL 8.3.11 |

---

## Display Layout

```
┌─────────────────────────────┐
│ ONO-SENDAI   MS-01      USB │  ← Header + charging status
├─────────────────────────────┤
│                             │
│   · MD2          · RT1      │  ← Radar with labelled dots
│        ·   [+]              │
│      MD1                    │
│              · MD3          │
│                             │
├─────────────────────────────┤
│ MD1  FE:0A:39:E7:67  -82  4m│  ← Device list (scrollable)
│ MD2  A4:B2:C1:D3:12  -75  2m│
│ RT1  BC:EE:7B:44:21  -68  1m│
├─────────────────────────────┤
│ HDG 045°    TGT 03    ●●●○  │  ← Status bar
└─────────────────────────────┘
```

### Radar Rings
Four concentric rings represent distance zones. Inner ring = closest, outer ring = furthest (up to ~15m). A rotating sweep line scans continuously.

### Device Labels
Each detected device gets a short label based on its type:

| Label | Device Type |
|---|---|
| `MD1`, `MD2` | Mobile device / phone (WiFi probe request) |
| `RT1`, `RT2` | Router / Access Point |
| `PC1`, `PC2` | Laptop or desktop PC |
| `HP1`, `HP2` | Bluetooth headphones / audio device |
| `WR1` | Wearable (smartwatch, fitness band) |
| `CM1` | Suspected camera |
| `IOT1` | Generic IoT / BLE device |

### Status Bar
- **HDG 045°** — current heading in degrees (relative to boot orientation)
- **TGT 03** — number of actively tracked devices
- **●●●○** — signal activity bar
- **USB** (green) — charging via USB
- **BAT** (orange) — running on battery

---

## How To Use

### Basic Operation
1. Power on via battery connector
2. Device boots and begins scanning immediately
3. Walk around holding the scanner — dots appear as devices are detected
4. Rotate the scanner — the heading updates and dots shift to maintain world-space positions

### Tap To Inspect
Tap any dot on the radar to open a detail panel showing:
- Device short name and type
- Full MAC address
- RSSI signal strength (dBm)
- Estimated distance (metres)
- Bearing angle
- Confidence percentage
- Time since last seen

Tap `[X]` or anywhere outside the panel to close.

### Power Button
- **Hold BOOT button 3 seconds** → "POWERING OFF" appears, device enters deep sleep
- **Press RST button** → device wakes and restarts

### Heading Reference
On boot, the current facing direction is set as 0°. Rotate the device and the HDG value updates. All dot positions are maintained in world coordinates — a device to your north stays at the top of the radar regardless of which way you face.

---

## Architecture

```
WiFi Promiscuous Mode
        │
        ▼
Localization Engine          IMU (QMI8658)
(RSSI smoothing,              │
 bearing histogram,           ▼
 distance estimate)      Heading update
        │                     │
        └──────────┬──────────┘
                   ▼
            Radar Engine
         (coordinate transform,
          target tracking,
          position smoothing)
                   │
                   ▼
           Radar Renderer
         (LVGL dot placement,
          sweep animation,
          dot labels)
                   │
                   ▼
          AMOLED Display
```

### Key Components

**`radar_engine`** — Maintains tracked target list, applies IMU heading transform, smooths positions using exponential moving average. Never draws anything.

**`radar_renderer`** — Draws everything: rings, crosshair, dots, sweep line, dot labels. Never computes anything.

**`localization`** — RSSI history smoothing, bearing histogram, distance estimation using log-distance path loss model.

**`device_classification`** — Classifies devices by OUI (MAC prefix), BLE appearance codes, service UUIDs, and name keywords.

---

## Building

### Requirements
- ESP-IDF v5.5.x
- Waveshare ESP32-S3-Touch-AMOLED-1.64 board
- LVGL 8.3.11 (included as managed component)

### Build Steps
```bash
git clone https://github.com/Arkadianas/MotionScanner
cd MotionScanner
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

### File Placement
The modified source files in this repository replace files in the following locations of the full project:

| Repository Path | Project Location |
|---|---|
| `main/main.c` | `main/main.c` |
| `main/CMakeLists.txt` | `main/CMakeLists.txt` |
| `components/display_ui/*` | `components/display_ui/*` |
| `components/localization/*` | `components/localization/*` |
| `components/device_classification/*` | `components/device_classification/*` |

---

## Limitations

### WiFi Detection
- **Randomised MACs** — modern phones use randomised MAC addresses for probe requests. The same phone may appear as multiple different devices over time as its MAC rotates. Short-term detection works; long-term identity tracking does not.
- **Range** — practical indoor detection range is approximately 10–15 metres depending on walls and interference.
- **Privacy-aware devices** — some devices send very few probe requests when already connected to a network.

### Distance Estimation
- RSSI-based distance is an estimate only. Walls, reflections, and antenna orientation all affect accuracy significantly.
- Calibrated for `rssi0 = -70 dBm` at 1 metre with internal antenna, `n = 2.0` indoor path loss exponent. These values can be tuned in `localization.c`.

### Bearing / Direction
- Bearing is estimated from a histogram of RSSI readings taken from multiple angles. **You must walk around a device** to build an accurate bearing estimate. Standing still gives low confidence directional data.
- Heading is gyroscope-integrated — it accumulates drift over time. Press RST to reset heading reference.

### Battery
- No battery percentage available — the board does not have a voltage divider wired to any GPIO.
- Charging status (USB/BAT) is detected via the USB Serial JTAG peripheral.

### BLE vs WiFi
- This build uses WiFi scanning only. BLE and WiFi cannot run simultaneously on ESP32-S3 without RF coexistence configuration due to shared RF hardware.
- WiFi scanning is more effective for detecting phones than BLE in most environments.

---

## Technical Specifications

| Parameter | Value |
|---|---|
| Scan mode | WiFi promiscuous + active AP scan |
| Update rate | 5 Hz (device feed to radar engine) |
| IMU sample rate | 100 Hz |
| Display refresh | 20 Hz |
| Max tracked devices | 32 |
| Stale timeout | 15 seconds |
| Max radar range | 15 metres |
| RSSI calibration | rssi0 = -70 dBm, n = 2.0 |
| Heading drift | ~2–5°/min (gyro integration) |
| Power consumption | ~150mA active scanning |
| Deep sleep current | < 1mA |

---

## Display Aesthetics

The UI is inspired by ONO-SENDAI terminal interfaces from William Gibson's Neuromancer — industrial instrumentation with a cyberpunk sensibility. Black background, thin orange graphics, minimal chrome. No gradients, no shadows, no unnecessary animations. Everything on screen has a function.

---

## License

MIT License — see LICENSE file.

---

## Credits

Built with:
- [ESP-IDF](https://github.com/espressif/esp-idf) v5.5.x
- [LVGL](https://lvgl.io) 8.3.11
- [NimBLE](https://github.com/apache/mynewt-nimble) (Espressif port)
- Waveshare ESP32-S3-Touch-AMOLED-1.64 BSP
