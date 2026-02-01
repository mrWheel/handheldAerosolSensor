# Low-Power Air Quality Monitor (SPS30 + E-Paper)

This project is a **battery-powered, low-power air quality monitor** based on:

- **Arduino Nano (ATmega328P)**
- **Sensirion SPS30** particulate matter sensor (I²C)
- **1.54" black/white e‑paper display (200×200)**
- **High-side power latch** using a P‑channel MOSFET + 2N7000
- **Single-cell Li‑ion (18650)** battery with DC/DC boost to 5 V

The system powers itself on with a button, performs a measurement cycle, updates the e‑paper display, and then **fully powers itself off** to achieve near-zero standby current.

---

## Features

- True hardware power-off (no sleep leakage)
- SPS30 warm-up with countdown on e‑paper
- Configurable number of measurement attempts
- **Only valid sensor samples are averaged**
- PM1.0 / PM2.5 / PM10 display
- Simple air-quality classification (PM2.5 based)
- Battery voltage and percentage display
- SRAM-safe e‑paper rendering (paged buffer, fits in 2 KB RAM)

---

## Measurement Flow

1. User presses power button
2. Power latch engages
3. E‑paper resets and shows status
4. SPS30 starts measurement
5. Warm-up delay (default: 15 s)
6. Up to `maxMetingen` measurement attempts
   - Only valid samples are accumulated
7. Average values are calculated
8. Results are shown on the e‑paper
9. SPS30 is stopped
10. Power latch is released (device turns fully off)

---

## Hardware Connections

### Arduino Nano

| Function | Pin |
|--------|-----|
| Power latch control | D4 |
| E‑paper CS | D10 |
| E‑paper DC | D9 |
| E‑paper RST | D8 |
| E‑paper BUSY | D7 |
| SPI MOSI | D11 |
| SPI SCK | D13 |
| Battery ADC | A0 |
| I²C SDA | A4 |
| I²C SCL | A5 |

### Battery Measurement

- 100 kΩ / 100 kΩ divider from battery to A0
- Optional 100 nF capacitor from A0 to GND

---

## Software Stack

- **PlatformIO**
- **Arduino framework**
- **Libraries**
  - GxEPD2
  - Adafruit GFX
  - Sensirion SPS30 (GitHub)

---

## PlatformIO Configuration

```ini
[env:nano]
platform = atmelavr
board = nanoatmega328
framework = arduino

upload_protocol = arduino
upload_speed = 57600   ; use 115200 for new bootloader
monitor_speed = 115200

lib_deps =
  zinggjm/GxEPD2@^1.6.5
  adafruit/Adafruit GFX Library@^1.11.9
  https://github.com/Sensirion/arduino-sps.git
```

---

## Power Latch Notes

- Upload firmware with **stable power** (USB directly)
- During normal operation the latch cuts power completely
- GPIO D4 must be set HIGH early in `setup()` to hold power

---

## Memory Considerations

The ATmega328P only has **2 KB SRAM**.

To fit the e‑paper buffer:
- GxEPD2 is configured in **paged mode**
- Page height ≈ 32 rows (~800 bytes buffer)

Do **not** use full-frame buffers on this MCU.

---

## Customization

You can easily adjust:

- `warmupSeconds` (e.g. 15 → 30)
- `maxMetingen` (e.g. 5 → 10)
- PM2.5 thresholds for status classification
- Display layout and fonts

---

## License

This project is intended for personal and educational use.
You are free to modify and adapt it for your own hardware.

---

## Notes

This design prioritizes:
- Measurement correctness
- Ultra-low standby power
- Robust behavior on battery power

It is well suited for **portable or infrequently-used air quality measurements** where long battery life matters.
