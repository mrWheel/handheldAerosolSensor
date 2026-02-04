# Low-Power Air Quality Monitor (SPS30 + E-Paper)

This project is a **battery-powered, low-power air quality monitor** based on:

- **Arduino Nano (ATmega328P)**
- **Sensirion SPS30** particulate matter sensor (I²C)
- **1.54" black/white e‑paper display (200×200)**
- **High-side power latch** using a P‑channel MOSFET + 2N7000 or BSS138
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

| Nano pin | ATmega328P | Standaard functie | AirosolSensor | Opmerking |
|-----------|---------|-----------|-----------------------|-------------------------------------|
| D2 | PD2 | INT0 | GPIO_PIN_LATCH | - |
| D3 | PD3 | INT1 / PWM | GPIO_PIN_SWITCH | - |
| D4 | PD4 | Digital | Vrij | - |
| D5 | PD5 | PWM | Vrij | - |
| D6 | PD6 | PWM | Vrij | - |
| D7 | PD7 | Digital | GPIO_PIN_EPD_BUSY | e-paper BUSY |
| D8 | PB0 | Digital | GPIO_PIN_EPD_RST | e-paper RST |
| D9 | PB1 | PWM | Gebruikt | e-paper DC |
| D10 | PB2 | SPI SS / PWM | GPIO_PIN_EPD_CS | e-paper CS |
| D11 | PB3 | SPI MOSI | GPIO_PIN_EPD_MOSI | Hardware SPI MOSI (vast voor e-paper) |
| D12 | PB4 | SPI MISO | GPIO_PIN_EPD_MISO | E-paper gebruikt vaak geen MISO; mag vrij blijven |
| D13 | PB5 | SPI SCK + LED | GPIO_PIN_EPD_SCK | SCK voor e-paper; LED_BUILTIN zit hier ook |
| A0 (D14)| PC0 | ADC0 | GPIO_PIN_BAT_ADC | Batterij/2 |
| A1 (D15)| PC1 | ADC1 | Vrij | - |
| A2 (D16)| PC2 | ADC2 | Vrij | - |
| A3 (D17)| PC3 | ADC3 | Vrij | - |
| A4 (D18)| PC4 | I²C SDA | GPIO_PIN_SPS30_SDA | SPS30 SDA |
| A5 (D19)| PC5 | I²C SCL | GPIO_PIN_SPS30_SCL | SPS30 SCL |
| A6 | ADC6 | Analog in | Vrij | Alleen analogRead, géén digital |
| A7 | ADC7 | Analog in | Vrij | Alleen analogRead, géén digital |



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
  - safeTimers

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

build_flags =
  -D GPIO_PIN_LATCH=2
  -D GPIO_PIN_SWITCH=3
  -D GPIO_PIN_BAT_ADC=A0
  -D GPIO_PIN_SPS30_SDA=A4
  -D GPIO_PIN_SPS30_SCL=A5 
  -D GPIO_PIN_EPD_MOSI=11
  -D GPIO_PIN_EPD_MISO=12
  -D GPIO_PIN_EPD_SCK=13
  -D GPIO_PIN_EPD_CS=10
  -D GPIO_PIN_EPD_DC=9
  -D GPIO_PIN_EPD_RST=8
  -D GPIO_PIN_EPD_BUSY=7 ; E-paper BUSY
  -D WARMUP_SECONDS=4
  -D MAX_METINGEN=3
  -D ADC_VREF_VOLTAGE=5.0
;  -D HAS_E_PAPER_DISPLAY

lib_deps =
  zinggjm/GxEPD2@^1.6.5
  adafruit/Adafruit GFX Library@^1.11.9
  https://github.com/Sensirion/arduino-sps.git
  https://github.com/mrWheel/safeTimers.git
  
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
