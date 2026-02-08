# Low-Power Air Quality Monitor (SPS30 + E-Paper)

## Disclaimer
This software is developed incrementally. That means I have no clear idea how it works (though it *mostly* does).

If you have questions about this software, it will probably take you just as long to figure things out as it would take me. So I’d prefer that you investigate it yourself.

Having said that **Don’t Even Think About Using It**

Seriously. Don’t.

Building this design may injure or kill you during construction, burn your house down while in use, and then—just to be thorough—explode afterward.

This is **not** a joke. This project involves **lethal voltages**. If you are not a **qualified electronics engineer**, close this repository, step away from the soldering iron, and make yourself a cup of tea.

If you decide to ignore all of the above and build it anyway, you do so **entirely at your own risk**. You are fully responsible for taking proper safety precautions. I take **zero responsibility** for anything that happens—electrically, mechanically, chemically, spiritually, or otherwise.

Also, full disclosure: I am **not** a qualified electrical engineer. I provide **no guarantees**, **no warranties**, and **absolutely no assurance** that this design is correct, safe, or suitable for any purpose whatsoever.

# What it is
This project is a **battery-powered, low-power air quality monitor** based on:

- **ESP32 DEVKIT V1**
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

### ESP32 DevKit V1 - Pin mapping for AerosolSensor

| ESP32 pin | DEVKIT V1 | Standaard functie              | AerosolSensor        | Opmerking |
|-----------|-----------|--------------------------------|----------------------|-----------|
| 25        | D25       | GPIO / DAC1                    | GPIO_PIN_LATCH       | Latch control (OUTPUT) |
| 33        | D33       | GPIO / ADC1_CH5 / Touch        | GPIO_PIN_SWITCH      | Switch input (INPUT_PULLUP), non-strapping |
| 34        | D34       | ADC1_CH6 (input-only)          | GPIO_PIN_BAT_ADC     | Battery ADC (input-only), ADC1 recommended |
| 21        | D21       | I2C SDA (typical)              | GPIO_PIN_SPS30_SDA   | SPS30 SDA |
| 22        | D22       | I2C SCL (typical)              | GPIO_PIN_SPS30_SCL   | SPS30 SCL |
| 16        | RX2       | Future Ext.                    | GPIO_PIN_SPS_UART_RX | Serial SPS module |
| 17        | TX2       | Future Ext.                    | GPIO_PIN_SPS_UART_TX | Serial SPS module | 
| 23        | D23       | VSPI MOSI (typical)            | GPIO_PIN_EPD_MOSI    | E-paper DIN (MOSI) |
| -1        | n.c.      | n.v.t.                         | GPIO_PIN_EPD_MISO    | Not connected on display |
| 18        | D18       | VSPI SCK (typical)             | GPIO_PIN_EPD_SCK     | E-paper CLK (SCK) |
| 27        | D27       | GPIO                           | GPIO_PIN_EPD_CS      | E-paper CS |
| 19        | D19       | e-Paper DC                     | GPIO_PIN_EPD_DC      | E-paper DC |
| 13        | D13       | e-Paper RST                    | GPIO_PIN_EPD_RST     | E-paper RST |
| 32        | D32       | GPIO / ADC1_CH4 / Touch        | GPIO_PIN_EPD_BUSY    | E-paper BUSY (INPUT_PULLUP), non-strapping |
| 14        | D14       | GPIO PWM Touch                 | GPIO_PIN_BUZZER_PWM  | Buzzer 



### Battery Measurement

- 100 kΩ / 100 kΩ divider from battery to D34
- Optional 100 nF capacitor from D34 to GND

---

## Software Stack

- **PlatformIO**
- **Arduino framework**
- **Libraries**
  - GxEPD2
  - Adafruit GFX
  - Adafruit BMP280 Library
  - Adafruit Unified Sensor
  - Sensirion SPS30 (GitHub)
  - safeTimers

---

## PlatformIO Configuration

```ini
; platformio.ini
;
;— ESP32 + Arduino framework
;— Pin choices avoid ESP32 strapping pins (GPIO0,2,4,5,12,15) and flash pins (GPIO6–11).
;— Battery ADC uses ADC1 (GPIO34) so it keeps working even if WiFi were enabled.
;— SPI uses VSPI defaults for ESP32: MOSI=23, MISO=19, SCK=18.

[platformio]
workspace_dir = .pio.nosync
default_envs = esp32dev

[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

monitor_speed = 115200
upload_speed = 921600

;-- do NOT use 0, 2, 4, 5, 12, 15 (Strapping pins) or 6–11 (Flash pins)
build_flags =
  ;==================== Power & user input ====================
  ;— Power latch control (OUTPUT)
  -D GPIO_PIN_LATCH=25

  ;— User pushbutton (INPUT_PULLUP, LOW = pressed)
  -D GPIO_PIN_SWITCH=33


  ;==================== Battery measurement ====================
  ;— Battery voltage divider ADC input (ADC1)
  -D GPIO_PIN_BAT_ADC=34


  ;==================== SPS air quality sensor ====================
  ;— SPS I2C interface
  -D GPIO_PIN_SPS_SDA=21
  -D GPIO_PIN_SPS_SCL=22

  ;— SPS UART interface (UART2, hardware default pins)
  -D GPIO_PIN_SPS_UART_TX=17
  -D GPIO_PIN_SPS_UART_RX=16


  ;==================== E-paper display (SPI) ====================
  ;— SPI data pins (VSPI defaults)
  -D GPIO_PIN_EPD_MOSI=23
  -D GPIO_PIN_EPD_MISO=-1
  -D GPIO_PIN_EPD_SCK=18

  ;— E-paper control pins
  -D GPIO_PIN_EPD_CS=27
  -D GPIO_PIN_EPD_DC=19
  -D GPIO_PIN_EPD_RST=13
  -D GPIO_PIN_EPD_BUSY=32


  ;==================== User feedback ====================
  ;— Piezo buzzer (PWM via LEDC)
  -D GPIO_PIN_BUZZER_PWM=14


  ;==================== Application configuration ====================
  -D WARMUP_SECONDS=5
  -D MAX_METINGEN=5


  ;==================== ADC calibration ====================
  -D ADC_VREF_VOLTAGE=3.30f
  
  ;==================== Battery calibration ====================
  -D BAT_CAL_GAIN=0.94245f
  -D BAT_CAL_OFFSET=0.48763f

  ;==================== Library configuration ====================
  -D DISABLE_DIAGNOSTIC_OUTPUT

lib_deps =
  zinggjm/GxEPD2@^1.6.5
  adafruit/Adafruit GFX Library@^1.11.9
  adafruit/Adafruit BMP280 Library
	adafruit/Adafruit Unified Sensor
  https://github.com/Sensirion/arduino-sps.git
  https://github.com/mrWheel/safeTimers.git
```

---

## Power Latch Notes

- Upload firmware with **stable power** (USB directly)
- During normal operation the latch cuts power completely
- GPIO D25 must be set HIGH early in `setup()` to hold power

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
