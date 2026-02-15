/*** Last Changed: 2026-02-11 - 14:19 ***/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// — ESP32 radio control to disable WiFi/Bluetooth as early as possible
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_bt.h>

// — Display
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>

// — Sensors
#include <sps30.h>
#include <sensirion_i2c.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// — Timing
#include <safeTimers.h>

// — Program version string (keep manually updated with each release)
// — NEVER CHANGE THIS const char* NAME
// —             vvvvvvvvvvvvvv
static const char* PROG_VERSION = "v0.3.7";
// —             ^^^^^^^^^^^^^^

// ===================== User configuration (from build_flags) =====================

// — Latch control pin (drives the 2N7000 gate through a series resistor)
static const uint8_t pinLatch = GPIO_PIN_LATCH;

// — Pushbutton pin (LOW = pressed with INPUT_PULLUP)
static const uint8_t pinSwitch = GPIO_PIN_SWITCH;

// — Battery divider ADC input (default math assumes 2:1 divider => ADC sees Vbat/2)
static const uint8_t pinBatAdc = GPIO_PIN_BAT_ADC;

// — E-paper SPI pins (from build_flags)
static const int8_t pinEpdMosi = GPIO_PIN_EPD_MOSI;
static const int8_t pinEpdMiso = GPIO_PIN_EPD_MISO;
static const int8_t pinEpdSck = GPIO_PIN_EPD_SCK;

// — E-paper control pins
static const uint8_t pinEpdCs = GPIO_PIN_EPD_CS;
static const uint8_t pinEpdDc = GPIO_PIN_EPD_DC;
static const uint8_t pinEpdRst = GPIO_PIN_EPD_RST;
static const uint8_t pinEpdBusy = GPIO_PIN_EPD_BUSY;

// — SPS I2C pins (also used for BMP280)
static const uint8_t pinSpsSda = GPIO_PIN_SPS30_SDA;
static const uint8_t pinSpsScl = GPIO_PIN_SPS_SCL;

// — SPS UART pins (UART2)
static const int8_t pinSpsUartTx = GPIO_PIN_SPS_UART_TX;
static const int8_t pinSpsUartRx = GPIO_PIN_SPS_UART_RX;

static HardwareSerial spsUart(2);

// — BMP280 instance
static Adafruit_BMP280 bmp;
static bool bmpOk = false;

// — Warm-up time after starting SPS measurement
static const uint16_t warmupSeconds = WARMUP_SECONDS;

// — Maximum number of measurement attempts (only valid reads are averaged)
static const uint8_t maxMetingen = MAX_METINGEN;

// — ADC reference voltage used for battery math (calibrate for best accuracy)
static const float adcVref = (float)ADC_VREF_VOLTAGE;

// ===================== Timing (safeTimers) =====================

// — UI + sampling cadence
static const uint32_t warmupTickMs = 1000UL;
static const uint32_t sampleIntervalMs = 2500UL;
static const uint32_t spsAttemptTimeoutMs = 7000UL;
static const uint32_t spsPollIntervalMs = 100UL;

// — Switch handling
static const uint32_t switchDebounceMs = 30UL;
static const uint32_t switchLongPressMs = 2000UL;

// — safeTimers declarations
DECLARE_TIMER_MS(tWarmupTick, warmupTickMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tSampleInterval, sampleIntervalMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tAttemptTimeout, spsAttemptTimeoutMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tSpsPoll, spsPollIntervalMs, SKIP_MISSED_TICKS);

DECLARE_TIMER_MS(tSwitchDebounce, switchDebounceMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tLongPress, switchLongPressMs, SKIP_MISSED_TICKS);

// ===================== Display selection =====================

// — 1.54" 200x200 B/W display; full height buffer is fine on ESP32
static const uint16_t pageHeight = 200;

// — Display driver instance
GxEPD2_BW<GxEPD2_154, pageHeight> display(
    GxEPD2_154(pinEpdCs, pinEpdDc, pinEpdRst, pinEpdBusy));

// ===================== Fixed layout coordinates =====================

struct UiLayout
{
  int16_t xText;

  int16_t xPmCol1;
  int16_t xPmCol2;
  int16_t xPmCol3;

  int16_t yBattery;
  int16_t yEnv;

  int16_t yPmHeader;
  int16_t yPmValues;

  int16_t yMessage;
};

// — Layout tuned for 200x200, rotation=1
// — 9pt for small lines + PM header
// — 12pt for PM values
static const UiLayout ui =
{
  4,    // xText

  4,    // PM column 1
  70,   // PM column 2
  136,  // PM column 3

  26,   // yBattery (9pt) -26-
  52,   // yEnv (9pt) -48-

  90,   // yPmHeader (9pt) -72-
  128,  // yPmValues (12pt) -110-

  184   // yMessage (9pt) -186-
};

// — UI text fields
static String batteryText;
static String envText;
static String pm1Text;
static String pm25Text;
static String pm10Text;
static String messageText;

// ===================== Text helpers =====================

static String truncateToChars(const String& s, uint8_t maxChars)
{
  if (s.length() <= maxChars)
  {
    return s;
  }

  return s.substring(0, maxChars);

} //   truncateToChars()

// ===================== Air quality classification =====================

enum AirStatus
{
  STATUS_GEMIDDELD,
  STATUS_HOOG,
  STATUS_EXTREEM
};

static AirStatus classifyPm25(float pm25)
{
  if (pm25 < 35.0f)
  {
    return STATUS_GEMIDDELD;
  }

  if (pm25 < 75.0f)
  {
    return STATUS_HOOG;
  }

  return STATUS_EXTREEM;

} //   classifyPm25()

static const char* statusText(AirStatus s)
{
  switch (s)
  {
  case STATUS_GEMIDDELD:
    return "GEMIDDELD";
  case STATUS_HOOG:
    return "HOOG";
  default:
    return "EXTREEM";
  }

} //   statusText()

// ===================== ESP32 radio shutoff =====================

static void disableRadiosAsap()
{
  // — Ensure Arduino WiFi is off
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);

  // — Stop WiFi driver (ignore errors if not started yet)
  (void)esp_wifi_stop();
  (void)esp_wifi_deinit();

  // — Stop Bluetooth controller if present
  btStop();
  (void)esp_bt_controller_disable();
  (void)esp_bt_controller_deinit();

} //   disableRadiosAsap()

// ===================== I2C helpers =====================

static void i2cInit()
{
  // — Initialize I2C once, early (shared by BMP and SPS)
  Wire.begin(pinSpsSda, pinSpsScl);
  delay(10);
  Wire.setClock(100000);

  // — Prevent rare deadlocks on ESP32 if a device holds the bus
  Wire.setTimeOut(50);

  // — Bind Sensirion SPS30 driver to Wire
  sensirion_i2c_init(Wire);

  Serial.printf("I2C init: SDA=%u SCL=%u\n",
                (unsigned)pinSpsSda,
                (unsigned)pinSpsScl);

} //   i2cInit()

static void i2cScan()
{
  Serial.printf("I2C scan start\n");

  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    uint8_t err = (uint8_t)Wire.endTransmission(true);

    if (err == 0)
    {
      Serial.printf("I2C device found at 0x%02X\n", (unsigned)addr);
      found++;
    }

    delay(2);
    yield();
  }

  Serial.printf("I2C scan done (%u devices)\n", (unsigned)found);

} //   i2cScan()

// ===================== UART passthrough =====================

static void spsUartPassthroughInit()
{
  // — SPS UART: 115200 8N1
  spsUart.begin(115200, SERIAL_8N1, pinSpsUartRx, pinSpsUartTx);

  Serial.printf("SPS UART passthrough on UART2: RX=%d TX=%d\n",
                (int)pinSpsUartRx, (int)pinSpsUartTx);

} //   spsUartPassthroughInit()

static void spsUartPassthroughLoop()
{
  while (spsUart.available() > 0)
  {
    int c = spsUart.read();
    Serial.write((uint8_t)c);
  }

} //   spsUartPassthroughLoop()

// ===================== Battery reading =====================

#ifndef BAT_CAL_GAIN
//--Battery calibration gain (default: no correction)
#define BAT_CAL_GAIN 1.0f
#endif

#ifndef BAT_CAL_OFFSET
//--Battery calibration offset in volts (default: no correction)
#define BAT_CAL_OFFSET 0.0f
#endif

static float readBatteryVoltage()
{
  //--Read calibrated ADC voltage in millivolts (ESP32 eFuse based)
  uint32_t mv = (uint32_t)analogReadMilliVolts(pinBatAdc);
  float vAdc = (float)mv / 1000.0f;

  //--Default assumes a 2:1 divider (ADC sees Vbat/2)
  float vBatRaw = vAdc * 2.0f;

  //--Apply calibration (derived from measured points)
  float vBatCal = (vBatRaw * (float)BAT_CAL_GAIN) + (float)BAT_CAL_OFFSET;

  return vBatCal;

} //   readBatteryVoltage()

static uint8_t batteryPercentFromVoltage(float vbat)
{
  //--Typical 1S Li-ion (18650) voltage-to-SOC curve (approx.)
  //--Note: Under load the voltage sag can make SOC appear lower.
  struct Vp
  {
    float v;
    uint8_t p;
  };

  static const Vp curve[] =
      {
          {4.20f, 100},
          {4.15f, 95},
          {4.11f, 90},
          {4.08f, 85},
          {4.02f, 80},
          {3.98f, 75},
          {3.95f, 70},
          {3.91f, 65},
          {3.87f, 60},
          {3.85f, 55},
          {3.82f, 50},
          {3.79f, 45},
          {3.77f, 40},
          {3.74f, 35},
          {3.72f, 30},
          {3.70f, 25},
          {3.68f, 20},
          {3.65f, 15},
          {3.62f, 10},
          {3.58f, 5},
          {3.50f, 0}};

  //--Clamp above max
  if (vbat >= curve[0].v)
  {
    return 100;
  }

  //--Clamp below min
  const uint8_t last = (uint8_t)(sizeof(curve) / sizeof(curve[0]) - 1);
  if (vbat <= curve[last].v)
  {
    return 0;
  }

  //--Find segment and interpolate
  for (uint8_t i = 0; i < last; i++)
  {
    float vHigh = curve[i].v;
    float vLow = curve[i + 1].v;

    if (vbat <= vHigh && vbat >= vLow)
    {
      uint8_t pHigh = curve[i].p;
      uint8_t pLow = curve[i + 1].p;

      float t = (vbat - vLow) / (vHigh - vLow);
      float p = (float)pLow + t * ((float)pHigh - (float)pLow);

      if (p < 0.0f)
      {
        p = 0.0f;
      }

      if (p > 100.0f)
      {
        p = 100.0f;
      }

      return (uint8_t)(p + 0.5f);
    }
  }

  //--Should never happen
  return 0;

} //   batteryPercentFromVoltage()

static String formatBatteryLine(float vbat, uint8_t pct)
{
  // — Example: "4.02V 78%"
  char buf[48];
  snprintf(buf, sizeof(buf), "%.2fV %u%%", vbat, pct);
  return String(buf);

} //   formatBatteryLine()

static void updateBatteryNow(const char* reason)
{
  // — Read and render battery line immediately
  float vbat = readBatteryVoltage();
  uint8_t pct = batteryPercentFromVoltage(vbat);

  batteryText = formatBatteryLine(vbat, pct);
  batteryText = truncateToChars(batteryText, 12);

  Serial.printf("Battery (%s): %.2f V (%u%%)\n", reason, vbat, pct);

} //   updateBatteryNow()

// ===================== BMP280 helpers =====================

static bool bmpInit()
{
  // — BMP280 can be at 0x76 or 0x77; your chip responds at 0x76
  if (!bmp.begin(0x76))
  {
    if (!bmp.begin(0x77))
    {
      return false;
    }
  }

  // — Reasonable default config (fast enough, stable)
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,  // temp
                  Adafruit_BMP280::SAMPLING_X16, // pressure
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_125);

  return true;

} //   bmpInit()

static void updateEnvNow(const char* reason)
{
  // — Format: "21.3C 1013h" (no RH on BMP280)
  if (!bmpOk)
  {
    envText = "-";
    return;
  }

  float tC = bmp.readTemperature();
  float pHpa = bmp.readPressure() / 100.0f;

  if (!isfinite(tC) || !isfinite(pHpa))
  {
    envText = "-";
    return;
  }

  char buf[24];
  snprintf(buf, sizeof(buf), "%.1fC %.0fhPa", tC, pHpa);

  envText = truncateToChars(String(buf), 18);

  Serial.printf("BMP (%s): T=%.1f*C P=%.0fhPa\n", reason, tC, pHpa);

} //   updateEnvNow()

// ===================== E-paper helpers =====================

static void epdInit()
{
  // — Configure control pins
  pinMode(pinEpdCs, OUTPUT);
  digitalWrite(pinEpdCs, HIGH);

  pinMode(pinEpdDc, OUTPUT);
  digitalWrite(pinEpdDc, LOW);

  pinMode(pinEpdRst, OUTPUT);
  digitalWrite(pinEpdRst, HIGH);

  // — BUSY input with pull-up (board is open drain)
  pinMode(pinEpdBusy, INPUT_PULLUP);

  // — Map SPI pins explicitly (MISO may be -1)
  SPI.begin(pinEpdSck, pinEpdMiso, pinEpdMosi, pinEpdCs);

  // — Manual reset pulse (some boards need this)
  digitalWrite(pinEpdRst, LOW);
  delay(20);
  digitalWrite(pinEpdRst, HIGH);
  delay(50);

  Serial.printf("EPD BUSY before init: %d\n", digitalRead(pinEpdBusy));

  // — Init display (GxEPD2 controls SPI transactions internally)
  display.init(115200, true, 2, false);

  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);
  display.setFullWindow();

} //   epdInit()


static void uiUpdateLinePartial9pt(int16_t yBaseline, const String& text)
{
  // — Clear a full-width band and print one line in 9pt
  const int16_t bandHeight = 22;

  int16_t x = 0;
  int16_t y = yBaseline - bandHeight + 2;
  int16_t w = display.width();
  int16_t h = bandHeight;

  if (y < 0)
  {
    y = 0;
  }

  if ((y + h) > display.height())
  {
    h = display.height() - y;
  }

  display.setPartialWindow(x, y, w, h);

  display.firstPage();
  do
  {
    display.fillRect(x, y, w, h, GxEPD_WHITE);

    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(ui.xText, yBaseline);
    display.print(text);

  } while (display.nextPage());

} //   uiUpdateLinePartial9pt()

static void uiUpdatePmValuesPartial12pt()
{
  // — Clear band for PM values (12pt)
  const int16_t bandHeight = 36;

  int16_t x = 0;
  int16_t y = ui.yPmValues - bandHeight + 4;
  int16_t w = display.width();
  int16_t h = bandHeight;

  if (y < 0)
  {
    y = 0;
  }

  if ((y + h) > display.height())
  {
    h = display.height() - y;
  }

  display.setPartialWindow(x, y, w, h);

  display.firstPage();
  do
  {
    display.fillRect(x, y, w, h, GxEPD_WHITE);

    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMonoBold12pt7b);

    display.setCursor(ui.xPmCol1, ui.yPmValues);
    display.print(pm1Text);

    display.setCursor(ui.xPmCol2, ui.yPmValues);
    display.print(pm25Text);

    display.setCursor(ui.xPmCol3, ui.yPmValues);
    display.print(pm10Text);

  } while (display.nextPage());

} //   uiUpdatePmValuesPartial12pt()

static void uiDrawAllFull()
{
  display.setFullWindow();

  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);

    // — Battery line (9pt)
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(ui.xText, ui.yBattery);
    display.print(String("Bat: ") + batteryText);

    // — BME line (9pt)
    display.setCursor(ui.xText, ui.yEnv);
    display.print(String("T: ") + envText);

    // — PM header (9pt)
    display.setFont(&FreeMonoBold9pt7b);

    display.setCursor(ui.xPmCol1, ui.yPmHeader);
    display.print("PM1");

    display.setCursor(ui.xPmCol2, ui.yPmHeader);
    display.print("PM2.5");

    display.setCursor(ui.xPmCol3, ui.yPmHeader);
    display.print("PM10");

    // — PM values (12pt)
    display.setFont(&FreeMonoBold12pt7b);

    display.setCursor(ui.xPmCol1, ui.yPmValues);
    display.print(pm1Text);

    display.setCursor(ui.xPmCol2, ui.yPmValues);
    display.print(pm25Text);

    display.setCursor(ui.xPmCol3, ui.yPmValues);
    display.print(pm10Text);

    // — Status/message (9pt)
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(ui.xText, ui.yMessage);
    display.print(messageText);

  } while (display.nextPage());

} //   uiDrawAllFull()

static void uiUpdateBatteryPartial()
{
  uiUpdateLinePartial9pt(ui.yBattery, String("Bat: ") + batteryText);

} //   uiUpdateBatteryPartial()

static void uiUpdateEnvPartial()
{
  uiUpdateLinePartial9pt(ui.yEnv, String("T: ") + envText);

} //   uiUpdateEnvPartial()

static void uiUpdatePmPartial()
{
  uiUpdatePmValuesPartial12pt();

} //   uiUpdatePmPartial()

static void uiUpdateMessagePartial()
{
  uiUpdateLinePartial9pt(ui.yMessage, messageText);

} //   uiUpdateMessagePartial()

static void uiUpdateDynamicAll()
{
  uiUpdateBatteryPartial();
  uiUpdateEnvPartial();
  uiUpdatePmPartial();
  uiUpdateMessagePartial();

} //   uiUpdateDynamicAll()

// ===================== SPS helpers =====================

static bool spsInitAndStart()
{
  // — Probe sensor (Sensirion driver bound in i2cInit())
  if (sps30_probe() != 0)
  {
    return false;
  }

  // — Start measurement mode
  if (sps30_start_measurement() != 0)
  {
    return false;
  }

  return true;

} //   spsInitAndStart()

static void spsStop()
{
  // — Stop measurement to reduce power
  (void)sps30_stop_measurement();

} //   spsStop()

static bool isValidSample(float pm1, float pm25, float pm10)
{
  // — Reject NaN/Inf
  if (!isfinite(pm1) || !isfinite(pm25) || !isfinite(pm10))
  {
    return false;
  }

  // — Reject negative values
  if (pm1 < 0.0f || pm25 < 0.0f || pm10 < 0.0f)
  {
    return false;
  }

  // — Reject obviously out-of-range values
  if (pm1 > 2000.0f || pm25 > 2000.0f || pm10 > 2000.0f)
  {
    return false;
  }

  return true;

} //   isValidSample()

static String formatPmLine(float v)
{
  // — Format PM value as "123.4"
  char buf[16];
  snprintf(buf, sizeof(buf), "%.1f", v);
  return String(buf);

} //   formatPmLine()

static String formatAvgLine(float v, const char* suffix)
{
  // — Format average line as "123.4 <suffix>" (e.g. "12.3 AVG")
  char buf[24];
  snprintf(buf, sizeof(buf), "%.1f %s", v, suffix);
  return String(buf);

} //   formatAvgLine()

// — Play a two-tone beep on the buzzer (blocking)
static void playTwoToneBeep(uint16_t firstHz, uint16_t firstToneMs, uint16_t secondHz, uint16_t secondToneMs)
{
  // — First tone (start)
  tone((uint8_t)GPIO_PIN_BUZZER_PWM, (unsigned int)firstHz);
  delay((unsigned long)firstToneMs);

  // — Stop
  noTone((uint8_t)GPIO_PIN_BUZZER_PWM);

  // — Second tone (start)
  tone((uint8_t)GPIO_PIN_BUZZER_PWM, (unsigned int)secondHz);
  delay((unsigned long)secondToneMs);

  // — Stop
  noTone((uint8_t)GPIO_PIN_BUZZER_PWM);
  digitalWrite((uint8_t)GPIO_PIN_BUZZER_PWM, LOW); // Ensure buzzer pin is HIGH when not active

} //   playTwoToneBeep()

// — Beep when latch is fixed (low -> high)
static void beepLatchFixed(void)
{
  // — Low-to-high confirmation beep
  playTwoToneBeep(700, 200, 1500, 300);

} //   beepLatchFixed()

// — Beep before latch disable during switchoff (high -> low)
static void beepBeforeLatchDisable(void)
{
  // — High-to-low shutdown beep
  playTwoToneBeep(1500, 200, 700, 300);

} //   beepBeforeLatchDisable()

// ===================== Power off =====================

static void switchOff()
{
  // — Update battery and env readings right before unlatching power
  updateBatteryNow("pre-off");
  updateEnvNow("pre-off");

  uiUpdateBatteryPartial();
  uiUpdateEnvPartial();

  // — Show message before power-down
  messageText = " ";
  messageText = truncateToChars(messageText, 18);
  uiUpdateMessagePartial();

  beepBeforeLatchDisable();

  // — Log and flush
  Serial.printf("Switching off latch (PIN %u)\n", (unsigned)pinLatch);
  Serial.flush();

  // — Release the latch (hardware should cut power)
  digitalWrite(pinLatch, LOW);

  // — Fallback loop if power does not drop (debug aid)
  while (true)
  {
    delay(100);
  }

} //   switchOff()

// ===================== Non-blocking state machine =====================

enum MainState
{
  STATE_WARMUP,
  STATE_MEASURE_START_ATTEMPT,
  STATE_MEASURE_POLL_READY,
  STATE_MEASURE_WAIT_NEXT,
  STATE_SHOW_RESULTS,
  STATE_ERROR,
  STATE_POWER_DOWN
};

static MainState mainState = STATE_WARMUP;

// — Warmup remaining seconds
static uint16_t warmupRemaining = 0;

// — Measurement accumulation
static float sumPm1 = 0.0f;
static float sumPm25 = 0.0f;
static float sumPm10 = 0.0f;
static uint8_t validCount = 0;
static uint8_t attemptIndex = 0;

// — Results data
static float avgPm1 = 0.0f;
static float avgPm25 = 0.0f;
static float avgPm10 = 0.0f;

// — Switch tracking
static bool switchStableState = HIGH;
static bool switchLastReading = HIGH;

static void updateSwitch()
{
  // — Read switch with pull-up logic (LOW means pressed)
  bool reading = (digitalRead(pinSwitch) == LOW) ? LOW : HIGH;

  // — On edge change, restart debounce timer
  if (reading != switchLastReading)
  {
    switchLastReading = reading;
    RESTART_TIMER(tSwitchDebounce);
  }

  // — When debounce expires, accept new stable state
  if (DUE(tSwitchDebounce))
  {
    if (switchStableState != switchLastReading)
    {
      switchStableState = switchLastReading;

      if (switchStableState == LOW)
      {
        // — Press: start long-press timer
        RESTART_TIMER(tLongPress);
      }
      else
      {
        // — Release: restart long-press timer so it cannot instantly fire
        RESTART_TIMER(tLongPress);
      }
    }
  }

  // — Long-press detection while held down
  if (switchStableState == LOW)
  {
    if (DUE(tLongPress))
    {
      switchOff();
    }
  }

} //   updateSwitch()

// ===================== Main loop helpers =====================

static void serviceBackgroundTasks()
{
  // — Keep switch handling and UART passthrough running continuously
  updateSwitch();
  spsUartPassthroughLoop();

} //   serviceBackgroundTasks()

static void handleStateWarmup()
{
  if (warmupRemaining == 0)
  {
    // — Reset accumulation at end of warmup
    attemptIndex = 0;
    validCount = 0;
    sumPm1 = 0.0f;
    sumPm25 = 0.0f;
    sumPm10 = 0.0f;

    // — Start measurement cycle cleanly
    RESTART_TIMER(tSampleInterval);

    messageText = "Measuring...";
    messageText = truncateToChars(messageText, 18);
    uiUpdateMessagePartial();

    mainState = STATE_MEASURE_START_ATTEMPT;
    return;
  }

  // — 1Hz warmup tick
  if (DUE(tWarmupTick))
  {
    warmupRemaining--;

    // — Update env line during warmup
    updateEnvNow("warmup");
    uiUpdateEnvPartial();

    messageText = String("Warmup: ") + String((unsigned)warmupRemaining) + "s";
    messageText = truncateToChars(messageText, 18);
    uiUpdateMessagePartial();
  }

} //   handleStateWarmup()

static void handleStateMeasureStartAttempt()
{
  if (attemptIndex >= maxMetingen)
  {
    if (validCount == 0)
    {
      messageText = "ERROR: no samples";
      messageText = truncateToChars(messageText, 18);
      uiUpdateMessagePartial();
      mainState = STATE_ERROR;
      return;
    }

    // — Compute averages
    avgPm1 = sumPm1 / (float)validCount;
    avgPm25 = sumPm25 / (float)validCount;
    avgPm10 = sumPm10 / (float)validCount;

    mainState = STATE_SHOW_RESULTS;
    return;
  }

  // — Wait until the next sample slot is due
  if (!DUE(tSampleInterval))
  {
    return;
  }

  attemptIndex++;

  // — Refresh env line occasionally while sampling
  updateEnvNow("sample");
  uiUpdateEnvPartial();

  // — Show attempt progress
  {
    char buf[32];
    snprintf(buf, sizeof(buf), "Sample %u/%u", (unsigned)attemptIndex, (unsigned)maxMetingen);
    messageText = String(buf);
  }

  messageText = truncateToChars(messageText, 18);
  uiUpdateMessagePartial();

  // — Start attempt timeout + polling deterministically
  RESTART_TIMER(tAttemptTimeout);
  RESTART_TIMER(tSpsPoll);

  mainState = STATE_MEASURE_POLL_READY;

} //   handleStateMeasureStartAttempt()

static void handleStateMeasurePollReady()
{
  // — Attempt timeout
  if (DUE(tAttemptTimeout))
  {
    messageText = "Timeout: no data";
    messageText = truncateToChars(messageText, 18);
    uiUpdateMessagePartial();
    mainState = STATE_MEASURE_WAIT_NEXT;
    return;
  }

  // — Poll not faster than poll interval
  if (!DUE(tSpsPoll))
  {
    return;
  }

  uint16_t ready = 0;

  // — Read data-ready flag
  if (sps30_read_data_ready(&ready) != 0)
  {
    return;
  }

  if (ready == 0)
  {
    return;
  }

  struct sps30_measurement m;

  // — Read measurement
  if (sps30_read_measurement(&m) != 0)
  {
    messageText = "Read failed";
    messageText = truncateToChars(messageText, 18);
    uiUpdateMessagePartial();
    mainState = STATE_MEASURE_WAIT_NEXT;
    return;
  }

  float pm1 = m.mc_1p0;
  float pm25 = m.mc_2p5;
  float pm10 = m.mc_10p0;

  // — Always show current measurement values
  pm1Text = truncateToChars(formatPmLine(pm1), 12);
  pm25Text = truncateToChars(formatPmLine(pm25), 12);
  pm10Text = truncateToChars(formatPmLine(pm10), 12);

  // — Validate and accumulate
  if (isValidSample(pm1, pm25, pm10))
  {
    sumPm1 += pm1;
    sumPm25 += pm25;
    sumPm10 += pm10;
    validCount++;

    {
      char buf[32];
      snprintf(buf, sizeof(buf), "Valid: %u/%u", (unsigned)validCount, (unsigned)attemptIndex);
      Serial.printf("%s\n", buf);
      messageText = String(buf);
      Serial.printf("Sample %u: PM1=%.1f PM2.5=%.1f PM10=%.1f\n",
                    (unsigned)attemptIndex, pm1, pm25, pm10);
    }
  }
  else
  {
    messageText = "Invalid sample";
  }

  messageText = truncateToChars(messageText, 18);

  // — Update PM rows + message
  uiUpdatePmPartial();
  uiUpdateMessagePartial();

  mainState = STATE_MEASURE_WAIT_NEXT;

} //   handleStateMeasurePollReady()

static void handleStateMeasureWaitNext()
{
  mainState = STATE_MEASURE_START_ATTEMPT;

} //   handleStateMeasureWaitNext()

static void handleStateShowResults()
{
  pm1Text = truncateToChars(formatPmLine(avgPm1), 8);
  pm25Text = truncateToChars(formatPmLine(avgPm25), 8);
  pm10Text = truncateToChars(formatPmLine(avgPm10), 8);

  {
    AirStatus st = classifyPm25(avgPm25);
    messageText = String("Done: ") + String(statusText(st));
  }

  messageText = truncateToChars(messageText, 18);

  // — Full refresh before final screen
  uiDrawAllFull();

  spsStop();
  mainState = STATE_POWER_DOWN;

} //   handleStateShowResults()

static void handleStateError()
{
  spsStop();
  mainState = STATE_POWER_DOWN;

} //   handleStateError()

static void handleStatePowerDown()
{
  switchOff();

} //   handleStatePowerDown()

static void runMainStateMachine()
{
  switch (mainState)
  {
  case STATE_WARMUP:
  {
    handleStateWarmup();
    break;
  }

  case STATE_MEASURE_START_ATTEMPT:
  {
    handleStateMeasureStartAttempt();
    break;
  }

  case STATE_MEASURE_POLL_READY:
  {
    handleStateMeasurePollReady();
    break;
  }

  case STATE_MEASURE_WAIT_NEXT:
  {
    handleStateMeasureWaitNext();
    break;
  }

  case STATE_SHOW_RESULTS:
  {
    handleStateShowResults();
    break;
  }

  case STATE_ERROR:
  {
    handleStateError();
    break;
  }

  case STATE_POWER_DOWN:
  default:
  {
    handleStatePowerDown();
    break;
  }
  }

} //   runMainStateMachine()

// ===================== Arduino entrypoints =====================

void setup()
{
  // — Disable radios immediately to reduce current draw and RF noise
  disableRadiosAsap();

  // — Latch power immediately
  pinMode(pinLatch, OUTPUT);
  digitalWrite(pinLatch, HIGH);

  beepLatchFixed();

  // — Start serial logging
  Serial.begin(115200);
  delay(20);

  // — Configure basic IO
  pinMode(pinSwitch, INPUT_PULLUP);

  // — Configure ADC input and resolution
  pinMode(pinBatAdc, INPUT);
  analogReadResolution(12);

  // — Discard first ADC read to settle
  (void)analogRead(pinBatAdc);

  // — UART passthrough (debug)
  spsUartPassthroughInit();

  // — Boot logs
  Serial.printf("\n\nAnd then it starts ...\n\n");
  Serial.printf("Firmware version: %s\n", PROG_VERSION);
  Serial.printf("Warm-up time: %u seconds\n", (unsigned)warmupSeconds);
  Serial.printf("Max measurements: %u\n", (unsigned)maxMetingen);

  // — Initialize I2C early (required for BMP init and SPS)
  i2cInit();

  // — Bring-up diagnostics
  i2cScan();

  // — Initialize BMP and read immediately
  bmpOk = bmpInit();
  Serial.printf("BMP init: %s\n", bmpOk ? "OK" : "FAIL");

  // — Initialize e-paper
  epdInit();

  // — Initialize UI strings
  batteryText.reserve(32);
  envText.reserve(24);
  pm1Text.reserve(16);
  pm25Text.reserve(16);
  pm10Text.reserve(16);
  messageText.reserve(64);

  // — Placeholders
  batteryText = "-";
  envText = "-";
  pm1Text = "-";
  pm25Text = "-";
  pm10Text = "-";
  messageText = "Booting...";
  messageText = truncateToChars(messageText, 18);

  // — Initial readings BEFORE first draw
  updateBatteryNow("boot");
  updateEnvNow("boot");

  // — Full-screen initial draw (no title, 12pt everywhere)
  uiDrawAllFull();

  // — Initialize SPS and decide state
  if (!spsInitAndStart())
  {
    Serial.printf("Error: SPS not found\n");
    messageText = "ERROR: SPS missing";
    messageText = truncateToChars(messageText, 18);
    uiUpdateMessagePartial();
    delay(2000);
    mainState = STATE_ERROR;
  }
  else
  {
    warmupRemaining = warmupSeconds;
    messageText = "Warming up...";
    messageText = truncateToChars(messageText, 18);
    uiUpdateMessagePartial();
    mainState = STATE_WARMUP;
  }

  // — Start timers deterministically
  RESTART_TIMER(tWarmupTick);
  RESTART_TIMER(tSampleInterval);
  RESTART_TIMER(tAttemptTimeout);
  RESTART_TIMER(tSpsPoll);
  RESTART_TIMER(tSwitchDebounce);
  RESTART_TIMER(tLongPress);

  // — Initialize switch state tracking
  switchStableState = (digitalRead(pinSwitch) == LOW) ? LOW : HIGH;
  switchLastReading = switchStableState;
  updateSwitch();

} //   setup()

void loop()
{
  serviceBackgroundTasks();
  runMainStateMachine();

} //   loop()
