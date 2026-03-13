/*** Last Changed: 2026-03-13 - 12:21 ***/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// — ESP32 radio control to disable WiFi/Bluetooth as early as possible
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_bt.h>

#include "WiFiManagerExt.h"

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
static const char* PROG_VERSION = "v0.10.3";
// —             ^^^^^^^^^^^^^^

#ifndef E_PAPER_ROTATION
#define E_PAPER_ROTATION 1
#endif

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
static const uint8_t epdRotation = (uint8_t)E_PAPER_ROTATION;

// — SPS I2C pins (also used for BMP280)
static const uint8_t pinSpsSda = GPIO_PIN_SPS30_SDA;
static const uint8_t pinSpsScl = GPIO_PIN_SPS30_SCL;

// — PMS UART pins (UART2)
static const int8_t pinPms5003UartTx = GPIO_PIN_UART_TX2_PMS5003;
static const int8_t pinPms5003UartRx = GPIO_PIN_UART_RX2_PMS5003;
static const uint8_t pinPms5003Set = GPIO_PIN_PMS5003_SET;
static const uint8_t pinPms5003Reset = GPIO_PIN_PMS5003_RESET;

static HardwareSerial spsUart(2);

enum ParticleSensorType
{
  SENSOR_TYPE_SPS30,
  SENSOR_TYPE_PMS5003
};

static ParticleSensorType activeParticleSensor = SENSOR_TYPE_SPS30;
static WiFiManagerExt wifiManagerExt;

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
static const uint32_t warmupSampleIntervalMs = 5000UL;
static const uint32_t sampleIntervalMs = 2500UL;
static const uint32_t spsAttemptTimeoutMs = 7000UL;
static const uint32_t spsPollIntervalMs = 100UL;

// — Switch handling
static const uint32_t switchDebounceMs = 30UL;
static const uint32_t switchShortPressMs = 2000UL;
static const uint32_t switchLongPressMs = 3000UL;
static const uint32_t switchMultiPressWindowMs = 1500UL;
static const uint32_t switchTaskIntervalMs = 10UL;

// — safeTimers declarations
DECLARE_TIMER_MS(tWarmupTick, warmupTickMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tWarmupSampleInterval, warmupSampleIntervalMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tSampleInterval, sampleIntervalMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tAttemptTimeout, spsAttemptTimeoutMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tSpsPoll, spsPollIntervalMs, SKIP_MISSED_TICKS);

// ===================== Display selection =====================

// — 1.54" 200x200 B/W display; full height buffer is fine on ESP32
static const uint16_t pageHeight = 200;

// — Display driver instance
#if E_PAPER_VERSION == 1
// — Old Waveshare panel (IL3829)
GxEPD2_BW<GxEPD2_154, pageHeight> display(
    GxEPD2_154(pinEpdCs, pinEpdDc, pinEpdRst, pinEpdBusy));
#elif E_PAPER_VERSION == 2
// — New Waveshare panel (SSD1681)
GxEPD2_BW<GxEPD2_154_D67, pageHeight> display(
    GxEPD2_154_D67(pinEpdCs, pinEpdDc, pinEpdRst, pinEpdBusy));
#else
#error "Error: Unsupported DISPLAY_VERSION"
#endif

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
        4, // xText

        4,   // PM column 1
        70,  // PM column 2
        136, // PM column 3

        26, // yBattery (9pt) -26-
        52, // yEnv (9pt) -48-

        90,  // yPmHeader (9pt) -72-
        128, // yPmValues (12pt) -110-

        183 // yMessage (9pt) -186-
};

// — UI text fields
static String batteryText;
static String envText;
static String pm1Text;
static String pm25Text;
static String pm10Text;
static String messageText;
static uint8_t batteryPercentLast = 0;
static float batteryVoltageLast = 0.0f;
static volatile bool displayRefreshInProgress = false;

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

static bool i2cScanHasAddress(uint8_t targetAddr)
{
  Serial.printf("I2C scan start\n");

  uint8_t found = 0;
  bool targetFound = false;

  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    uint8_t err = (uint8_t)Wire.endTransmission(true);

    if (err == 0)
    {
      Serial.printf("I2C device found at 0x%02X\n", (unsigned)addr);
      found++;

      if (addr == targetAddr)
      {
        targetFound = true;
      }
    }

    delay(2);
    yield();
  }

  Serial.printf("I2C scan done (%u devices)\n", (unsigned)found);

  return targetFound;

} //   i2cScanHasAddress()

// ===================== UART passthrough =====================

static void spsUartPassthroughInit()
{
  // — PMS UART: 115200 8N1
  spsUart.begin(115200, SERIAL_8N1, pinPms5003UartRx, pinPms5003UartTx);

  Serial.printf("PMS UART passthrough on UART2: RX=%d TX=%d\n",
                (int)pinPms5003UartRx, (int)pinPms5003UartTx);

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
  // — Example: "4.1V (78%)"
  char buf[48];
  snprintf(buf, sizeof(buf), "%.1fV (%u%%)", vbat, pct);
  return String(buf);

} //   formatBatteryLine()

static void updateBatteryNow(const char* reason)
{
  // — Read and render battery line immediately
  float vbat = readBatteryVoltage();
  uint8_t pct = batteryPercentFromVoltage(vbat);

  batteryVoltageLast = vbat;
  batteryPercentLast = pct;

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

  if (epdRotation <= 3)
  {
    display.setRotation(epdRotation);
  }
  else
  {
    Serial.printf("Warning: invalid E_PAPER_ROTATION=%u, fallback to 1\n", (unsigned)epdRotation);
    display.setRotation(1);
  }

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

  displayRefreshInProgress = true;
  display.firstPage();
  do
  {
    display.fillRect(x, y, w, h, GxEPD_WHITE);

    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(ui.xText, yBaseline);
    display.print(text);

  } while (display.nextPage());
  displayRefreshInProgress = false;

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

  displayRefreshInProgress = true;
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
  displayRefreshInProgress = false;

} //   uiUpdatePmValuesPartial12pt()

static void uiUpdatePmTextCenteredPartial12pt(const String& text)
{
  // — Clear PM values band and draw one centered text line in 12pt
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

  int16_t textX = ui.xPmCol1;
  int16_t textBoundsX = 0;
  int16_t textBoundsY = 0;
  uint16_t textBoundsW = 0;
  uint16_t textBoundsH = 0;

  display.setFont(&FreeMonoBold12pt7b);
  display.getTextBounds(text.c_str(), 0, ui.yPmValues, &textBoundsX, &textBoundsY, &textBoundsW, &textBoundsH);

  if (textBoundsW < (uint16_t)display.width())
  {
    textX = (((int16_t)display.width() - (int16_t)textBoundsW) / 2) - textBoundsX;
  }

  display.setPartialWindow(x, y, w, h);

  displayRefreshInProgress = true;
  display.firstPage();
  do
  {
    display.fillRect(x, y, w, h, GxEPD_WHITE);

    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMonoBold12pt7b);
    display.setCursor(textX, ui.yPmValues);
    display.print(text);

  } while (display.nextPage());
  displayRefreshInProgress = false;

} //   uiUpdatePmTextCenteredPartial12pt()

static void uiDrawAllFull()
{
  display.setFullWindow();

  displayRefreshInProgress = true;
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
  displayRefreshInProgress = false;

} //   uiDrawAllFull()

static void uiDrawWaitingForUpdateFull(const String& hostName, const String& ipAddress)
{
  display.setFullWindow();

  displayRefreshInProgress = true;
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);

    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(ui.xText, 70);
    display.print("WAITING FOR");

    display.setCursor(ui.xText, 96);
    display.print("UPDATE");

    display.setCursor(ui.xText, 136);
    display.print(hostName);

    display.setCursor(ui.xText, 164);
    display.print(ipAddress);

  } while (display.nextPage());
  displayRefreshInProgress = false;

} //   uiDrawWaitingForUpdateFull()

static void uiDrawCenteredLine9pt(int16_t yBaseline, const String& text)
{
  int16_t textBoundsX = 0;
  int16_t textBoundsY = 0;
  uint16_t textBoundsW = 0;
  uint16_t textBoundsH = 0;

  display.setFont(&FreeMonoBold9pt7b);
  display.getTextBounds(text.c_str(), 0, yBaseline, &textBoundsX, &textBoundsY, &textBoundsW, &textBoundsH);

  int16_t textX = ui.xText;
  if (textBoundsW < (uint16_t)display.width())
  {
    textX = (((int16_t)display.width() - (int16_t)textBoundsW) / 2) - textBoundsX;
  }

  display.setCursor(textX, yBaseline);
  display.print(text);

} //   uiDrawCenteredLine9pt()

static void uiDrawWifiPortalStartedFull(const String& apName)
{
  display.setFullWindow();

  displayRefreshInProgress = true;
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);

    uiDrawCenteredLine9pt(52, String("WiFi Portal"));
    uiDrawCenteredLine9pt(78, String("Started"));
    uiDrawCenteredLine9pt(104, String("Waiting for user"));
    uiDrawCenteredLine9pt(130, String("input"));
    uiDrawCenteredLine9pt(156, apName);

  } while (display.nextPage());
  displayRefreshInProgress = false;

} //   uiDrawWifiPortalStartedFull()

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
  // — Clear from status band down to panel bottom to remove descender artifacts
  int16_t x = 0;
  int16_t y = ui.yMessage - 24;
  int16_t w = display.width();
  int16_t h = display.height() - y;

  if (y < 0)
  {
    y = 0;
    h = display.height();
  }

  if (h < 1)
  {
    h = 1;
  }

  display.setPartialWindow(x, y, w, h);

  displayRefreshInProgress = true;
  display.firstPage();
  do
  {
    display.fillRect(x, y, w, h, GxEPD_WHITE);

    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(ui.xText, ui.yMessage);
    display.print(messageText);

  } while (display.nextPage());
  displayRefreshInProgress = false;

} //   uiUpdateMessagePartial()

static void uiUpdateDynamicAll()
{
  uiUpdateBatteryPartial();
  uiUpdateEnvPartial();
  uiUpdatePmPartial();
  uiUpdateMessagePartial();

} //   uiUpdateDynamicAll()

static void epdEnterLowPowerMode()
{
  // — Ensure panel high-voltage stage is turned off before hard power cut
  display.powerOff();

  // — Put controller in deep sleep to preserve image contrast until VCC drops
  display.hibernate();

} //   epdEnterLowPowerMode()

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

static bool pms5003InitAndStart()
{
  // — PMS5003 UART is fixed at 9600 8N1
  spsUart.begin(9600, SERIAL_8N1, pinPms5003UartRx, pinPms5003UartTx);

  // — SET and RESET are active-low control pins
  pinMode(pinPms5003Set, OUTPUT);
  pinMode(pinPms5003Reset, OUTPUT);
  digitalWrite(pinPms5003Set, HIGH);
  digitalWrite(pinPms5003Reset, HIGH);

  delay(100);

  if (pinPms5003Reset == GPIO_PIN_LED)
  {
    Serial.printf("Warning: PMS5003 RESET shares GPIO_PIN_LED=%u\n", (unsigned)GPIO_PIN_LED);
  }

  Serial.printf("PMS5003 UART active: RX=%d TX=%d SET=%u RESET=%u\n",
                (int)pinPms5003UartRx,
                (int)pinPms5003UartTx,
                (unsigned)pinPms5003Set,
                (unsigned)pinPms5003Reset);

  return true;

} //   pms5003InitAndStart()

static void pms5003Stop()
{
  // — Put PMS5003 into sleep mode before hard power cut
  digitalWrite(pinPms5003Set, LOW);

} //   pms5003Stop()

static bool pms5003ReadMeasurement(float* pm1, float* pm25, float* pm10)
{
  static uint8_t frame[32];
  static uint8_t idx = 0;

  while (spsUart.available() > 0)
  {
    uint8_t b = (uint8_t)spsUart.read();

    if (idx == 0)
    {
      if (b != 0x42)
      {
        continue;
      }

      frame[idx++] = b;
      continue;
    }

    if (idx == 1)
    {
      if (b != 0x4D)
      {
        idx = 0;
        continue;
      }

      frame[idx++] = b;
      continue;
    }

    frame[idx++] = b;

    if (idx < (uint8_t)sizeof(frame))
    {
      continue;
    }

    idx = 0;

    uint16_t frameLen = ((uint16_t)frame[2] << 8) | (uint16_t)frame[3];
    if (frameLen != 28)
    {
      continue;
    }

    uint16_t checksum = 0;
    for (uint8_t i = 0; i < 30; i++)
    {
      checksum = (uint16_t)(checksum + frame[i]);
    }

    uint16_t checksumRx = ((uint16_t)frame[30] << 8) | (uint16_t)frame[31];
    if (checksum != checksumRx)
    {
      continue;
    }

    uint16_t pm1Atm = ((uint16_t)frame[10] << 8) | (uint16_t)frame[11];
    uint16_t pm25Atm = ((uint16_t)frame[12] << 8) | (uint16_t)frame[13];
    uint16_t pm10Atm = ((uint16_t)frame[14] << 8) | (uint16_t)frame[15];

    *pm1 = (float)pm1Atm;
    *pm25 = (float)pm25Atm;
    *pm10 = (float)pm10Atm;
    return true;
  }

  return false;

} //   pms5003ReadMeasurement()

static bool pms5003Probe(uint32_t timeoutMs)
{
  uint32_t startMs = millis();

  while ((millis() - startMs) < timeoutMs)
  {
    float pm1 = 0.0f;
    float pm25 = 0.0f;
    float pm10 = 0.0f;

    if (pms5003ReadMeasurement(&pm1, &pm25, &pm10))
    {
      Serial.printf("PMS5003 probe OK: PM1=%.1f PM2.5=%.1f PM10=%.1f\n", pm1, pm25, pm10);
      return true;
    }

    delay(20);
    yield();
  }

  Serial.printf("PMS5003 probe timeout\n");
  return false;

} //   pms5003Probe()

static bool particleSensorInitAndStart(bool sps30SeenOnI2c)
{
  if (sps30SeenOnI2c)
  {
    if (spsInitAndStart())
    {
      activeParticleSensor = SENSOR_TYPE_SPS30;
      Serial.printf("Particle sensor selected: SPS30 (I2C)\n");
      return true;
    }

    Serial.printf("SPS30 seen on I2C, but init failed. Trying PMS5003 fallback.\n");
  }
  else
  {
    Serial.printf("SPS30 not found on I2C scan. Trying PMS5003 fallback.\n");
  }

  if (pms5003InitAndStart())
  {
    if (pms5003Probe(2500UL))
    {
      activeParticleSensor = SENSOR_TYPE_PMS5003;
      Serial.printf("Particle sensor selected: PMS5003 (UART2)\n");
      return true;
    }

    pms5003Stop();
    Serial.printf("PMS5003 not detected on UART2\n");
  }

  return false;

} //   particleSensorInitAndStart()

static void particleSensorStop()
{
  if (activeParticleSensor == SENSOR_TYPE_SPS30)
  {
    spsStop();
    return;
  }

  pms5003Stop();

} //   particleSensorStop()

static bool particleSensorReadMeasurement(float* pm1, float* pm25, float* pm10)
{
  if (activeParticleSensor == SENSOR_TYPE_SPS30)
  {
    uint16_t ready = 0;

    if (sps30_read_data_ready(&ready) != 0)
    {
      return false;
    }

    if (ready == 0)
    {
      return false;
    }

    struct sps30_measurement m;

    if (sps30_read_measurement(&m) != 0)
    {
      return false;
    }

    *pm1 = m.mc_1p0;
    *pm25 = m.mc_2p5;
    *pm10 = m.mc_10p0;
    return true;
  }

  return pms5003ReadMeasurement(pm1, pm25, pm10);

} //   particleSensorReadMeasurement()

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
  if (pm1 > 1000.0f || pm25 > 1000.0f || pm10 > 1000.0f)
  {
    return false;
  }

  return true;

} //   isValidSample()

static String formatPmLine(float v)
{
  // — PM display formatting:
  // — <100: one decimal
  // — 100..999: integer
  // — >999: show "999"
  char buf[16];

  if (v > 999.0f)
  {
    snprintf(buf, sizeof(buf), "999");
    return String(buf);
  }

  if (v >= 100.0f)
  {
    snprintf(buf, sizeof(buf), "%.0f", v);
    return String(buf);
  }

  float belowHundred = (v < 99.9f) ? v : 99.9f;
  snprintf(buf, sizeof(buf), "%.1f", belowHundred);
  return String(buf);

} //   formatPmLine()

static String formatAvgLine(float v, const char* suffix)
{
  // — Format average line as "123.4 <suffix>" (e.g. "12.3 AVG")
  char buf[24];
  snprintf(buf, sizeof(buf), "%.1f %s", v, suffix);
  return String(buf);

} //   formatAvgLine()

// — Very short beep to mark each measurement read
static void beepBeforeMeasurement(void)
{
  tone((uint8_t)GPIO_PIN_BUZZER_PWM, 2200U);
  delay(12);
  noTone((uint8_t)GPIO_PIN_BUZZER_PWM);
  digitalWrite((uint8_t)GPIO_PIN_BUZZER_PWM, LOW);

} //   beepBeforeMeasurement()

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

// ===================== WiFi + OTA callbacks =====================

static void onWifiPortalStarted(const String& apName)
{
  uiDrawWifiPortalStartedFull(apName);

} //   onWifiPortalStarted()

static void onOtaStarted()
{
  messageText = "UPDATING";
  messageText = truncateToChars(messageText, 18);
  uiDrawAllFull();

} //   onOtaStarted()

static void onOtaEnded()
{
  messageText = "OTA READY";
  messageText = truncateToChars(messageText, 18);
  uiUpdateMessagePartial();

} //   onOtaEnded()

static void onWifiConnected(const String& hostName, const String& ipAddress)
{
  uiDrawWaitingForUpdateFull(hostName, ipAddress);

} //   onWifiConnected()

// ===================== Power off =====================

static void switchOff(const char* overrideMessage = nullptr)
{
  // — Never release latch while OTA update is active
  if (wifiManagerExt.isOtaInProgress())
  {
    return;
  }

  // — Update battery and env readings right before unlatching power
  updateBatteryNow("pre-off");
  updateEnvNow("pre-off");

  // — Show optional forced shutdown message
  if ((overrideMessage != nullptr) && (overrideMessage[0] != '\0'))
  {
    messageText = String(overrideMessage);
  }
  // — Show battery warning before shutdown when battery is below 3.6V
  else if (batteryVoltageLast < 3.6f)
  {
    messageText = "BATTERY LOW";
  }
  else
  {
    messageText = " ";
  }

  messageText = truncateToChars(messageText, 18);
  uiDrawAllFull();

  // — Put e-paper into low-power mode before cutting board power
  epdEnterLowPowerMode();

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
static uint32_t switchLastReadingChangeMs = 0;
static uint32_t switchPressedSinceMs = 0;
static bool switchLongPressHandled = false;
static bool ledState = LOW;
static bool ledAutoOffActive = false;
static uint32_t ledOnSinceMs = 0;
static volatile bool switchOffRequested = false;
static volatile bool wifiStartRequested = false;
static uint8_t switchShortPressCount = 0;
static uint32_t switchFirstShortPressMs = 0;

static TaskHandle_t switchTaskHandle = nullptr;

static void updateSwitch()
{
  // — Read switch with pull-up logic (LOW means pressed)
  bool reading = (digitalRead(pinSwitch) == LOW) ? LOW : HIGH;
  uint32_t nowMs = millis();

  // — Track raw edges for debounce
  if (reading != switchLastReading)
  {
    switchLastReading = reading;
    switchLastReadingChangeMs = nowMs;
  }

  // — Accept state change only after debounce interval
  if ((nowMs - switchLastReadingChangeMs) >= switchDebounceMs)
  {
    if (switchStableState != switchLastReading)
    {
      switchStableState = switchLastReading;

      if (switchStableState == LOW)
      {
        // — New press started
        switchPressedSinceMs = nowMs;
        switchLongPressHandled = false;

        // — Press always turns LED on; auto-off after short-press window
        ledState = HIGH;
        digitalWrite(GPIO_PIN_LED, HIGH);
        ledAutoOffActive = true;
        ledOnSinceMs = nowMs;
      }
      else
      {
        // — Released: detect short-press burst for WiFi start
        if (!switchLongPressHandled)
        {
          uint32_t pressDurationMs = nowMs - switchPressedSinceMs;

          if (pressDurationMs < switchLongPressMs)
          {
            if ((switchShortPressCount == 0) || ((nowMs - switchFirstShortPressMs) > switchMultiPressWindowMs))
            {
              switchShortPressCount = 0;
              switchFirstShortPressMs = nowMs;
            }

            switchShortPressCount++;

            if (switchShortPressCount >= 3)
            {
              switchShortPressCount = 0;
              wifiStartRequested = true;
            }
          }
        }
      }
    }
  }

  // — Auto-off LED 2 seconds after press start
  if (ledAutoOffActive)
  {
    if ((nowMs - ledOnSinceMs) >= switchShortPressMs)
    {
      ledAutoOffActive = false;
      ledState = LOW;
      digitalWrite(GPIO_PIN_LED, LOW);
    }
  }

  // — Long-press detection while held down
  if ((switchStableState == LOW) && !switchLongPressHandled)
  {
    if ((nowMs - switchPressedSinceMs) >= switchLongPressMs)
    {
      switchLongPressHandled = true;
      switchShortPressCount = 0;
      switchOffRequested = true;
    }
  }

} //   updateSwitch()

static void switchTask(void* parameter)
{
  (void)parameter;

  while (true)
  {
    updateSwitch();
    vTaskDelay(pdMS_TO_TICKS(switchTaskIntervalMs));
  }

} //   switchTask()

// ===================== Main loop helpers =====================

static void serviceBackgroundTasks()
{
  // — Keep UART passthrough running continuously
  if (activeParticleSensor == SENSOR_TYPE_SPS30)
  {
    spsUartPassthroughLoop();
  }

} //   serviceBackgroundTasks()

static void processSwitchOffRequest()
{
  if (wifiManagerExt.isOtaInProgress())
  {
    return;
  }

  if (switchOffRequested && !displayRefreshInProgress)
  {
    switchOffRequested = false;
    switchOff();
  }

} //   processSwitchOffRequest()

static void processWifiStartRequest()
{
  if (!wifiStartRequested)
  {
    return;
  }

  wifiStartRequested = false;

  // — Show portal start intent immediately, independent of callback timing
  {
    String apName = wifiManagerExt.getApNamePreview();
    uiDrawWifiPortalStartedFull(apName);
  }

  wifiManagerExt.requestStart();

} //   processWifiStartRequest()

static void handleStateWarmup()
{
  // — During warmup, show live PM values every 5s without adding to averages
  if (DUE(tWarmupSampleInterval))
  {
    float pm1 = 0.0f;
    float pm25 = 0.0f;
    float pm10 = 0.0f;

    if (particleSensorReadMeasurement(&pm1, &pm25, &pm10))
    {
      pm1Text = truncateToChars(formatPmLine(pm1), 12);
      pm25Text = truncateToChars(formatPmLine(pm25), 12);
      pm10Text = truncateToChars(formatPmLine(pm10), 12);

      uiUpdatePmPartial();

      Serial.printf("Warmup sample: PM1=%.1f PM2.5=%.1f PM10=%.1f\n",
                    pm1,
                    pm25,
                    pm10);
    }
  }

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

    // — Single short beep when warmup is finished
    beepBeforeMeasurement();

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

  float pm1 = 0.0f;
  float pm25 = 0.0f;
  float pm10 = 0.0f;

  // — Read one measurement from active sensor
  if (!particleSensorReadMeasurement(&pm1, &pm25, &pm10))
  {
    return;
  }

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

  particleSensorStop();
  mainState = STATE_POWER_DOWN;

} //   handleStateShowResults()

static void handleStateError()
{
  particleSensorStop();
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

  // — Boot logs
  Serial.printf("\n\nAnd then it starts ...\n\n");
  Serial.printf("Firmware version: %s\n", PROG_VERSION);
  Serial.printf("Latch Pin %u latched (set %s), radios off\n", (unsigned)pinLatch, digitalRead(pinLatch) == HIGH ? "HIGH" : "LOW");
  Serial.printf("Warm-up time: %u seconds\n", (unsigned)warmupSeconds);
  Serial.printf("Max measurements: %u\n", (unsigned)maxMetingen);
  Serial.flush();

  pinMode(GPIO_PIN_LED, OUTPUT);
  digitalWrite(GPIO_PIN_LED, LOW);
  ledState = LOW;

  // — Initialize I2C early (required for BMP init and SPS)
  i2cInit();

  // — Bring-up diagnostics
  bool sps30SeenOnI2c = i2cScanHasAddress(0x69);

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

  // — Initialize WiFi/OTA manager callbacks
  wifiManagerExt.setPortalStartCallback(onWifiPortalStarted);
  wifiManagerExt.setConnectedCallback(onWifiConnected);
  wifiManagerExt.setOtaStartCallback(onOtaStarted);
  wifiManagerExt.setOtaEndCallback(onOtaEnded);

  // — Protect battery from deep discharge at startup
  if (batteryVoltageLast < 3.4f)
  {
    switchOff("BATTERY TO LOW");
    return;
  }

  // — Initialize particle sensor with automatic fallback
  if (!particleSensorInitAndStart(sps30SeenOnI2c))
  {
    Serial.printf("Error: no particle sensor available\n");

    uiUpdatePmTextCenteredPartial12pt(String("NO SENSOR"));

    messageText = "";
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
  RESTART_TIMER(tWarmupSampleInterval);
  RESTART_TIMER(tSampleInterval);
  RESTART_TIMER(tAttemptTimeout);
  RESTART_TIMER(tSpsPoll);

  // — Initialize switch state tracking
  switchStableState = (digitalRead(pinSwitch) == LOW) ? LOW : HIGH;
  switchLastReading = switchStableState;
  switchLastReadingChangeMs = millis();
  switchPressedSinceMs = switchLastReadingChangeMs;
  switchLongPressHandled = false;
  switchShortPressCount = 0;
  switchFirstShortPressMs = 0;
  ledAutoOffActive = false;
  ledOnSinceMs = 0;
  updateSwitch();

  // — Run switch handling in a dedicated low-priority task
  xTaskCreatePinnedToCore(
      switchTask,
      "switchTask",
      2048,
      nullptr,
      1,
      &switchTaskHandle,
      tskNO_AFFINITY);

} //   setup()

void loop()
{
  serviceBackgroundTasks();
  processWifiStartRequest();
  wifiManagerExt.handle();
  processSwitchOffRequest();

  if (wifiManagerExt.isServiceModeActive())
  {
    delay(2);
    return;
  }

  runMainStateMachine();
  processSwitchOffRequest();

} //   loop()
