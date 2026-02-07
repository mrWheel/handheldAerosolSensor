/*** Last Changed: 2026-02-07 - 14:28 ***/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// — ESP32 radio control to disable WiFi/Bluetooth as early as possible
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_bt.h>

#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>

#include <sps30.h>
#include <safeTimers.h>

// — Program version string (keep manually updated with each release)
static const char* PROG_VERSION = "v0.2.2";

// ===================== User configuration (from build_flags) =====================

// — Latch control pin (drives the 2N7000 gate through a series resistor)
static const uint8_t pinLatch = GPIO_PIN_LATCH;

// — Pushbutton pin (LOW = pressed with INPUT_PULLUP)
static const uint8_t pinSwitch = GPIO_PIN_SWITCH;

// — Battery divider ADC input (default math assumes 2:1 divider => ADC sees Vbat/2)
static const uint8_t pinBatAdc = GPIO_PIN_BAT_ADC;

// — E-paper pins
// — E-paper SPI pins (from build_flags)
static const uint8_t pinEpdMosi = GPIO_PIN_EPD_MOSI;
static const uint8_t pinEpdMiso = GPIO_PIN_EPD_MISO;
static const uint8_t pinEpdSck = GPIO_PIN_EPD_SCK;
static const uint8_t pinEpdCs = GPIO_PIN_EPD_CS;
static const uint8_t pinEpdDc = GPIO_PIN_EPD_DC;
static const uint8_t pinEpdRst = GPIO_PIN_EPD_RST;
static const uint8_t pinEpdBusy = GPIO_PIN_EPD_BUSY;

// — SPS30 I2C pins
static const uint8_t pinSps30Sda = GPIO_PIN_SPS30_SDA;
static const uint8_t pinSps30Scl = GPIO_PIN_SPS30_SCL;

// — Warm-up time after starting SPS30 measurement
static const uint16_t warmupSeconds = WARMUP_SECONDS;

// — Maximum number of measurement attempts (only valid reads are averaged)
static const uint8_t maxMetingen = MAX_METINGEN;

// — ADC reference voltage used for battery math (calibrate for best accuracy)
static const float adcVref = (float)ADC_VREF_VOLTAGE;

// ===================== Timing (safeTimers) =====================

// — UI + sampling cadence
static const uint32_t warmupTickMs = 1000UL;
static const uint32_t sampleIntervalMs = 1000UL;
static const uint32_t sps30AttemptTimeoutMs = 7000UL;
static const uint32_t sps30PollIntervalMs = 100UL;

// — Switch handling
static const uint32_t switchDebounceMs = 30UL;
static const uint32_t switchLongPressMs = 5000UL;

// — safeTimers declarations
DECLARE_TIMER_MS(tWarmupTick, warmupTickMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tSampleInterval, sampleIntervalMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tAttemptTimeout, sps30AttemptTimeoutMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tSpsPoll, sps30PollIntervalMs, SKIP_MISSED_TICKS);

DECLARE_TIMER_MS(tSwitchDebounce, switchDebounceMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tLongPress, switchLongPressMs, SKIP_MISSED_TICKS);

// ===================== Display selection =====================

// — 1.54" 200x200 B/W display; page buffer sizing
//-x-static const uint16_t maxDisplayBufferSize = 800;
static const uint16_t pageHeight = 200; //(maxDisplayBufferSize / (200 / 8));

// — Display driver instance
//-- GxEPD2_BW<GxEPD2_154, pageHeight> display(...);
GxEPD2_BW<GxEPD2_154, pageHeight> display(
    GxEPD2_154(pinEpdCs, pinEpdDc, pinEpdRst, pinEpdBusy));
// GxEPD2_BW<GxEPD2_154_T8, pageHeight> display(
//   GxEPD2_154_T8(pinEpdCs, pinEpdDc, pinEpdRst, -1) //pinEpdBusy)
//);
// GxEPD2_BW<GxEPD2_154_D67, pageHeight> display(
//   GxEPD2_154_D67(pinEpdCs, pinEpdDc, pinEpdRst, -1) // -1 = no pinEpdBusy)
//);

// ===================== Fixed layout coordinates =====================

struct UiLayout
{
  int16_t xLabel;
  int16_t xValue;

  int16_t yTitle;

  int16_t yBattery;
  int16_t yPm1;
  int16_t yPm25;
  int16_t yPm10;

  int16_t yMessage;
};

// — Fixed positions for TITLE, Battery, PM lines and message area
static const UiLayout ui =
    {
        8,   // xLabel
        86,  // xValue (values aligned)
        24,  // yTitle baseline
        52,  // yBattery baseline
        84,  // yPm1 baseline
        114, // yPm25 baseline
        144, // yPm10 baseline
        186  // yMessage baseline
};

// — Remember last text per field to avoid unnecessary redraws
static String titleText;
static String batteryText;
static String pm1Text;
static String pm25Text;
static String pm10Text;
static String messageText;

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
}

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
}

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
}

// ===================== Battery reading =====================

static float readBatteryVoltage()
{
  // — ESP32 ADC is 12-bit by default (0..4095) when analogReadResolution(12) is used
  uint16_t raw = (uint16_t)analogRead(pinBatAdc);

  // — Convert ADC reading to voltage at the ADC pin
  float vAdc = ((float)raw * adcVref) / 4095.0f;

  // — Default assumes a 2:1 divider (ADC sees Vbat/2)
  return vAdc * 2.0f;
}

static uint8_t batteryPercentFromVoltage(float vbat)
{
  if (vbat >= 4.20f)
  {
    return 100;
  }

  if (vbat <= 3.20f)
  {
    return 0;
  }

  if (vbat > 3.70f)
  {
    float x = (vbat - 3.70f) * (50.0f / 0.50f);
    int p = (int)(50.0f + x);

    if (p > 100)
    {
      p = 100;
    }

    return (uint8_t)p;
  }
  else
  {
    float x = (vbat - 3.20f) * (50.0f / 0.50f);
    int p = (int)x;

    if (p < 0)
    {
      p = 0;
    }

    return (uint8_t)p;
  }
}

static String formatBatteryLine(float vbat, uint8_t pct)
{
  // — Format battery line using snprintf then convert to String
  char buf[48];

  // — Example: "4.02V 78%"
  snprintf(buf, sizeof(buf), "%.2fV %u%%", vbat, pct);

  return String(buf);
}

// ===================== E-paper helpers (fixed-layout fields) =====================
static void epdGeometryTest()
{
  // — Print geometry and rotation
  Serial.printf("EPD geometry: width=%d height=%d rotation=%d\n",
                (int)display.width(),
                (int)display.height(),
                (int)display.getRotation());

  // — Force full window to avoid partial artifacts
  display.setFullWindow();

  display.firstPage();
  do
  {
    // — Clear screen
    display.fillScreen(GxEPD_WHITE);

    // — Draw outer border
    display.drawRect(0, 0, display.width(), display.height(), GxEPD_BLACK);

    // — Draw safe border inset (helps to see clipping)
    display.drawRect(2, 2, display.width() - 4, display.height() - 4, GxEPD_BLACK);

    // — Draw center crosshair
    int16_t cx = display.width() / 2;
    int16_t cy = display.height() / 2;
    display.drawLine(cx, 0, cx, display.height() - 1, GxEPD_BLACK);
    display.drawLine(0, cy, display.width() - 1, cy, GxEPD_BLACK);

    // — Draw horizontal guide lines every 20px
    for (int16_t y = 0; y < display.height(); y += 20)
    {
      display.drawLine(0, y, display.width() - 1, y, GxEPD_BLACK);
    }

    // — Print corners and size text
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMonoBold9pt7b);

    // — Top-left
    display.setCursor(2, 12);
    display.print("TL (0,0)");

    // — Top-right
    {
      String s = String("TR (") + String(display.width() - 1) + ",0)";
      int16_t x = display.width() - (int16_t)(s.length() * 6) - 2;
      if (x < 2)
      {
        x = 2;
      }
      display.setCursor(x, 12);
      display.print(s);
    }

    // — Bottom-left (baseline must stay above bottom edge)
    {
      int16_t y = display.height() - 4;
      display.setCursor(2, y);
      display.print("BL (0,");
      display.print(display.height() - 1);
      display.print(")");
    }

    // — Size label in center
    {
      String s = String("W=") + String(display.width()) + " H=" + String(display.height());
      display.setCursor(10, cy - 4);
      display.print(s);
    }

  } while (display.nextPage());

  Serial.printf("EPD geometry test drawn\n");

} // epdGeometryTest()

static bool hasPartialUpdate()
{
  // — GxEPD2 provides this member; partial update support depends on panel/driver
  return display.epd2.hasFastPartialUpdate;
}

static void epdSmokeTest()
{
  Serial.printf("EPD smoke test: begin\n");

  Serial.printf("EPD setFullWindow()\n");
  display.setFullWindow();

  Serial.printf("EPD firstPage()\n");
  display.firstPage();

  Serial.printf("EPD drawing page...\n");
  display.fillScreen(GxEPD_WHITE);
  display.drawRect(0, 0, display.width(), display.height(), GxEPD_BLACK);

  Serial.printf("EPD nextPage() (this may block)\n");
  bool more = display.nextPage();

  Serial.printf("EPD nextPage returned: %d\n", more ? 1 : 0);
  Serial.printf("EPD smoke test: done\n");

} // epdSmokeTest()

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

  // — Log BUSY around init
  Serial.printf("EPD BUSY before init: %d\n", digitalRead(pinEpdBusy));

  // — Init display (GxEPD2 controls SPI transactions internally)
  display.init(115200, true, 2, false);

  // — Reduce SPI speed for long dupont wires (very common ESP32 issue)
  //-x-display.epd2.setSPISpeed(4000000);

  Serial.printf("EPD init returned\n");

  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);
  display.setFullWindow();

} // epdInit()

static void drawStaticLayout()
{
  // — Draw all static labels and frame once (full refresh)
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);

    display.setFont(&FreeMonoBold12pt7b);
    display.setCursor(ui.xLabel, ui.yTitle);
    display.print("TITLE");

    display.setFont(&FreeMonoBold9pt7b);

    display.setCursor(ui.xLabel, ui.yBattery);
    display.print("BAT:");

    display.setCursor(ui.xLabel, ui.yPm1);
    display.print("PM1.0:");

    display.setCursor(ui.xLabel, ui.yPm25);
    display.print("PM2.5:");

    display.setCursor(ui.xLabel, ui.yPm10);
    display.print("PM10 :");

    display.setCursor(ui.xLabel, ui.yMessage);
    display.print("MSG :");
  } while (display.nextPage());

  Serial.printf("EPD smoke test loop exited\n");

} // drawStaticLayout()

static void drawValueAt(int16_t yBaseline, const String& text)
{
  // — Draw a value at the fixed value column for the given baseline
  display.setCursor(ui.xValue, yBaseline);
  display.print(text);
}

static void clearValueBand(int16_t yBaseline, int16_t bandHeight)
{
  // — Clear the value area to the right of labels in a band around the baseline
  int16_t x = ui.xValue;
  int16_t y = yBaseline - bandHeight + 2;
  int16_t w = display.width() - ui.xValue;
  int16_t h = bandHeight + 4;

  display.fillRect(x, y, w, h, GxEPD_WHITE);
}

static void updateFieldBand(int16_t yBaseline, int16_t bandHeight, const String& newText, String& cache)
{
  // — Skip redraw when nothing changed
  if (newText == cache)
  {
    return;
  }

  cache = newText;

  // — Use partial update if supported; otherwise full window update
  if (hasPartialUpdate())
  {
    int16_t x = 0;
    int16_t y = yBaseline - bandHeight + 2;
    int16_t w = display.width();
    int16_t h = bandHeight + 6;

    display.setPartialWindow(x, y, w, h);
  }
  else
  {
    display.setFullWindow();
  }

  display.firstPage();
  do
  {
    // — Clear only the value band
    clearValueBand(yBaseline, bandHeight);

    // — Draw updated value text
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    drawValueAt(yBaseline, newText);
  } while (display.nextPage());
}

static void setTitle(const String& t)
{
  // — Title uses larger font, so we update it with its own band height
  if (t == titleText)
  {
    return;
  }

  titleText = t;

  // — Update title band (larger)
  if (hasPartialUpdate())
  {
    display.setPartialWindow(0, 0, display.width(), 40);
  }
  else
  {
    display.setFullWindow();
  }

  display.firstPage();
  do
  {
    // — Clear title value area (to the right of "TITLE")
    display.fillRect(ui.xValue, 0, display.width() - ui.xValue, 40, GxEPD_WHITE);

    // — Render title value
    display.setFont(&FreeMonoBold12pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(ui.xValue, ui.yTitle);
    display.print(t);
  } while (display.nextPage());
}

static void setBatteryText(const String& t)
{
  // — Battery line fits in a 24px band with 9pt font
  updateFieldBand(ui.yBattery, 22, t, batteryText);
}

static void setPm1Text(const String& t)
{
  // — PM1 line fits in a 24px band
  updateFieldBand(ui.yPm1, 22, t, pm1Text);
}

static void setPm25Text(const String& t)
{
  // — PM2.5 line fits in a 24px band
  updateFieldBand(ui.yPm25, 22, t, pm25Text);
}

static void setPm10Text(const String& t)
{
  // — PM10 line fits in a 24px band
  updateFieldBand(ui.yPm10, 22, t, pm10Text);
}

static void setMessageText(const String& t)
{
  // — Message line fits in a 24px band
  updateFieldBand(ui.yMessage, 22, t, messageText);
}

// ===================== SPS30 helpers =====================

static bool sps30InitAndStart()
{
  // — Initialize I2C on explicit pins
  Wire.begin(pinSps30Sda, pinSps30Scl);

  // — Probe sensor
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
}

static void sps30Stop()
{
  // — Stop measurement to reduce power
  (void)sps30_stop_measurement();
}

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
}

static String formatPmLine(float v)
{
  // — Format PM value as "123.4"
  char buf[16];
  snprintf(buf, sizeof(buf), "%.1f", v);
  return String(buf);
}

static String formatAvgLine(float v, const char* suffix)
{
  // — Format average line as "123.4 <suffix>" (e.g. "12.3 AVG")
  char buf[24];
  snprintf(buf, sizeof(buf), "%.1f %s", v, suffix);
  return String(buf);
}

// ===================== Power off =====================

static void updateBatteryNow(const char* reason)
{
  // — Read and render battery line immediately
  float vbat = readBatteryVoltage();
  uint8_t pct = batteryPercentFromVoltage(vbat);

  // — Update display battery field
  setBatteryText(formatBatteryLine(vbat, pct));

  // — Log battery status
  Serial.printf("Battery (%s): %.2f V (%u%%)\n", reason, vbat, pct);
}

static void switchOff()
{
  // — Update battery reading right before unlatching power
  updateBatteryNow("pre-off");

  // — Update message on display before switching off
  setMessageText("Switching off...");

  // — Log and flush
  Serial.printf("Switching off latch (PIN %u)\n", pinLatch);
  Serial.flush();

  // — Release the latch (hardware should cut power)
  digitalWrite(pinLatch, LOW);

  // — Fallback loop if power does not drop (debug aid)
  while (true)
  {
    delay(100);
  }
}

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
      Serial.printf("Long press detected -> switchOff()\n");
      switchOff();
    }
  }
}

// ===================== Arduino entrypoints =====================

void setup()
{
  // — Disable radios immediately to reduce current draw and RF noise
  disableRadiosAsap();

  // — Latch power immediately
  pinMode(pinLatch, OUTPUT);
  digitalWrite(pinLatch, HIGH);

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

  // — Basic boot logs
  Serial.printf("\n\nAnd then it starts ...\n\n");
  Serial.printf("Firmware version: %s\n", PROG_VERSION);
  Serial.printf("Warm-up time: %u seconds\n", (unsigned)warmupSeconds);
  Serial.printf("Max measurements: %u\n", (unsigned)maxMetingen);
  Serial.printf("PIN_LATCH: %u\n", (unsigned)pinLatch);
  Serial.printf("PIN_SWITCH: %u\n", (unsigned)pinSwitch);
  Serial.printf("PIN_BAT_ADC: %u\n", (unsigned)pinBatAdc);
  Serial.printf("I2C SDA/SCL: %u/%u\n", (unsigned)pinSps30Sda, (unsigned)pinSps30Scl);

  // — Prepare e-paper pins
  pinMode(pinEpdCs, OUTPUT);
  digitalWrite(pinEpdCs, HIGH);
  // — Quick BUSY sanity check
  // — BUSY level test with and without pull-up
  pinMode(pinEpdBusy, INPUT);
  delay(20);
  Serial.printf("EPD BUSY (INPUT)        : %d\n", digitalRead(pinEpdBusy));
  pinMode(pinEpdBusy, INPUT_PULLUP);
  delay(20);
  Serial.printf("EPD BUSY (INPUT_PULLUP) : %d\n", digitalRead(pinEpdBusy));

  // — Hard reset display
  pinMode(pinEpdRst, OUTPUT);
  digitalWrite(pinEpdRst, LOW);
  delay(10);
  digitalWrite(pinEpdRst, HIGH);
  delay(10);

  // — Sanity: toggle control pins so you can probe with a multimeter/logic probe
  pinMode(pinEpdCs, OUTPUT);
  pinMode(pinEpdDc, OUTPUT);
  pinMode(pinEpdRst, OUTPUT);

  Serial.printf("Toggling CS/DC/RST...\n");

  digitalWrite(pinEpdCs, HIGH);
  digitalWrite(pinEpdDc, LOW);
  digitalWrite(pinEpdRst, HIGH);
  delay(100);

  digitalWrite(pinEpdCs, LOW);
  delay(50);
  digitalWrite(pinEpdCs, HIGH);

  digitalWrite(pinEpdDc, HIGH);
  delay(50);
  digitalWrite(pinEpdDc, LOW);

  digitalWrite(pinEpdRst, LOW);
  delay(50);
  digitalWrite(pinEpdRst, HIGH);

  Serial.printf("Toggle done\n");

  // — Initialize e-paper
  Serial.printf("Initializing e-paper...\n");
  epdInit();

  // — After epdInit()
  epdGeometryTest();
  delay(10000);

  Serial.printf("Running epdSmokeTest()\n");
  epdSmokeTest();

  // — Draw fixed layout once
  drawStaticLayout();

  // — Initialize cached strings to something deterministic
  titleText.reserve(32);
  batteryText.reserve(32);
  pm1Text.reserve(32);
  pm25Text.reserve(32);
  pm10Text.reserve(32);
  messageText.reserve(64);

  // — Set initial title value
  setTitle("AIR MONITOR");

  // — Read battery at boot and show it immediately
  updateBatteryNow("boot");

  // — Initialize PM fields to placeholders
  setPm1Text("-");
  setPm25Text("-");
  setPm10Text("-");

  // — Initialize SPS30 and decide state
  if (!sps30InitAndStart())
  {
    Serial.printf("ERROR: SPS30 not found\n");
    setMessageText("ERROR: SPS30 missing");
    mainState = STATE_ERROR;
  }
  else
  {
    warmupRemaining = warmupSeconds;
    Serial.printf("SPS30 OK. Warming up...\n");
    setMessageText("Warming up...");
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
}

void loop()
{
  updateSwitch();

  switch (mainState)
  {
  case STATE_WARMUP:
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

      setMessageText("Measuring...");
      mainState = STATE_MEASURE_START_ATTEMPT;
      break;
    }

    // — 1Hz warmup tick
    if (DUE(tWarmupTick))
    {
      warmupRemaining--;

      // — Show warmup remaining seconds in message field
      String msg = String("Warmup: ") + String((unsigned)warmupRemaining) + "s";
      setMessageText(msg);

      Serial.printf("Warmup remaining: %u s\n", (unsigned)warmupRemaining);
    }

    break;
  }

  case STATE_MEASURE_START_ATTEMPT:
  {
    if (attemptIndex >= maxMetingen)
    {
      if (validCount == 0)
      {
        Serial.printf("ERROR: No valid samples\n");
        setMessageText("ERROR: no valid samples");
        mainState = STATE_ERROR;
        break;
      }

      // — Compute averages
      avgPm1 = sumPm1 / (float)validCount;
      avgPm25 = sumPm25 / (float)validCount;
      avgPm10 = sumPm10 / (float)validCount;

      mainState = STATE_SHOW_RESULTS;
      break;
    }

    // — Wait until the next sample slot is due
    if (!DUE(tSampleInterval))
    {
      break;
    }

    attemptIndex++;

    // — Show attempt progress in message field
    {
      char buf[48];
      snprintf(buf, sizeof(buf), "Sample %u/%u", (unsigned)attemptIndex, (unsigned)maxMetingen);
      setMessageText(String(buf));
      Serial.printf("%s\n", buf);
    }

    // — Start attempt timeout + polling deterministically
    RESTART_TIMER(tAttemptTimeout);
    RESTART_TIMER(tSpsPoll);

    mainState = STATE_MEASURE_POLL_READY;
    break;
  }

  case STATE_MEASURE_POLL_READY:
  {
    // — Attempt timeout
    if (DUE(tAttemptTimeout))
    {
      Serial.printf("Attempt timeout (no data-ready)\n");
      setMessageText("Timeout: no data");
      mainState = STATE_MEASURE_WAIT_NEXT;
      break;
    }

    // — Poll not faster than poll interval
    if (!DUE(tSpsPoll))
    {
      break;
    }

    uint16_t ready = 0;

    // — Read data-ready flag
    if (sps30_read_data_ready(&ready) != 0)
    {
      Serial.printf("WARN: sps30_read_data_ready() failed\n");
      break;
    }

    if (ready == 0)
    {
      break;
    }

    struct sps30_measurement m;

    // — Read measurement
    if (sps30_read_measurement(&m) != 0)
    {
      Serial.printf("WARN: sps30_read_measurement() failed\n");
      setMessageText("Read failed");
      mainState = STATE_MEASURE_WAIT_NEXT;
      break;
    }

    float pm1 = m.mc_1p0;
    float pm25 = m.mc_2p5;
    float pm10 = m.mc_10p0;

    // — Always show current measurement values on the fixed PM fields
    setPm1Text(formatPmLine(pm1));
    setPm25Text(formatPmLine(pm25));
    setPm10Text(formatPmLine(pm10));

    Serial.printf("Sample %u: PM1=%.1f  PM2.5=%.1f  PM10=%.1f\n",
                  (unsigned)attemptIndex, pm1, pm25, pm10);

    // — Validate and accumulate
    if (isValidSample(pm1, pm25, pm10))
    {
      sumPm1 += pm1;
      sumPm25 += pm25;
      sumPm10 += pm10;
      validCount++;

      // — Show valid counter in message field
      {
        char buf[48];
        snprintf(buf, sizeof(buf), "Valid: %u/%u", (unsigned)validCount, (unsigned)attemptIndex);
        setMessageText(String(buf));
      }
    }
    else
    {
      Serial.printf("Invalid sample rejected\n");
      setMessageText("Invalid sample");
    }

    // — Wait for next sample slot
    mainState = STATE_MEASURE_WAIT_NEXT;
    break;
  }

  case STATE_MEASURE_WAIT_NEXT:
  {
    // — No work; next loop returns to start attempt which waits on sample interval
    mainState = STATE_MEASURE_START_ATTEMPT;
    break;
  }

  case STATE_SHOW_RESULTS:
  {
    // — Show averages at the end (overwrite PM fields with "AVG" suffix)
    setPm1Text(formatAvgLine(avgPm1, "AVG"));
    setPm25Text(formatAvgLine(avgPm25, "AVG"));
    setPm10Text(formatAvgLine(avgPm10, "AVG"));

    // — Show status derived from PM2.5 average
    {
      AirStatus st = classifyPm25(avgPm25);
      String msg = String("Done: ") + String(statusText(st));
      setMessageText(msg);
      Serial.printf("Averages: PM1=%.1f  PM2.5=%.1f  PM10=%.1f  STATUS=%s\n",
                    avgPm1, avgPm25, avgPm10, statusText(st));
    }

    // — Stop sensor before powering down
    sps30Stop();

    mainState = STATE_POWER_DOWN;
    break;
  }

  case STATE_ERROR:
  {
    // — Stop sensor in error case too
    sps30Stop();
    mainState = STATE_POWER_DOWN;
    break;
  }

  case STATE_POWER_DOWN:
  default:
  {
    switchOff();
    break;
  }
  }
}
