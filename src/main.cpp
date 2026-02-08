/*** Last Changed: 2026-02-08 - 12:01 ***/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

//--ESP32 radio control to disable WiFi/Bluetooth as early as possible
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_bt.h>

#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>

#include <sps30.h>
#include <safeTimers.h>

//--Program version string (keep manually updated with each release)
//-- NEVER CHANGE THIS const char* NAME
//--             vvvvvvvvvvvvvv
static const char* PROG_VERSION = "v0.3.0";
//--             ^^^^^^^^^^^^^^

// ===================== User configuration (from build_flags) =====================

//--Latch control pin (drives the 2N7000 gate through a series resistor)
static const uint8_t pinLatch = GPIO_PIN_LATCH;

//--Pushbutton pin (LOW = pressed with INPUT_PULLUP)
static const uint8_t pinSwitch = GPIO_PIN_SWITCH;

//--Battery divider ADC input (default math assumes 2:1 divider => ADC sees Vbat/2)
static const uint8_t pinBatAdc = GPIO_PIN_BAT_ADC;

//--E-paper SPI pins (from build_flags)
static const int8_t pinEpdMosi = GPIO_PIN_EPD_MOSI;
static const int8_t pinEpdMiso = GPIO_PIN_EPD_MISO;
static const int8_t pinEpdSck = GPIO_PIN_EPD_SCK;

//--E-paper control pins
static const uint8_t pinEpdCs = GPIO_PIN_EPD_CS;
static const uint8_t pinEpdDc = GPIO_PIN_EPD_DC;
static const uint8_t pinEpdRst = GPIO_PIN_EPD_RST;
static const uint8_t pinEpdBusy = GPIO_PIN_EPD_BUSY;

//--SPS30 I2C pins
static const uint8_t pinSps30Sda = GPIO_PIN_SPS30_SDA;
static const uint8_t pinSps30Scl = GPIO_PIN_SPS30_SCL;
// — Use UART2 on remapped pins (ESP32 allows routing UART signals to most GPIOs)
static const int8_t pinSpsUartTx = GPIO_PIN_SPS_UART_TX;
static const int8_t pinSpsUartRx = GPIO_PIN_SPS_UART_RX;

static HardwareSerial spsUart(2);

static void sps30UartPassthroughInit()
{
  // — SPS30 UART: 115200 8N1
  spsUart.begin(115200, SERIAL_8N1, pinSpsUartRx, pinSpsUartTx);

  Serial.printf("SPS30 UART passthrough on UART2: RX=%d TX=%d\n",
                (int)pinSpsUartRx, (int)pinSpsUartTx);

} // sps30UartPassthroughInit()

static void sps30UartPassthroughLoop()
{
  while (spsUart.available() > 0)
  {
    int c = spsUart.read();
    Serial.write((uint8_t)c);
  }

} // sps30UartPassthroughLoop()

//--Warm-up time after starting SPS30 measurement
static const uint16_t warmupSeconds = WARMUP_SECONDS;

//--Maximum number of measurement attempts (only valid reads are averaged)
static const uint8_t maxMetingen = MAX_METINGEN;

//--ADC reference voltage used for battery math (calibrate for best accuracy)
static const float adcVref = (float)ADC_VREF_VOLTAGE;

// ===================== Timing (safeTimers) =====================

//--UI + sampling cadence
static const uint32_t warmupTickMs = 1000UL;
static const uint32_t sampleIntervalMs = 1000UL;
static const uint32_t sps30AttemptTimeoutMs = 7000UL;
static const uint32_t sps30PollIntervalMs = 100UL;

//--Switch handling
static const uint32_t switchDebounceMs = 30UL;
static const uint32_t switchLongPressMs = 2000UL;

//--safeTimers declarations
DECLARE_TIMER_MS(tWarmupTick, warmupTickMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tSampleInterval, sampleIntervalMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tAttemptTimeout, sps30AttemptTimeoutMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tSpsPoll, sps30PollIntervalMs, SKIP_MISSED_TICKS);

DECLARE_TIMER_MS(tSwitchDebounce, switchDebounceMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tLongPress, switchLongPressMs, SKIP_MISSED_TICKS);

// ===================== Display selection =====================

//--1.54" 200x200 B/W display; full height buffer is fine on ESP32
static const uint16_t pageHeight = 200;

//--Display driver instance
GxEPD2_BW<GxEPD2_154, pageHeight> display(
    GxEPD2_154(pinEpdCs, pinEpdDc, pinEpdRst, pinEpdBusy));

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

//--Fixed positions for TITLE, Battery, PM lines and message area (safe for 200x200, rotation=1)
static const UiLayout ui =
    {
        8,  // xLabel
        86, // xValue

        30, // yTitle baseline (12pt)

        62,  // yBattery baseline (9pt)
        92,  // yPm1 baseline
        122, // yPm25 baseline
        152, // yPm10 baseline

        188 // yMessage baseline (9pt)
};

//--UI text fields
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

} // classifyPm25()

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

} // statusText()

// ===================== ESP32 radio shutoff =====================

static void disableRadiosAsap()
{
  //--Ensure Arduino WiFi is off
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);

  //--Stop WiFi driver (ignore errors if not started yet)
  (void)esp_wifi_stop();
  (void)esp_wifi_deinit();

  //--Stop Bluetooth controller if present
  btStop();
  (void)esp_bt_controller_disable();
  (void)esp_bt_controller_deinit();

} // disableRadiosAsap()

// ===================== Battery reading =====================

static float readBatteryVoltage()
{
  //--ESP32 ADC is 12-bit (0..4095) when analogReadResolution(12) is used
  uint16_t raw = (uint16_t)analogRead(pinBatAdc);

  //--Convert ADC reading to voltage at the ADC pin
  float vAdc = ((float)raw * adcVref) / 4095.0f;

  //--Default assumes a 2:1 divider (ADC sees Vbat/2)
  return vAdc * 2.0f;

} // readBatteryVoltage()

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

} // batteryPercentFromVoltage()

static String formatBatteryLine(float vbat, uint8_t pct)
{
  //--Format battery line using snprintf then convert to String
  char buf[48];

  //--Example: "4.02V 78%"
  snprintf(buf, sizeof(buf), "%.2fV %u%%", vbat, pct);

  return String(buf);

} // formatBatteryLine()

// ===================== E-paper helpers =====================

static void epdInit()
{
  //--Configure control pins
  pinMode(pinEpdCs, OUTPUT);
  digitalWrite(pinEpdCs, HIGH);

  pinMode(pinEpdDc, OUTPUT);
  digitalWrite(pinEpdDc, LOW);

  pinMode(pinEpdRst, OUTPUT);
  digitalWrite(pinEpdRst, HIGH);

  //--BUSY input with pull-up (board is open drain)
  pinMode(pinEpdBusy, INPUT_PULLUP);

  //--Map SPI pins explicitly (MISO may be -1)
  SPI.begin(pinEpdSck, pinEpdMiso, pinEpdMosi, pinEpdCs);

  //--Manual reset pulse (some boards need this)
  digitalWrite(pinEpdRst, LOW);
  delay(20);
  digitalWrite(pinEpdRst, HIGH);
  delay(50);

  //--Log BUSY around init
  Serial.printf("EPD BUSY before init: %d\n", digitalRead(pinEpdBusy));

  //--Init display (GxEPD2 controls SPI transactions internally)
  display.init(115200, true, 2, false);

  Serial.printf("EPD init returned\n");

  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);
  display.setFullWindow();

} // epdInit()

static void uiDrawAllFull()
{
  //--Full redraw avoids partial-window artifacts during bring-up
  display.setFullWindow();

  display.firstPage();
  do
  {
    //--Clear background
    display.fillScreen(GxEPD_WHITE);

    //--Title
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMonoBold12pt7b);
    display.setCursor(ui.xLabel, ui.yTitle);
    display.print("AIR MONITOR");

    //--Rows
    display.setFont(&FreeMonoBold9pt7b);

    display.setCursor(ui.xLabel, ui.yBattery);
    display.print("BAT:");
    display.setCursor(ui.xValue, ui.yBattery);
    display.print(batteryText);

    display.setCursor(ui.xLabel, ui.yPm1);
    display.print("PM1.0:");
    display.setCursor(ui.xValue, ui.yPm1);
    display.print(pm1Text);

    display.setCursor(ui.xLabel, ui.yPm25);
    display.print("PM2.5:");
    display.setCursor(ui.xValue, ui.yPm25);
    display.print(pm25Text);

    display.setCursor(ui.xLabel, ui.yPm10);
    display.print("PM10 :");
    display.setCursor(ui.xValue, ui.yPm10);
    display.print(pm10Text);

    //--Message (full width; keep it short)
    display.setCursor(ui.xLabel, ui.yMessage);
    display.print(messageText);

  } while (display.nextPage());

} // uiDrawAllFull()

// ===================== SPS30 helpers =====================
static void i2cScan()
{
  Serial.printf("I2C scan start\n");

  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      Serial.printf("I2C device found at 0x%02X\n", addr);
    }
  }

  Serial.printf("I2C scan done\n");

} // i2cScan()

static bool sps30InitAndStart()
{
  //--Initialize I2C on explicit pins
  Wire.begin(pinSps30Sda, pinSps30Scl);
  delay(1000);           // short delay after Wire.begin() before setting clock
  Wire.setClock(100000); // or 400000 if SPS30 is happy
  delay(5000);
  i2cScan();
  //--Probe sensor
  if (sps30_probe() != 0)
  {
    return false;
  }

  //--Start measurement mode
  if (sps30_start_measurement() != 0)
  {
    return false;
  }

  return true;

} // sps30InitAndStart()

static void sps30Stop()
{
  //--Stop measurement to reduce power
  (void)sps30_stop_measurement();

} // sps30Stop()

static bool isValidSample(float pm1, float pm25, float pm10)
{
  //--Reject NaN/Inf
  if (!isfinite(pm1) || !isfinite(pm25) || !isfinite(pm10))
  {
    return false;
  }

  //--Reject negative values
  if (pm1 < 0.0f || pm25 < 0.0f || pm10 < 0.0f)
  {
    return false;
  }

  //--Reject obviously out-of-range values
  if (pm1 > 2000.0f || pm25 > 2000.0f || pm10 > 2000.0f)
  {
    return false;
  }

  return true;

} // isValidSample()

static String formatPmLine(float v)
{
  //--Format PM value as "123.4"
  char buf[16];
  snprintf(buf, sizeof(buf), "%.1f", v);
  return String(buf);

} // formatPmLine()

static String formatAvgLine(float v, const char* suffix)
{
  //--Format average line as "123.4 <suffix>" (e.g. "12.3 AVG")
  char buf[24];
  snprintf(buf, sizeof(buf), "%.1f %s", v, suffix);
  return String(buf);

} // formatAvgLine()

// ===================== Power off =====================

static void updateBatteryNow(const char* reason)
{
  //--Read and render battery line immediately
  float vbat = readBatteryVoltage();
  uint8_t pct = batteryPercentFromVoltage(vbat);

  batteryText = formatBatteryLine(vbat, pct);

  //--Log battery status
  Serial.printf("Battery (%s): %.2f V (%u%%)\n", reason, vbat, pct);

} // updateBatteryNow()

static void switchOff()
{
  //--Update battery reading right before unlatching power
  updateBatteryNow("pre-off");

  //--Show message before power-down
  messageText = "Switching off...";
  uiDrawAllFull();

  //--Log and flush
  Serial.printf("Switching off latch (PIN %u)\n", (unsigned)pinLatch);
  Serial.flush();

  //--Release the latch (hardware should cut power)
  digitalWrite(pinLatch, LOW);

  //--Fallback loop if power does not drop (debug aid)
  while (true)
  {
    delay(100);
  }

} // switchOff()

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

//--Warmup remaining seconds
static uint16_t warmupRemaining = 0;

//--Measurement accumulation
static float sumPm1 = 0.0f;
static float sumPm25 = 0.0f;
static float sumPm10 = 0.0f;
static uint8_t validCount = 0;
static uint8_t attemptIndex = 0;

//--Results data
static float avgPm1 = 0.0f;
static float avgPm25 = 0.0f;
static float avgPm10 = 0.0f;

//--Switch tracking
static bool switchStableState = HIGH;
static bool switchLastReading = HIGH;

static void updateSwitch()
{
  //--Read switch with pull-up logic (LOW means pressed)
  bool reading = (digitalRead(pinSwitch) == LOW) ? LOW : HIGH;

  //--On edge change, restart debounce timer
  if (reading != switchLastReading)
  {
    switchLastReading = reading;
    RESTART_TIMER(tSwitchDebounce);
    Serial.println("Reading changed -> Restart tSwitchDebounce timer");
  }

  //--When debounce expires, accept new stable state
  if (DUE(tSwitchDebounce))
  {
    if (switchStableState != switchLastReading)
    {
      switchStableState = switchLastReading;

      if (switchStableState == LOW)
      {
        //--Press: start long-press timer
        RESTART_TIMER(tLongPress);
      }
      else
      {
        //--Release: restart long-press timer so it cannot instantly fire
        RESTART_TIMER(tLongPress);
      }
    }
  }

  //--Long-press detection while held down
  if (switchStableState == LOW)
  {
    if (DUE(tLongPress))
    {
      switchOff();
    }
  }

} // updateSwitch()

// ===================== Arduino entrypoints =====================

void setup()
{
  //--Disable radios immediately to reduce current draw and RF noise
  disableRadiosAsap();

  //--Latch power immediately
  pinMode(pinLatch, OUTPUT);
  digitalWrite(pinLatch, HIGH);

  //--Start serial logging
  Serial.begin(115200);
  delay(20);

  //--Configure basic IO
  pinMode(pinSwitch, INPUT_PULLUP);

  //--Configure ADC input and resolution
  pinMode(pinBatAdc, INPUT);
  analogReadResolution(12);

  //--Discard first ADC read to settle
  (void)analogRead(pinBatAdc);

  sps30UartPassthroughInit();

  //--Boot logs
  Serial.printf("\n\nAnd then it starts ...\n\n");
  Serial.printf("Firmware version: %s\n", PROG_VERSION);
  Serial.printf("Warm-up time: %u seconds\n", (unsigned)warmupSeconds);
  Serial.printf("Max measurements: %u\n", (unsigned)maxMetingen);
  Serial.printf("PIN_LATCH: %u\n", (unsigned)pinLatch);
  Serial.printf("PIN_SWITCH: %u\n", (unsigned)pinSwitch);
  Serial.printf("PIN_BAT_ADC: %u\n", (unsigned)pinBatAdc);
  Serial.printf("I2C SDA/SCL: %u/%u\n", (unsigned)pinSps30Sda, (unsigned)pinSps30Scl);

  //--Initialize e-paper
  Serial.printf("Initializing e-paper...\n");
  epdInit();

  //--Initialize UI strings
  batteryText.reserve(32);
  pm1Text.reserve(16);
  pm25Text.reserve(16);
  pm10Text.reserve(16);
  messageText.reserve(64);

  //--Initial placeholders
  batteryText = "-";
  pm1Text = "-";
  pm25Text = "-";
  pm10Text = "-";
  messageText = "Booting...";

  //--Read battery at boot (even if not connected)
  updateBatteryNow("boot");

  //--Draw initial UI
  uiDrawAllFull();

  //--Initialize SPS30 and decide state
  if (!sps30InitAndStart())
  {
    Serial.printf("ERROR: SPS30 not found\n");
    messageText = "ERROR: SPS30 missing";
    uiDrawAllFull();
    delay(2000);
    mainState = STATE_ERROR;
    mainState = STATE_WARMUP;
  }
  else
  {
    warmupRemaining = warmupSeconds;
    Serial.printf("SPS30 OK. Warming up...\n");
    messageText = "Warming up...";
    uiDrawAllFull();
    mainState = STATE_WARMUP;
  }

  //--Start timers deterministically
  RESTART_TIMER(tWarmupTick);
  RESTART_TIMER(tSampleInterval);
  RESTART_TIMER(tAttemptTimeout);
  RESTART_TIMER(tSpsPoll);
  RESTART_TIMER(tSwitchDebounce);
  RESTART_TIMER(tLongPress);

  //--Initialize switch state tracking
  switchStableState = (digitalRead(pinSwitch) == LOW) ? LOW : HIGH;
  switchLastReading = switchStableState;
  updateSwitch();

} // setup()

void loop()
{
  updateSwitch();
  sps30UartPassthroughLoop();

  switch (mainState)
  {
  case STATE_WARMUP:
  {
    if (warmupRemaining == 0)
    {
      //--Reset accumulation at end of warmup
      attemptIndex = 0;
      validCount = 0;
      sumPm1 = 0.0f;
      sumPm25 = 0.0f;
      sumPm10 = 0.0f;

      //--Start measurement cycle cleanly
      RESTART_TIMER(tSampleInterval);

      messageText = "Measuring...";
      uiDrawAllFull();

      mainState = STATE_MEASURE_START_ATTEMPT;
      break;
    }

    //--1Hz warmup tick
    if (DUE(tWarmupTick))
    {
      warmupRemaining--;

      //--Show warmup remaining seconds
      messageText = String("Warmup: ") + String((unsigned)warmupRemaining) + "s";
      uiDrawAllFull();

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
        messageText = "ERROR: no valid samples";
        uiDrawAllFull();
        mainState = STATE_ERROR;
        break;
      }

      //--Compute averages
      avgPm1 = sumPm1 / (float)validCount;
      avgPm25 = sumPm25 / (float)validCount;
      avgPm10 = sumPm10 / (float)validCount;

      mainState = STATE_SHOW_RESULTS;
      break;
    }

    //--Wait until the next sample slot is due
    if (!DUE(tSampleInterval))
    {
      break;
    }

    attemptIndex++;

    //--Show attempt progress
    {
      char buf[48];
      snprintf(buf, sizeof(buf), "Sample %u/%u", (unsigned)attemptIndex, (unsigned)maxMetingen);
      messageText = String(buf);
      Serial.printf("%s\n", buf);
    }

    uiDrawAllFull();

    //--Start attempt timeout + polling deterministically
    RESTART_TIMER(tAttemptTimeout);
    RESTART_TIMER(tSpsPoll);

    mainState = STATE_MEASURE_POLL_READY;
    break;
  }

  case STATE_MEASURE_POLL_READY:
  {
    //--Attempt timeout
    if (DUE(tAttemptTimeout))
    {
      Serial.printf("Attempt timeout (no data-ready)\n");
      messageText = "Timeout: no data";
      uiDrawAllFull();
      mainState = STATE_MEASURE_WAIT_NEXT;
      break;
    }

    //--Poll not faster than poll interval
    if (!DUE(tSpsPoll))
    {
      break;
    }

    uint16_t ready = 0;

    //--Read data-ready flag
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

    //--Read measurement
    if (sps30_read_measurement(&m) != 0)
    {
      Serial.printf("WARN: sps30_read_measurement() failed\n");
      messageText = "Read failed";
      uiDrawAllFull();
      mainState = STATE_MEASURE_WAIT_NEXT;
      break;
    }

    float pm1 = m.mc_1p0;
    float pm25 = m.mc_2p5;
    float pm10 = m.mc_10p0;

    //--Always show current measurement values
    pm1Text = formatPmLine(pm1);
    pm25Text = formatPmLine(pm25);
    pm10Text = formatPmLine(pm10);

    Serial.printf("Sample %u: PM1=%.1f  PM2.5=%.1f  PM10=%.1f\n",
                  (unsigned)attemptIndex, pm1, pm25, pm10);

    //--Validate and accumulate
    if (isValidSample(pm1, pm25, pm10))
    {
      sumPm1 += pm1;
      sumPm25 += pm25;
      sumPm10 += pm10;
      validCount++;

      {
        char buf[48];
        snprintf(buf, sizeof(buf), "Valid: %u/%u", (unsigned)validCount, (unsigned)attemptIndex);
        messageText = String(buf);
      }
    }
    else
    {
      Serial.printf("Invalid sample rejected\n");
      messageText = "Invalid sample";
    }

    uiDrawAllFull();

    //--Wait for next sample slot
    mainState = STATE_MEASURE_WAIT_NEXT;
    break;
  }

  case STATE_MEASURE_WAIT_NEXT:
  {
    //--No work; next loop returns to start attempt which waits on sample interval
    mainState = STATE_MEASURE_START_ATTEMPT;
    break;
  }

  case STATE_SHOW_RESULTS:
  {
    //--Show averages at the end (overwrite PM fields with "AVG" suffix)
    pm1Text = formatAvgLine(avgPm1, "AVG");
    pm25Text = formatAvgLine(avgPm25, "AVG");
    pm10Text = formatAvgLine(avgPm10, "AVG");

    {
      AirStatus st = classifyPm25(avgPm25);
      messageText = String("Done: ") + String(statusText(st));
      Serial.printf("Averages: PM1=%.1f  PM2.5=%.1f  PM10=%.1f  STATUS=%s\n",
                    avgPm1, avgPm25, avgPm10, statusText(st));
    }

    uiDrawAllFull();

    //--Stop sensor before powering down
    sps30Stop();

    mainState = STATE_POWER_DOWN;
    break;
  }

  case STATE_ERROR:
  {
    //--Stop sensor in error case too
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

} // loop()
