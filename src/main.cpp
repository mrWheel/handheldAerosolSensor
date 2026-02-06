/*** Last Changed: 2026-02-06 - 14:04 ***/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>

#include <sps30.h>

#include <safeTimers.h>

const char* PROG_VERSION = "v1.0.1";

// ===================== User configuration =====================

// Latch control pin (drives the 2N7000 gate through a series resistor)
static const uint8_t pinLatch = GPIO_PIN_LATCH;

// Pushbutton pin to release the latch or do other things
static const uint8_t pinSwitch = GPIO_PIN_SWITCH;

// Battery divider ADC input (100k/100k => ADC sees Vbat/2)
static const uint8_t pinBatAdc = GPIO_PIN_BAT_ADC;

// E-paper pins (SPI on Nano: MOSI=D11, SCK=D13 are fixed by hardware SPI)
static const uint8_t pinEpdCs = GPIO_PIN_EPD_CS;
static const uint8_t pinEpdDc = GPIO_PIN_EPD_DC;
static const uint8_t pinEpdRst = GPIO_PIN_EPD_RST;
static const uint8_t pinEpdBusy = GPIO_PIN_EPD_BUSY;

// Warm-up time after starting SPS30 measurement
static const uint16_t warmupSeconds = WARMUP_SECONDS;

// Maximum number of measurement attempts (only valid reads are averaged)
static const uint8_t maxMetingen = MAX_METINGEN;

// ADC reference voltage (Nano uses Vcc as AREF by default; in your case this is the boosted 5V rail)
static const float adcVref = (float)ADC_VREF_VOLTAGE;

// Timing (non-blocking)
static const uint32_t warmupTickMs = 1000UL;
static const uint32_t sampleIntervalMs = 1000UL;
static const uint32_t sps30AttemptTimeoutMs = 7000UL;
static const uint32_t sps30PollIntervalMs = 100UL;

// Switch handling
static const uint32_t switchDebounceMs = 30UL;
static const uint32_t switchLongPressMs = 5000UL;

// ---- safeTimers ----
DECLARE_TIMER_MS(tWarmupTick, warmupTickMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tSampleInterval, sampleIntervalMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tAttemptTimeout, sps30AttemptTimeoutMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tSpsPoll, sps30PollIntervalMs, SKIP_MISSED_TICKS);

DECLARE_TIMER_MS(tSwitchDebounce, switchDebounceMs, SKIP_MISSED_TICKS);
DECLARE_TIMER_MS(tLongPress, switchLongPressMs, SKIP_MISSED_TICKS);

// ===================== Display selection =====================
//
// 1.54" 200x200 B/W "D67"
static const uint16_t MAX_DISPLAY_BUFFER_SIZE = 800;
static const uint16_t PAGE_HEIGHT = (MAX_DISPLAY_BUFFER_SIZE / (200 / 8)); // 32

//-- Optie A:
// GxEPD2_BW<GxEPD2_154, PAGE_HEIGHT> display(
//  GxEPD2_154(pinEpdCs, pinEpdDc, pinEpdRst, pinEpdBusy)
//);
//-- Optie B: screen inits .. but nothing shows
GxEPD2_BW<GxEPD2_154_T8, PAGE_HEIGHT> display(
    GxEPD2_154_T8(pinEpdCs, pinEpdDc, pinEpdRst, pinEpdBusy));
//-- Optie C: does not work
// GxEPD2_BW<GxEPD2_154_D67, PAGE_HEIGHT> display(
//    GxEPD2_154_D67(pinEpdCs, pinEpdDc, pinEpdRst, pinEpdBusy)
//);

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

// ===================== Battery reading =====================

static float readBatteryVoltage()
{
  uint16_t raw = analogRead(pinBatAdc);
  float vAdc = (raw * adcVref) / 1023.0f;
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

// ===================== E-paper helpers =====================

static bool hasPartialUpdate()
{
#if defined(GxEPD2_HAS_FAST_PARTIAL_UPDATE)
  return display.epd2.hasFastPartialUpdate;
#else
  return display.epd2.hasFastPartialUpdate;
#endif
} // hasPartialUpdate()

static void epdResetAndInit()
{
  SPI.begin();
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0)); // 250 kHz

  display.init(115200, true, 2, false);
  display.setRotation(1);
  display.setFullWindow();

  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
  } while (display.nextPage());

} // epdResetAndInit()

static void drawHeader(const char* title)
{
  display.setTextColor(GxEPD_BLACK);

  display.setFont(&FreeMonoBold12pt7b);
  display.setCursor(10, 25);
  display.print(title);

  display.setFont(&FreeMonoBold9pt7b);
} // drawHeader()

static void showMessageFull(const char* line1, const char* line2)
{
  display.setFullWindow();

  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);

    drawHeader("AIR MONITOR");

    display.setCursor(10, 60);
    display.print(line1);

    if (line2 != nullptr)
    {
      display.setCursor(10, 85);
      display.print(line2);
    }
  } while (display.nextPage());
} // showMessageFull()

static void showMessagePartial(const char* line1, const char* line2)
{
  display.setPartialWindow(0, 40, display.width(), display.height() - 40);

  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);

    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);

    display.setCursor(10, 60);
    display.print(line1);

    if (line2 != nullptr)
    {
      display.setCursor(10, 85);
      display.print(line2);
    }
  } while (display.nextPage());
} // showMessagePartial()

static void showStatusLine(const char* text)
{
  // if (hasPartialUpdate())
  //{
  //   showMessagePartial(text, nullptr);
  // }
  // else
  {
    showMessageFull(text, nullptr);
  }
} // showStatusLine()

static void showMeasurementProgress(uint8_t attempt, uint8_t total)
{
  char line[32];
  snprintf(line, sizeof(line), "meting %u / %u", attempt, total);
  showStatusLine(line);
  Serial.println(line);
} // showMeasurementProgress()

static void showResults(float pm1, float pm25, float pm10, float vbat, uint8_t pct, AirStatus st, uint8_t validCount, uint8_t attempts)
{
  display.setFullWindow();

  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);

    drawHeader("RESULTAAT");

    display.setCursor(10, 50);
    display.print("BAT: ");
    display.print(vbat, 2);
    display.print("V ");
    display.print(pct);
    display.print("%");

    display.setCursor(10, 75);
    display.print("STATUS: ");
    display.print(statusText(st));

    display.setCursor(10, 105);
    display.print("PM1.0: ");
    display.print(pm1, 1);

    display.setCursor(10, 130);
    display.print("PM2.5: ");
    display.print(pm25, 1);

    display.setCursor(10, 155);
    display.print("PM10 : ");
    display.print(pm10, 1);

    display.setCursor(10, 185);
    display.print("avg: ");
    display.print(validCount);
    display.print("/");
    display.print(attempts);
    display.print(" valid");
  } while (display.nextPage());

  Serial.println("\n------------- Results -------------\n");
  Serial.print("BAT: ");
  Serial.print(vbat, 2);
  Serial.print(" V ");
  Serial.print(pct);
  Serial.println("%");

  Serial.print("STATUS: ");
  Serial.println(statusText(st));

  Serial.print("PM1.0: ");
  Serial.println(pm1, 1);
  Serial.print("PM2.5: ");
  Serial.println(pm25, 1);
  Serial.print("PM10 : ");
  Serial.println(pm10, 1);

  Serial.print("avg: ");
  Serial.print(validCount);
  Serial.print(" / ");
  Serial.println(attempts);
  Serial.println(" valid samples");

} // showResults()

static void showError(const char* line1, const char* line2)
{
  display.setFullWindow();

  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);

    drawHeader("ERROR");

    display.setCursor(10, 70);
    display.print(line1);

    if (line2 != nullptr)
    {
      display.setCursor(10, 95);
      display.print(line2);
    }
  } while (display.nextPage());

  Serial.print("ERROR: ");
  Serial.print(line1);
  if (line2 != nullptr)
  {
    Serial.print(" / ");
    Serial.print(line2);
  }
  Serial.println();

} // showError()

// ===================== SPS30 helpers =====================

static bool sps30InitAndStart()
{
  //-- GPIO_PIN_SPS30_SDA, GPIO_PIN_SPS30_SCL
  Wire.begin();

  if (sps30_probe() != 0)
  {
    return false;
  }

  if (sps30_start_measurement() != 0)
  {
    return false;
  }

  return true;

} // sps30InitAndStart()

static void sps30Stop()
{
  (void)sps30_stop_measurement();

} // sps30Stop()

static bool isValidSample(float pm1, float pm25, float pm10)
{
  if (!isfinite(pm1) || !isfinite(pm25) || !isfinite(pm10))
  {
    return false;
  }

  if (pm1 < 0.0f || pm25 < 0.0f || pm10 < 0.0f)
  {
    return false;
  }

  if (pm1 > 2000.0f || pm25 > 2000.0f || pm10 > 2000.0f)
  {
    return false;
  }

  return true;
} // isValidSample()

// ===================== Power off =====================

void switchOff()
{
  float vbat = readBatteryVoltage();
  uint8_t pct = batteryPercentFromVoltage(vbat);

  Serial.print(F("Battery voltage: "));
  Serial.print(vbat, 2);
  Serial.print(F(" V ("));
  Serial.print(pct);
  Serial.println(F("%)"));

  Serial.println(F("Switching off..."));
  Serial.flush();

  for (int i = 0; i < 5; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1250);
  }

  digitalWrite(pinLatch, LOW);

  while (true)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
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

// Warmup
static uint16_t warmupRemaining = 0;
static uint32_t warmupLastTickMs = 0;

// Measurement accumulation
static float sumPm1 = 0.0f;
static float sumPm25 = 0.0f;
static float sumPm10 = 0.0f;
static uint8_t validCount = 0;
static uint8_t attemptIndex = 0;

// Attempt timing
static uint32_t attemptStartMs = 0;
static uint32_t lastPollMs = 0;
static uint32_t nextSampleDueMs = 0;

// Results data
static float avgPm1 = 0.0f;
static float avgPm25 = 0.0f;
static float avgPm10 = 0.0f;

// Switch tracking
static bool switchStableState = HIGH;
static bool switchLastReading = HIGH;
//-x-static uint32_t switchLastChangeMs = 0;
//-x-static uint32_t switchPressStartMs = 0;

static void updateSwitch()
{
  bool reading = (digitalRead(pinSwitch) == LOW) ? LOW : HIGH;

  // Edge detect: zodra reading verandert, start debounce-timer opnieuw
  if (reading != switchLastReading)
  {
    switchLastReading = reading;
    RESTART_TIMER(tSwitchDebounce);
  }

  // Als debounce-tijd voorbij is, accepteer de nieuwe stabiele state
  if (DUE(tSwitchDebounce))
  {
    if (switchStableState != switchLastReading)
    {
      switchStableState = switchLastReading;

      if (switchStableState == LOW)
      {
        // Press
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        RESTART_TIMER(tLongPress);
      }
      else
      {
        // Release: geen long-press meer actief (optioneel: restart zodat 'ie niet meteen due kan zijn)
        RESTART_TIMER(tLongPress);
      }
    }
  }

  // Long-press detect (alleen terwijl knop ingedrukt blijft)
  if (switchStableState == LOW)
  {
    if (DUE(tLongPress))
    {
      Serial.println(F("Long press detected -> switchOff()"));
      switchOff();
    }
  }
} // updateSwitch()

// ===================== Arduino entrypoints =====================

void setup()
{
  pinMode(pinLatch, OUTPUT);
  digitalWrite(pinLatch, HIGH);

  Serial.begin(115200);
  delay(20);
  Serial.flush();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pinSwitch, INPUT_PULLUP);

  pinMode(pinBatAdc, INPUT);
  digitalWrite(pinBatAdc, LOW);
  analogRead(pinBatAdc); // Discard first reading
  delayMicroseconds(50);
  uint16_t raw = analogRead(pinBatAdc);

  pinMode(6, OUTPUT);
  digitalWrite(6, LOW); // shifter uit (Hi-Z) tijdens boot

  delay(20);             // rails even stabiliseren
  digitalWrite(6, HIGH); // shifter aan

  pinMode(pinEpdCs, OUTPUT);
  digitalWrite(pinEpdCs, HIGH);
  pinMode(pinEpdBusy, INPUT_PULLUP);
  delay(20);
  Serial.print(F("BUSY stable (idle) after 20ms:"));
  Serial.flush();
  Serial.println(digitalRead(pinEpdBusy));
  Serial.flush();

  Serial.println(F("\n\nAnd then it starts ...\n"));
  Serial.print(F("Firmware version: "));
  Serial.println(PROG_VERSION);
  Serial.print(F("Warm-up time: "));
  Serial.print(warmupSeconds);
  Serial.println(F(" seconds"));
  Serial.print(F("Max measurements: "));
  Serial.println(maxMetingen);
  Serial.print(F("PIN_LATCH: "));
  Serial.println(pinLatch);
  Serial.print(F("PIN_SWITCH: "));
  Serial.println(pinSwitch);
  Serial.print(F("PIN_BAT_ADC: "));
  Serial.println(pinBatAdc);
  Serial.flush();

  analogReference(DEFAULT);
  delay(10);

  Serial.println(F("Initializing e-paper..."));
  pinMode(pinEpdBusy, INPUT);
  Serial.print(F("BUSY (idle) before init: "));
  Serial.println(digitalRead(pinEpdBusy));

  pinMode(pinEpdRst, OUTPUT);
  digitalWrite(pinEpdRst, LOW);
  delay(10);
  digitalWrite(pinEpdRst, HIGH);
  delay(10);

  Serial.print(F("BUSY after manual reset pulse: "));
  Serial.println(digitalRead(pinEpdBusy));

  epdResetAndInit();
  showMessageFull("HELLO", "e-paper test");
  //  while(true) { delay(1000); }

  showMessageFull("power latched", nullptr);
  Serial.println(F("Power latched."));

  if (!sps30InitAndStart())
  {
    showError("SPS30 not found", "check I2C wiring");
    mainState = STATE_ERROR;
  }
  else
  {
    warmupRemaining = warmupSeconds;
    warmupLastTickMs = millis();

    showMessageFull("power latched", "warming up...");
    Serial.println("Warming up...");
    mainState = STATE_WARMUP;
  }

  // ---- safeTimers: start deterministically (override initial random staggering) ----
  RESTART_TIMER(tWarmupTick);
  RESTART_TIMER(tSampleInterval);
  RESTART_TIMER(tAttemptTimeout);
  RESTART_TIMER(tSpsPoll);
  RESTART_TIMER(tSwitchDebounce);
  RESTART_TIMER(tLongPress);
  // Ensure known switch states
  switchStableState = (digitalRead(pinSwitch) == LOW) ? LOW : HIGH;
  switchLastReading = switchStableState;

} // setup()

void loop()
{
  updateSwitch();

  switch (mainState)
  {
  case STATE_WARMUP:
  {
    if (warmupRemaining == 0)
    {
      attemptIndex = 0;
      validCount = 0;
      sumPm1 = 0.0f;
      sumPm25 = 0.0f;
      sumPm10 = 0.0f;

      // Start meetcyclus "clean" (zonder random offset)
      RESTART_TIMER(tSampleInterval);

      mainState = STATE_MEASURE_START_ATTEMPT;
      break;
    }

    // 1Hz warmup tick
    if (DUE(tWarmupTick))
    {
      warmupRemaining--;

      char line[32];
      snprintf(line, sizeof(line), "warming up: %us", warmupRemaining);
      Serial.println(line);

      if (hasPartialUpdate())
        showMessagePartial(line, nullptr);
      else
        showMessageFull(line, nullptr);
    }
    break;
  }

  case STATE_MEASURE_START_ATTEMPT:
  {
    if (attemptIndex >= maxMetingen)
    {
      if (validCount == 0)
      {
        showError("no valid samples", "check sensor");
        mainState = STATE_ERROR;
        break;
      }

      avgPm1 = sumPm1 / (float)validCount;
      avgPm25 = sumPm25 / (float)validCount;
      avgPm10 = sumPm10 / (float)validCount;

      mainState = STATE_SHOW_RESULTS;
      break;
    }

    // Wacht tot volgende sample-slot
    if (!DUE(tSampleInterval))
    {
      break;
    }

    attemptIndex++;
    showMeasurementProgress(attemptIndex, maxMetingen);

    // Start attempt timeout + polling deterministisch
    RESTART_TIMER(tAttemptTimeout);
    RESTART_TIMER(tSpsPoll);

    mainState = STATE_MEASURE_POLL_READY;
    break;
  }

  case STATE_MEASURE_POLL_READY:
  {
    // Attempt timeout
    if (DUE(tAttemptTimeout))
    {
      Serial.println("Attempt timeout (no data-ready).");
      // Volgende attempt start pas bij volgende tSampleInterval tick
      mainState = STATE_MEASURE_WAIT_NEXT;
      break;
    }

    // Poll niet sneller dan poll-interval
    if (!DUE(tSpsPoll))
    {
      break;
    }

    uint16_t ready = 0;
    if (sps30_read_data_ready(&ready) != 0)
    {
      // Keep polling until timeout
      break;
    }

    if (ready == 0)
    {
      break;
    }

    struct sps30_measurement m;
    if (sps30_read_measurement(&m) != 0)
    {
      Serial.println("Read measurement failed.");
      mainState = STATE_MEASURE_WAIT_NEXT;
      break;
    }

    float pm1 = m.mc_1p0;
    float pm25 = m.mc_2p5;
    float pm10 = m.mc_10p0;

    if (isValidSample(pm1, pm25, pm10))
    {
      sumPm1 += pm1;
      sumPm25 += pm25;
      sumPm10 += pm10;
      validCount++;

      Serial.print(F("Valid sample: "));
      Serial.print(pm1, 1);
      Serial.print(F(" / "));
      Serial.print(pm25, 1);
      Serial.print(F(" / "));
      Serial.println(pm10, 1);
    }
    else
    {
      Serial.println(F("Invalid sample rejected."));
    }

    // We wachten nu op de volgende sample-interval tick
    mainState = STATE_MEASURE_WAIT_NEXT;
    break;
  }

  case STATE_MEASURE_WAIT_NEXT:
  {
    // Niets te doen: STATE_MEASURE_START_ATTEMPT wacht op DUE(tSampleInterval)
    mainState = STATE_MEASURE_START_ATTEMPT;
    break;
  }

  case STATE_SHOW_RESULTS:
  {
    float vbat = readBatteryVoltage();
    uint8_t pct = batteryPercentFromVoltage(vbat);
    AirStatus st = classifyPm25(avgPm25);

    showResults(avgPm1, avgPm25, avgPm10, vbat, pct, st, validCount, maxMetingen);

    sps30Stop();
    mainState = STATE_POWER_DOWN;
    break;
  }

  case STATE_ERROR:
  {
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