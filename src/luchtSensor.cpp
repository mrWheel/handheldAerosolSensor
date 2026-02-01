#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>

#include <sps30.h>


const char* PROG_VERSION = "v0.1.0";

// ===================== User configuration =====================

// Latch control pin (drives the 2N7000 gate through a series resistor)
static const uint8_t PIN_LATCH = GPIO_PIN_LATCH;

// Pushbutton pin to release the latch or do other things
static const uint8_t PIN_SWITCH = GPIO_PIN_SWITCH;

// Battery divider ADC input (100k/100k => ADC sees Vbat/2)
static const uint8_t PIN_BAT_ADC = GPIO_PIN_BAT_ADC;

// E-paper pins (SPI on Nano: MOSI=D11, SCK=D13 are fixed by hardware SPI)
static const uint8_t PIN_EPD_CS = 10;
static const uint8_t PIN_EPD_DC = 9;
static const uint8_t PIN_EPD_RST = 8;
static const uint8_t PIN_EPD_BUSY = 7;

// Warm-up time after starting SPS30 measurement
static const uint16_t warmupSeconds = WARMUP_SECONDS;

// Maximum number of measurement attempts (only valid reads are averaged)
static const uint8_t maxMetingen = MAX_METINGEN;

// ADC reference voltage (Nano uses Vcc as AREF by default; in your case this is the boosted 5V rail)
static const float adcVref = (float)ADC_VREF_VOLTAGE;

// ===================== Display selection =====================
//
// Try the 1.54" 200x200 B/W "D67" first (common for Waveshare 1.54 V2 style).
// If you get a blank screen or wrong refresh behavior, comment it and try GxEPD2_154.
// Use a small page buffer so it fits in 2KB SRAM on ATmega328P.
// 200 px width => 200/8 = 25 bytes per row.
// With 800 bytes buffer => 800 / 25 = 32 rows per page.
static const uint16_t MAX_DISPLAY_BUFFER_SIZE = 800;
static const uint16_t PAGE_HEIGHT = (MAX_DISPLAY_BUFFER_SIZE / (200 / 8)); // 32

GxEPD2_BW<GxEPD2_154_D67, PAGE_HEIGHT> display(
  GxEPD2_154_D67(PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RST, PIN_EPD_BUSY)
);
//GxEPD2_BW<GxEPD2_154, PAGE_HEIGHT> display(
//  GxEPD2_154(PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RST, PIN_EPD_BUSY)
//);

// ===================== Air quality classification =====================

enum AirStatus
{
  STATUS_GEMIDDELD,
  STATUS_HOOG,
  STATUS_EXTREEM
};

// PM2.5 thresholds in µg/m³ (simple and practical)
static AirStatus classifyPM25(float pm2_5)
{
  if (pm2_5 < 35.0f)
  {
    return STATUS_GEMIDDELD;
  }

  if (pm2_5 < 75.0f)
  {
    return STATUS_HOOG;
  }

  return STATUS_EXTREEM;
} // classifyPM25()

static const char *statusText(AirStatus s)
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

// Read Li-ion battery voltage through a 100k/100k divider.
// Note: this assumes the divider top is on VBAT and the Nano ADC reference is the 5V rail.
static float readBatteryVoltage()
{
  uint16_t raw = analogRead(PIN_BAT_ADC);
  float v_adc = (raw * adcVref) / 1023.0f;
  return v_adc * 2.0f;
} // readBatteryVoltage()

// Simple percentage estimate from open-circuit-ish Li-ion voltage.
// Under load this is approximate; it's good enough as a UI indicator.
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

// Check whether this panel/driver combination supports fast partial updates.
// If not supported, partial updates will behave like full refresh (more flicker).
static bool hasPartialUpdate()
{
#if defined(GxEPD2_HAS_FAST_PARTIAL_UPDATE)
  return display.epd2.hasFastPartialUpdate;
#else
  return display.epd2.hasFastPartialUpdate;
#endif
} // hasPartialUpdate()

// Hard reset / initialize and clear the panel.
// This is the "reset e-paper" step you asked for.
static void epdResetAndInit()
{
  display.init(115200);
  display.setRotation(1);
  display.setFullWindow();

  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
  }
  while (display.nextPage());
} // epdResetAndInit()

// Draw a consistent header for all UI screens.
static void drawHeader(const char *title)
{
  display.setTextColor(GxEPD_BLACK);

  display.setFont(&FreeMonoBold12pt7b);
  display.setCursor(10, 25);
  display.print(title);

  display.setFont(&FreeMonoBold9pt7b);
} // drawHeader()

// Full refresh message screen.
// Use this when you want something guaranteed visible on any panel variant.
static void showMessageFull(const char *line1, const char *line2)
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
  }
  while (display.nextPage());
} // showMessageFull()

// Partial refresh message area (below the header).
// If partial refresh is available, this reduces flashing while updating text.
static void showMessagePartial(const char *line1, const char *line2)
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
  }
  while (display.nextPage());
} // showMessagePartial()

// Show a single-line status update while minimizing flicker when possible.
static void showStatusLine(const char *text)
{
  if (hasPartialUpdate())
  {
    showMessagePartial(text, nullptr);
  }
  else
  {
    showMessageFull(text, nullptr);
  }
} // showStatusLine()

// Warm-up countdown.
// We show a full screen once, then update the countdown area every second.
// If the panel doesn't support partial updates, you will see a full refresh flash each second.
static void showWarmupCountdown(uint16_t seconds)
{
  showMessageFull("power latched", "warming up...");

  for (int s = seconds; s > 0; s--)
  {
    char line[32];
    snprintf(line, sizeof(line), "warming up: %ds", s);
    Serial.println(line);

    if (hasPartialUpdate())
    {
      showMessagePartial(line, nullptr);
    }
    else
    {
      showMessageFull(line, nullptr);
    }

    delay(1000);
  }
} // showWarmupCountdown()

// Measurement progress indicator.
static void showMeasurementProgress(uint8_t attempt, uint8_t total)
{
  char line[32];
  snprintf(line, sizeof(line), "meting %u / %u", attempt, total);
#ifdef HAS_E_PAPER_DISPLAY
  showStatusLine(line);
#endif
  Serial.println(line);
} // showMeasurementProgress()

// Render final results on the e-paper.
static void showResults(float pm1, float pm25, float pm10, float vbat, uint8_t pct, AirStatus st, uint8_t validCount, uint8_t attempts)
{
#ifdef HAS_E_PAPER_DISPLAY
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

  }
  while (display.nextPage());
#endif
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

// Render an error page (e.g. SPS30 not found or no valid samples).
static void showError(const char *line1, const char *line2)
{
#ifdef HAS_E_PAPER_DISPLAY
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
  }
  while (display.nextPage());
#endif
} // showError()

// ===================== SPS30 helpers =====================

// Initialize I2C, probe the SPS30, and start measurement.
static bool sps30InitAndStart()
{
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

// Stop the measurement.
// In your design you cut power after finishing, so this is mostly for cleanliness.
static void sps30Stop()
{
  (void)sps30_stop_measurement();
} // sps30Stop()

// Read one measurement from the SPS30 (blocking up to timeoutMs).
// Returns true only if the sensor reports data-ready and the read succeeds.
static bool sps30ReadOnce(float &pm1, float &pm25, float &pm10, uint32_t timeoutMs)
{
  uint32_t start = millis();

  while ((millis() - start) < timeoutMs)
  {
    uint16_t ready = 0;

    if (sps30_read_data_ready(&ready) != 0)
    {
      delay(50);
      continue;
    }

    if (ready == 0)
    {
      delay(100);
      continue;
    }

    struct sps30_measurement m;
    if (sps30_read_measurement(&m) != 0)
    {
      return false;
    }

    pm1 = m.mc_1p0;
    pm25 = m.mc_2p5;
    pm10 = m.mc_10p0;

    return true;
  }

  return false;
} // sps30ReadOnce()

// Basic sanity checks for measurement values.
// This filters out NaNs and obviously bogus values.
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

  // Practical upper bounds for typical environments; adjust if needed.
  // This is mainly to reject bad reads (bus glitch, partial frame, etc).
  if (pm1 > 2000.0f || pm25 > 2000.0f || pm10 > 2000.0f)
  {
    return false;
  }

  return true;
} // isValidSample()


void switchOff()
{
  Serial.flush();

  digitalWrite(PIN_LATCH, LOW);

  while(true) 
  {
    // Wait for power to drop
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
} // switchOff()

// ===================== Main flow =====================

void setup()
{
  // Latch the power as early as possible to avoid dropping out during boot.
  pinMode(PIN_LATCH, OUTPUT);
  digitalWrite(PIN_LATCH, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_SWITCH, INPUT_PULLUP);

  for(int i=0; i<5; i++) 
  {
    digitalWrite(LED_BUILTIN, HIGH);  
    delay(100);                       
    digitalWrite(LED_BUILTIN, LOW);   
    delay(100);                       
  }

  Serial.begin(115200);
  delay(20);
  Serial.flush();
  Serial.println("\n\nAnd then it starts ...\n");
  Serial.print("Firmware version: ");
  Serial.println(PROG_VERSION);
  Serial.flush();
  // Initialize ADC reference and allow it to settle.
  Serial.println("Reading battery voltage...");
  analogReference(DEFAULT);
  delay(10);

  // Reset and initialize the e-paper display.
#ifdef HAS_E_PAPER_DISPLAY
  Serial.println("Initializing e-paper...");
  epdResetAndInit();
#endif

  // Tell the user the power is latched.
  showMessageFull("power latched", nullptr);
  Serial.println("Power latched.");

  // Start SPS30 measurement.
  if (!sps30InitAndStart())
  {
    showError("SPS30 not found", "check I2C wiring");
    Serial.println("Error: SPS30 not found or failed to start.");
    delay(800);

    //-x- switchOff();
    //-x- return;
  }

  // Warm-up (15 seconds) before taking samples.
  showWarmupCountdown(warmupSeconds);

  // Take up to maxMetingen measurement attempts and only average valid samples.
  float sumPM1 = 0.0f;
  float sumPM25 = 0.0f;
  float sumPM10 = 0.0f;

  uint8_t validCount = 0;

  for (uint8_t i = 1; i <= maxMetingen; i++)
  {
    showMeasurementProgress(i, maxMetingen);

    float pm1 = 0.0f;
    float pm25 = 0.0f;
    float pm10 = 0.0f;

    bool ok = sps30ReadOnce(pm1, pm25, pm10, 7000);

    if (ok && isValidSample(pm1, pm25, pm10))
    {
      sumPM1 += pm1;
      sumPM25 += pm25;
      sumPM10 += pm10;
      validCount++;
    }

    // SPS30 typically updates about once per second.
    delay(1000);
  }

  // If we got no valid samples, show an error and power down.
  if (validCount == 0)
  {
    showError("no valid samples", "check sensor");
    Serial.println("Error: no valid SPS30 samples received.");
    delay(800);

    sps30Stop();
    Serial.println("Disconnect power..");
    switchOff();
    return;
  }

  // Compute averages from valid samples only.
  float avgPM1 = sumPM1 / (float)validCount;
  float avgPM25 = sumPM25 / (float)validCount;
  float avgPM10 = sumPM10 / (float)validCount;

  // Read battery state for display.
  float vbat = readBatteryVoltage();
  uint8_t pct = batteryPercentFromVoltage(vbat);

  // Classify PM2.5 for a simple status label.
  AirStatus st = classifyPM25(avgPM25);

  // Show final results on the e-paper.
  showResults(avgPM1, avgPM25, avgPM10, vbat, pct, st, validCount, maxMetingen);
  delay(800);

  // Stop the sensor (optional but clean).
  sps30Stop();

  // Cut power by releasing the latch.
  switchOff();

} // setup()

void loop()
{
  // Not used (device powers down at the end of setup).
} // loop()