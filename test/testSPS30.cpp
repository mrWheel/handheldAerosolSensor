/*** SPS30 wiring + functional check (I2C) ***/

#include <Arduino.h>
#include <Wire.h>

#include <sensirion_i2c.h>
#include <sps30.h>

// ===================== Pins (from platformio.ini build_flags) =====================

// — SPS I2C pins (also used for BMP280 in your main project)
static const uint8_t pinSpsSda = GPIO_PIN_SPS_SDA;
static const uint8_t pinSpsScl = GPIO_PIN_SPS_SCL;

// ===================== I2C bus instance =====================

// — Use explicit TwoWire instance so we control pins reliably on ESP32
static TwoWire i2cBus(0);

// ===================== Configuration =====================

static const uint32_t i2cClockHz = 100000UL;
static const uint32_t dataReadyTimeoutMs = 15000UL;

// ===================== Forward declarations =====================

static void i2cInit();
static void i2cScan();
static bool sps30CheckOnce();
static bool isValidPm(float pm1, float pm25, float pm10);
static void runChecks();

// ===================== I2C helpers =====================

static void i2cInit()
{
  // — Bind Sensirion driver to our bus object
  // — Note: sensirion_i2c_init() calls begin() without pins, so we set pins AFTER.
  sensirion_i2c_init(i2cBus);

  // — Now configure the ESP32 I2C pins explicitly
  i2cBus.begin(pinSpsSda, pinSpsScl);
  i2cBus.setClock(i2cClockHz);

  Serial.printf("Info: I2C init OK (SDA=%u SCL=%u, %lu Hz)\n",
                (unsigned)pinSpsSda,
                (unsigned)pinSpsScl,
                (unsigned long)i2cClockHz);

} //   i2cInit()

static void i2cScan()
{
  Serial.printf("Info: I2C scan start\n");

  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++)
  {
    i2cBus.beginTransmission(addr);
    uint8_t err = (uint8_t)i2cBus.endTransmission(true);

    if (err == 0)
    {
      Serial.printf("Info: I2C device found at 0x%02X\n", (unsigned)addr);
      found++;
    }
  }

  if (found == 0)
  {
    Serial.printf("Warning: I2C scan found no devices\n");
  }

  Serial.printf("Info: I2C scan done (%u devices)\n", (unsigned)found);

} //   i2cScan()

// ===================== SPS30 check =====================

static bool isValidPm(float pm1, float pm25, float pm10)
{
  // — Reject NaN/Inf
  if (!isfinite(pm1) || !isfinite(pm25) || !isfinite(pm10))
  {
    return false;
  }

  // — Reject negative
  if (pm1 < 0.0f || pm25 < 0.0f || pm10 < 0.0f)
  {
    return false;
  }

  // — Reject absurd values (sanity)
  if (pm1 > 5000.0f || pm25 > 5000.0f || pm10 > 5000.0f)
  {
    return false;
  }

  return true;

} //   isValidPm()

static bool sps30CheckOnce()
{
  // — Probe sensor on I2C
  if (sps30_probe() != 0)
  {
    Serial.printf("Error: SPS30 probe failed (no response)\n");
    return false;
  }

  Serial.printf("Info: SPS30 probe OK\n");

  // — Start measurements
  if (sps30_start_measurement() != 0)
  {
    Serial.printf("Error: SPS30 start_measurement failed\n");
    return false;
  }

  Serial.printf("Info: SPS30 measurement started\n");

  // — Wait for data-ready with timeout
  uint32_t t0 = millis();
  uint16_t ready = 0;

  while (true)
  {
    if ((millis() - t0) > dataReadyTimeoutMs)
    {
      Serial.printf("Error: SPS30 data_ready timeout (%lu ms)\n", (unsigned long)dataReadyTimeoutMs);
      (void)sps30_stop_measurement();
      return false;
    }

    if (sps30_read_data_ready(&ready) != 0)
    {
      Serial.printf("Warning: SPS30 read_data_ready failed, retrying\n");
      delay(200);
      continue;
    }

    if (ready != 0)
    {
      break;
    }

    delay(200);
  }

  // — Read one measurement
  struct sps30_measurement m;

  if (sps30_read_measurement(&m) != 0)
  {
    Serial.printf("Error: SPS30 read_measurement failed\n");
    (void)sps30_stop_measurement();
    return false;
  }

  float pm1 = m.mc_1p0;
  float pm25 = m.mc_2p5;
  float pm10 = m.mc_10p0;

  if (!isValidPm(pm1, pm25, pm10))
  {
    Serial.printf("Error: SPS30 returned invalid values: PM1=%.3f PM2.5=%.3f PM10=%.3f\n",
                  pm1, pm25, pm10);
    (void)sps30_stop_measurement();
    return false;
  }

  Serial.printf("Info: SPS30 OK (PM1=%.1f, PM2.5=%.1f, PM10=%.1f) ug/m3\n",
                pm1, pm25, pm10);

  // — Stop measurement (this is a check tool, not a sampler)
  if (sps30_stop_measurement() != 0)
  {
    Serial.printf("Warning: SPS30 stop_measurement failed\n");
  }
  else
  {
    Serial.printf("Info: SPS30 measurement stopped\n");
  }

  return true;

} //   sps30CheckOnce()

// ===================== Orchestration =====================

static void runChecks()
{
  i2cInit();
  i2cScan();

  bool ok = sps30CheckOnce();

  if (ok)
  {
    Serial.printf("Info: SPS30 CHECK PASSED\n");
  }
  else
  {
    Serial.printf("Error: SPS30 CHECK FAILED\n");
  }

} //   runChecks()

// ===================== Arduino entrypoints =====================

void setup()
{
  Serial.begin(115200);
  delay(200);

  Serial.printf("\nSPS30 I2C Check Tool\n");
  runChecks();

} //   setup()

void loop()
{
  // — Keep loop empty except function calls (per your rule)
  delay(1000);

} //   loop()