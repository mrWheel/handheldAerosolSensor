/*** Last Changed: 2026-03-13 - 12:21 ***/
#include "WiFiManagerExt.h"

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>

WiFiManagerExt* WiFiManagerExt::instance = nullptr;

WiFiManagerExt::WiFiManagerExt()
{
  instance = this;
  portalStartCallback = nullptr;
  connectedCallback = nullptr;
  otaStartCallback = nullptr;
  otaEndCallback = nullptr;
  startRequested = false;
  serviceModeActive = false;
  otaInProgress = false;
  otaReady = false;
  wifiSettingsErased = false;
}

void WiFiManagerExt::setPortalStartCallback(PortalStartCallback callback)
{
  portalStartCallback = callback;
}

void WiFiManagerExt::setOtaStartCallback(SimpleCallback callback)
{
  otaStartCallback = callback;
}

void WiFiManagerExt::setConnectedCallback(ConnectedCallback callback)
{
  connectedCallback = callback;
}

void WiFiManagerExt::setOtaEndCallback(SimpleCallback callback)
{
  otaEndCallback = callback;
}

void WiFiManagerExt::requestStart()
{
  startRequested = true;
}

void WiFiManagerExt::handle()
{
  if (startRequested)
  {
    startRequested = false;
    startServiceMode();
  }

  if (otaReady)
  {
    ArduinoOTA.handle();
  }
}

bool WiFiManagerExt::isServiceModeActive() const
{
  return serviceModeActive;
}

bool WiFiManagerExt::isOtaInProgress() const
{
  return otaInProgress;
}

String WiFiManagerExt::getApNamePreview() const
{
  return makeApName();
}

String WiFiManagerExt::makeApName() const
{
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);

  char buf[24];
  snprintf(buf, sizeof(buf), "HHPMS-%02X%02X%02X", (unsigned)mac[3], (unsigned)mac[4], (unsigned)mac[5]);
  return String(buf);
}

void WiFiManagerExt::startServiceMode()
{
  if (serviceModeActive)
  {
    return;
  }

  WiFi.mode(WIFI_STA);

  WiFiManager wifiManager;
  apName = makeApName();

#ifdef WIFI_ERASE
  if (!wifiSettingsErased)
  {
    Serial.printf("WiFiManager: erasing saved WiFi settings (WIFI_ERASE)\n");
    wifiManager.resetSettings();
    WiFi.disconnect(true, true);
    wifiSettingsErased = true;
  }
#endif

  wifiManager.setAPCallback(onPortalStartThunk);
  wifiManager.setConfigPortalBlocking(true);

  bool connected = wifiManager.autoConnect(apName.c_str());
  if (!connected)
  {
    Serial.printf("WiFiManager: connect/portal ended without WiFi connection\n");
    return;
  }

  setupOta();
  serviceModeActive = true;

  if (connectedCallback != nullptr)
  {
    connectedCallback(apName, WiFi.localIP().toString());
  }

  Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());
}

void WiFiManagerExt::setupOta()
{
  ArduinoOTA.setHostname(apName.c_str());
  ArduinoOTA.onStart(onOtaStartThunk);
  ArduinoOTA.onEnd(onOtaEndThunk);
  ArduinoOTA.onError(onOtaErrorThunk);
  ArduinoOTA.begin();

  otaReady = true;

  Serial.printf("OTA ready (hostname: %s)\n", apName.c_str());
}

void WiFiManagerExt::onPortalStartThunk(WiFiManager* wifiManager)
{
  if (instance == nullptr)
  {
    return;
  }

  String ssid = instance->apName;
  if ((wifiManager != nullptr) && (wifiManager->getConfigPortalSSID() != nullptr))
  {
    ssid = String(wifiManager->getConfigPortalSSID());
  }

  if (instance->portalStartCallback != nullptr)
  {
    instance->portalStartCallback(ssid);
  }
}

void WiFiManagerExt::onOtaStartThunk()
{
  if (instance == nullptr)
  {
    return;
  }

  instance->otaInProgress = true;

  if (instance->otaStartCallback != nullptr)
  {
    instance->otaStartCallback();
  }
}

void WiFiManagerExt::onOtaEndThunk()
{
  if (instance == nullptr)
  {
    return;
  }

  instance->otaInProgress = false;

  if (instance->otaEndCallback != nullptr)
  {
    instance->otaEndCallback();
  }
}

void WiFiManagerExt::onOtaErrorThunk(ota_error_t error)
{
  (void)error;

  if (instance == nullptr)
  {
    return;
  }

  instance->otaInProgress = false;

  if (instance->otaEndCallback != nullptr)
  {
    instance->otaEndCallback();
  }
}
