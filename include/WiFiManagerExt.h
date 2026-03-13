/*** Last Changed: 2026-03-13 - 12:21 ***/
#pragma once

#include <Arduino.h>
#include <ArduinoOTA.h>

class WiFiManagerExt
{
public:
  typedef void (*PortalStartCallback)(const String& apName);
  typedef void (*ConnectedCallback)(const String& hostName, const String& ipAddress);
  typedef void (*SimpleCallback)();

  WiFiManagerExt();

  void setPortalStartCallback(PortalStartCallback callback);
  void setConnectedCallback(ConnectedCallback callback);
  void setOtaStartCallback(SimpleCallback callback);
  void setOtaEndCallback(SimpleCallback callback);

  void requestStart();
  void handle();

  bool isServiceModeActive() const;
  bool isOtaInProgress() const;
  String getApNamePreview() const;

private:
  static WiFiManagerExt* instance;

  PortalStartCallback portalStartCallback;
  ConnectedCallback connectedCallback;
  SimpleCallback otaStartCallback;
  SimpleCallback otaEndCallback;

  bool startRequested;
  bool serviceModeActive;
  bool otaInProgress;
  bool otaReady;
  bool wifiSettingsErased;

  String apName;

  String makeApName() const;
  void startServiceMode();
  void setupOta();

  static void onPortalStartThunk(class WiFiManager* wifiManager);
  static void onOtaStartThunk();
  static void onOtaEndThunk();
  static void onOtaErrorThunk(ota_error_t error);
};
