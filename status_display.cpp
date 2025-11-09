#include "status_display.h"

#ifndef ARDUINO_ARCH_ESP32
#error "This project now targets ESP32 boards only."
#endif

#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

namespace {
constexpr uint8_t SCREEN_WIDTH = 128;
constexpr uint8_t SCREEN_HEIGHT = 64;
constexpr int8_t OLED_RESET_PIN = -1;
constexpr uint8_t OLED_I2C_ADDRESS = 0x3C;
constexpr unsigned long REFRESH_INTERVAL_MS = 1000;
constexpr uint8_t LARGE_TEXT_SIZE = 2;
constexpr uint8_t SMALL_TEXT_SIZE = 1;
constexpr uint8_t LIGHT_LINE_Y = 24; // leave a gap under the first line
constexpr uint8_t WIFI_LINE_Y = 52;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);

bool displayReady = false;
bool displayDirty = true;
unsigned long lastRefresh = 0;

String wifiLine = "WiFi: --";
String doorLine = "Door: --";
String lightLine = "Light: --";

String wifiStatusLine() {
  wl_status_t status = WiFi.status();
  switch (status) {
    case WL_CONNECTED:
      return String("WiFi: ok ") + WiFi.RSSI() + "dBm";
    case WL_NO_SSID_AVAIL:
      return "WiFi: no SSID";
    case WL_CONNECT_FAILED:
      return "WiFi: failed";
    case WL_CONNECTION_LOST:
      return "WiFi: lost";
    case WL_DISCONNECTED:
      return "WiFi: disconnected";
    case WL_IDLE_STATUS:
      return "WiFi: idle";
    default:
      return "WiFi: unknown";
  }
}

void drawDisplay() {
  if (!displayReady) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(LARGE_TEXT_SIZE);
  display.setCursor(0, 0);
  display.println(doorLine);

  display.setCursor(0, LIGHT_LINE_Y);
  display.println(lightLine);

  display.setTextSize(SMALL_TEXT_SIZE);
  display.setCursor(0, WIFI_LINE_Y);
  display.println(wifiLine);
  display.display();
  displayDirty = false;
  lastRefresh = millis();
}

}  // namespace

void statusDisplaySetup() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println("OLED: init failed");
    return;
  }
  displayReady = true;
  display.setRotation(0);
  display.setTextWrap(false);
  display.clearDisplay();
  drawDisplay();
}

void statusDisplaySetDoorState(const char* stateLabel) {
  doorLine = String("") + (stateLabel ? stateLabel : "--");
  displayDirty = true;
}

void statusDisplaySetLightLevel(int averagedValue) {
  lightLine = String("Light: ") + averagedValue;
  displayDirty = true;
}

void statusDisplayLoop() {
  if (!displayReady) return;

  String newWifi = wifiStatusLine();
  if (newWifi != wifiLine) {
    wifiLine = newWifi;
    displayDirty = true;
  }

  if (displayDirty || millis() - lastRefresh >= REFRESH_INTERVAL_MS) {
    drawDisplay();
  }
}
