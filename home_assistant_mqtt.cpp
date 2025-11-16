#include "home_assistant_mqtt.h"

#ifndef ARDUINO_ARCH_ESP32
#error "This project now targets ESP32 boards only."
#endif

#include <WiFi.h>
#include <PubSubClient.h>

#include "wifi_secrets.h"

#ifndef WIFI_SSID
#error "WIFI_SSID is not defined. Copy wifi_secrets_template.h to wifi_secrets.h and fill in your credentials."
#endif

#ifndef WIFI_PASSWORD
#error "WIFI_PASSWORD is not defined. Copy wifi_secrets_template.h to wifi_secrets.h and fill in your credentials."
#endif

#ifndef MQTT_SERVER
#error "MQTT_SERVER is not defined. Copy wifi_secrets_template.h to wifi_secrets.h and fill in your credentials."
#endif

#ifndef MQTT_PORT
#error "MQTT_PORT is not defined. Copy wifi_secrets_template.h to wifi_secrets.h and fill in your credentials."
#endif

namespace {

// ---------- CONFIGURATION (edit these) ----------
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

const char* mqtt_server = MQTT_SERVER;   // Home Assistant / MQTT broker IP
const int   mqtt_port = MQTT_PORT;
#ifdef MQTT_USERNAME
const char* mqtt_user = MQTT_USERNAME;   // Leave "" if not used
#else
const char* mqtt_user = "";
#endif
#ifdef MQTT_PASSWORD
const char* mqtt_pass = MQTT_PASSWORD;
#else
const char* mqtt_pass = "";
#endif

const char* device_name = "Chicken Coop Door";
const char* unique_id = "chicken_coop_door_esp8266";
const char* mqtt_base = "homeassistant/cover/chicken_coop_door";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMqttState = 0;

constexpr unsigned long WIFI_RETRY_INTERVAL_MS = 5000;
constexpr unsigned long WIFI_STATUS_PRINT_INTERVAL_MS = 10000;
unsigned long lastWifiAttempt = 0;
unsigned long lastWifiStatusPrint = 0;
bool wifiAttemptInProgress = false;

constexpr unsigned long MQTT_RETRY_INTERVAL_MS = 5000;
unsigned long lastMqttAttempt = 0;

const char* (*stateLabelProvider)() = nullptr;
void (*commandOpenFn)() = nullptr;
void (*commandCloseFn)() = nullptr;
void (*commandStopFn)() = nullptr;

void publishDiscovery();
void publishStateImpl();

String topicWithSuffix(const char* suffix) {
  return String(mqtt_base) + suffix;
}

void ensureWifi() {
  unsigned long now = millis();
  wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED) {
    if (wifiAttemptInProgress) {
      Serial.printf("WiFi: connected (RSSI %d)\n", WiFi.RSSI());
    } else if (now - lastWifiStatusPrint >= WIFI_STATUS_PRINT_INTERVAL_MS) {
      lastWifiStatusPrint = now;
      Serial.printf("WiFi: ok (RSSI %d)\n", WiFi.RSSI());
    }
    wifiAttemptInProgress = false;
    return;
  }

  if (!wifiAttemptInProgress || now - lastWifiAttempt >= WIFI_RETRY_INTERVAL_MS) {
    wifiAttemptInProgress = true;
    lastWifiAttempt = now;
    Serial.println("WiFi: starting connection attempt");
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
  }

  if (now - lastWifiStatusPrint >= WIFI_STATUS_PRINT_INTERVAL_MS) {
    lastWifiStatusPrint = now;
    Serial.printf("WiFi: still reconnecting (status %d)\n", status);
  }
}

void publishDiscovery() {
  String discoveryTopic = topicWithSuffix("/config");
  String payload = String("{") +
    "\"name\":\"" + String(device_name) + "\"," +
    "\"unique_id\":\"" + String(unique_id) + "\"," +
    "\"device_class\":\"garage\"," +
    "\"cmd_t\":\"" + topicWithSuffix("/set") + "\"," +
    "\"stat_t\":\"" + topicWithSuffix("/state") + "\"," +
    "\"opt\":\"false\"," +
    "\"retain\":\"true\"," +
    "\"payload_open\":\"OPEN\"," +
    "\"payload_close\":\"CLOSE\"," +
    "\"payload_stop\":\"STOP\"," +
    "\"state_open\":\"open\"," +
    "\"state_closed\":\"closed\"," +
    "\"avail_t\":\"" + topicWithSuffix("/availability") + "\"," +
    "\"pl_avail\":\"online\"," +
    "\"pl_not_avail\":\"offline\"" +
  "}";

  client.publish(discoveryTopic.c_str(), payload.c_str(), true);
  client.publish(topicWithSuffix("/availability").c_str(), "online", true);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += static_cast<char>(payload[i]);
  msg.trim();
  msg.toUpperCase();

  Serial.print("MQTT command received: ");
  Serial.println(msg);

  if (msg == "OPEN") {
    if (commandOpenFn) commandOpenFn();
  } else if (msg == "CLOSE") {
    if (commandCloseFn) commandCloseFn();
  } else if (msg == "STOP") {
    if (commandStopFn) commandStopFn();
  } else {
    Serial.println("MQTT command ignored (unknown payload)");
  }
}

void publishStateImpl() {
  if (!client.connected() || stateLabelProvider == nullptr) return;
  String stateTopic = topicWithSuffix("/state");
  client.publish(stateTopic.c_str(), stateLabelProvider(), true);
}

}  // namespace

void homeAssistantMqttSetup(const char* (*stateLabelFn)(),
                            void (*openFn)(),
                            void (*closeFn)(),
                            void (*stopFn)()) {
  stateLabelProvider = stateLabelFn;
  commandOpenFn = openFn;
  commandCloseFn = closeFn;
  commandStopFn = stopFn;

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  wifiAttemptInProgress = false;
  lastWifiAttempt = 0;
  lastWifiStatusPrint = 0;
  lastMqttAttempt = 0;
  ensureWifi();
}

void homeAssistantMqttLoop() {
  ensureWifi();

  if (!client.connected()) {
    if (WiFi.status() == WL_CONNECTED) {
      unsigned long now = millis();
      if (now - lastMqttAttempt >= MQTT_RETRY_INTERVAL_MS) {
        lastMqttAttempt = now;
        Serial.print("MQTT: connecting...");
        if (client.connect("chicken_coop_door_client", mqtt_user, mqtt_pass,
                           topicWithSuffix("/availability").c_str(), 0, true, "offline")) {
          Serial.println(" connected");
          client.setCallback(mqttCallback);
          client.subscribe(topicWithSuffix("/set").c_str());
          publishDiscovery();
          publishStateImpl();
        } else {
          Serial.print(" failed, rc=");
          Serial.println(client.state());
        }
      }
    }
    return;
  }

  client.loop();
  if (millis() - lastMqttState > 30000) {
    lastMqttState = millis();
    publishStateImpl();
  }
}

void homeAssistantPublishState() {
  publishStateImpl();
}

bool homeAssistantWifiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

bool homeAssistantMqttConnected() {
  return client.connected();
}
