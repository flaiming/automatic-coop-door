#pragma once

#include <Arduino.h>

// Initializes WiFi and MQTT connectivity for Home Assistant.
// Provide helpers for reporting door state and controlling the motor.
void homeAssistantMqttSetup(const char* (*stateLabelFn)(),
                            void (*openFn)(),
                            void (*closeFn)(),
                            void (*stopFn)());

// Keeps MQTT connection alive and periodically republishes state.
void homeAssistantMqttLoop();

// Immediately publishes the current door state to MQTT, if connected.
void homeAssistantPublishState();

// Returns true when the ESP32 is associated with the configured Wi-Fi network.
bool homeAssistantWifiConnected();

// Returns true when the MQTT client is connected to the broker.
bool homeAssistantMqttConnected();
