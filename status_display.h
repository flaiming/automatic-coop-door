#pragma once

#ifndef STATUS_DISPLAY_ENABLED
#define STATUS_DISPLAY_ENABLED 1
#endif

#include <Arduino.h>

void statusDisplaySetup();
void statusDisplaySetDoorState(const char* stateLabel);
void statusDisplaySetLightLevel(int averagedValue);
void statusDisplayLoop();
