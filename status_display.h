#pragma once

#include <Arduino.h>

void statusDisplaySetup();
void statusDisplaySetDoorState(const char* stateLabel);
void statusDisplaySetLightLevel(int averagedValue);
void statusDisplayLoop();
