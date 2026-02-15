#pragma once
#include "hardware_config.h"

void hw_init();
void hw_loop();
void hw_setBlinkEnabled(bool enabled);
void hw_setLed(bool on);
void hw_setRelay(int relay, bool on);
bool hw_getRelay(int relay);
