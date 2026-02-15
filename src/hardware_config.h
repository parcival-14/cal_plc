#pragma once

// Pin assignments
constexpr int LED_PIN     = 13;
constexpr int RELAY1_PIN  = 2;
constexpr int RELAY2_PIN  = 3;
constexpr int RELAY3_PIN  = 4;
constexpr int RELAY4_PIN  = 5;

// Default blink interval (ms)
constexpr unsigned long DEFAULT_BLINK_INTERVAL = 50UL;
// Analog sampling interval (ms)
// Reduced to 500ms to keep web responses closer to Serial output
constexpr unsigned long ANALOG_POLL_INTERVAL_MS = 500UL;
// Analog pin assignments (Teensy A14/A15/A16)
constexpr int ANALOG_PIN_14 = A14;
constexpr int ANALOG_PIN_15 = A15;
constexpr int ANALOG_PIN_16 = A16;
