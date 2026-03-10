#pragma once

// Analog sampling interval (ms)
// Reduced to 500ms to keep web responses closer to Serial output
constexpr unsigned long ANALOG_POLL_INTERVAL_MS = 10UL;
// Dedicated SPI chip-select for external ADC.
// Pin 10 is typically used by W5500 Ethernet, so avoid sharing it.
constexpr int ADC_CS_PIN = 9;
// ADC register address for channel/sample register to poll.
constexpr uint8_t ADC1_ADDR = 0x01;
// Analog pin assignments (Teensy A14/A15/A16)
constexpr int ANALOG_PIN_13 = A13;

