#pragma once

// Analog sampling interval (ms)
// 1 ms interval = 1000 Hz sampling.
constexpr unsigned long ANALOG_POLL_INTERVAL_MS = 1UL;
// Dedicated SPI chip-select for external ADC.
// Pin 10 is typically used by W5500 Ethernet, so avoid sharing it.
constexpr int ADC_CS_PIN = 9;
// ADC register address for channel/sample register to poll.
constexpr uint8_t ADC1_ADDR = 0x01;
// Analog pin assignments
constexpr int ANALOG_PIN_13 = A13;
constexpr int ANALOG_PIN_14 = A14;

