#include <Arduino.h>
#include "hardware.h"
#include "networking.h"

// Hardware globals & logic implemented below in `app_main.cpp` via the
// `hardware.h` interface; networking lives in `networking.cpp`.

// -----------------------------------------------------------------------------
// Hardware implementation (exposed via hardware.h)
// -----------------------------------------------------------------------------

// Blink state
static unsigned long lastToggle    = 0;
static unsigned long blinkInterval = DEFAULT_BLINK_INTERVAL;
static bool blinkEnabled = true;
static bool ledState     = false;

// Relay state tracking
static bool relayState[5] = {false, false, false, false, false};  // index 0 unused, 1-4 for relays

void hw_init() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);

  delay(500);

  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 8000)) {
    // wait up to 8 s for USB Serial
  }

  Serial.println();
  Serial.println("APP firmware running (hardware init + Ethernet handled separately).");
  Serial.println("Built as env: teensy41_app");
  Serial.println("fw_teensy41");   // marker so updater accepts HEX
  Serial.flush();

  // initialize networking after hardware is ready
  networking_init();
}

void hw_loop() {
  unsigned long now = millis();
  if (blinkEnabled && (now - lastToggle >= blinkInterval)) {
    lastToggle = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }

  // networking tasks
  networking_loop();
}

void hw_setBlinkEnabled(bool enabled) {
  blinkEnabled = enabled;
}

void hw_setLed(bool on) {
  blinkEnabled = false;
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

void hw_setRelay(int relay, bool on) {
  int pin = -1;
  switch (relay) {
    case 1: pin = RELAY1_PIN; break;
    case 2: pin = RELAY2_PIN; break;
    case 3: pin = RELAY3_PIN; break;
    case 4: pin = RELAY4_PIN; break;
    default: return;
  }
  relayState[relay] = on;
  digitalWrite(pin, on ? HIGH : LOW);
}

bool hw_getRelay(int relay) {
  if (relay < 1 || relay > 4) return false;
  return relayState[relay];
}

// Entry points for Arduino runtime
void setup() {
  hw_init();
}

void loop() {
  hw_loop();
}
