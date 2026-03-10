#include <Arduino.h>
#include "hardware.h"
#include "networking.h"

// Hardware globals & logic implemented below in `app_main.cpp` via the
// `hardware.h` interface; networking lives in `networking.cpp`.

// -----------------------------------------------------------------------------
// Hardware implementation (exposed via hardware.h)
// -----------------------------------------------------------------------------

void hw_init() {
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
  // networking tasks
  networking_loop();
}

// Entry points for Arduino runtime
void setup() {
  hw_init();
}

void loop() {
  hw_loop();
}
