#include <Arduino.h>
#include <SPI.h>
#include <algorithm>
#include <cstring>

// Networking + web server
#include <QNEthernet.h>
#include <AsyncWebServer_Teensy41.h>

// FlasherX / OTA helper
#include <FXUtil.h>
extern "C" {
  #include <FlashTxx.h>
}
#include <teensyupdater.hpp>
#include <SD.h>

#include "hardware.h"
#include "networking.h"
#include "constants.h"

using namespace qindesign::network;

// -----------------------------------------------------------------------------
// Globals (networking only)
// -----------------------------------------------------------------------------

static AsyncWebServer server(constants::HTTP_PORT);
static TeensyOtaUpdater ota(&server, constants::OTA_PATH);
static volatile bool pendingUpdate = false;
static bool routesRegistered = false;
static bool serverStarted = false;
static bool defaultHeadersConfigured = false;

// MAX22530/31-style SPI ADC access
static SPISettings max22530SPI(1000000, MSBFIRST, SPI_MODE0);
static bool adcReady = false;
static constexpr float ADC_REF_VOLTS = 1.8f;
static constexpr float ADC_MAX_CODE_12BIT = 4095.0f;
static constexpr float SHUNT_RESISTOR_OHMS = 47.0f;
static constexpr float TEENSY_ADC_REF_VOLTS = 3.3f;
static constexpr float A14_SHUNT_RESISTOR_OHMS = (560.0f / 3.0f);
static constexpr float LOOP_CURRENT_MIN_MA = 4.0f;
static constexpr float LOOP_CURRENT_MAX_MA = 20.0f;
static constexpr float LOOP_TEMP_MIN_C = 250.0f;
static constexpr float LOOP_TEMP_MAX_C = 2000.0f;
static constexpr uint16_t MAX22530_EXPECTED_PROD_ID = 0x0081u;

// MAX2253x register map and control bits used by this app.
static constexpr uint8_t REG_PROD_ID = 0x00;
static constexpr uint8_t REG_ADC1 = 0x01;
static constexpr uint8_t REG_INTERRUPT_STATUS = 0x12;
static constexpr uint8_t REG_INTERRUPT_ENABLE = 0x13;
static constexpr uint8_t REG_CONTROL = 0x14;
static constexpr uint16_t INT_STATUS_POR = (1u << 7);
static constexpr uint16_t INT_STATUS_SPICRC = (1u << 8);
static constexpr uint16_t INT_STATUS_SPIFRM = (1u << 9);
static constexpr uint16_t INT_STATUS_FLD = (1u << 10);
static constexpr uint16_t INT_STATUS_ADCF = (1u << 11);
static constexpr uint16_t INT_STATUS_EOC = (1u << 12);
static constexpr uint16_t CONTROL_REST = (1u << 0);
static constexpr uint16_t CONTROL_SRES = (1u << 1);
static constexpr uint16_t CONTROL_CLR_POR = (1u << 2);
static constexpr uint16_t CONTROL_DISPWR = (1u << 3);
static constexpr uint16_t CONTROL_CLEAR_ALL_FILTERS = 0x00F0u;
static constexpr uint16_t IE_EEOC = (1u << 12);

// Cached analog values (kept in sync with Serial prints)
static volatile int cachedShuntA0Counts = 0;
static volatile int cachedA14Counts = 0;
static constexpr int ANALOG_MEDIAN_WINDOW = 1000;
static int a13Samples[ANALOG_MEDIAN_WINDOW] = {0};
static int a13MedianScratch[ANALOG_MEDIAN_WINDOW] = {0};
static int a13SampleIndex = 0;
static int a13SampleCount = 0;
static volatile unsigned long lastAnalogPrintMillis = 0;
static constexpr unsigned long ANALOG_PRINT_INTERVAL_MS = 500UL;
static volatile bool cachedAdcFresh = false;
static volatile uint16_t cachedProdId = 0;
static volatile uint16_t cachedControl = 0;
static volatile uint16_t cachedInterruptStatus = 0;
static bool adcFieldPowerDisabled = false;
static volatile unsigned long lastAdcPollMillis = 0;
static volatile unsigned long lastAdcRecoverMillis = 0;
static volatile unsigned long lastFailedRecoveryMillis = 0;
static volatile unsigned long lastNetworkRestartMillis = 0;
static volatile unsigned long recoveryTriggerCount = 0;
static volatile unsigned long networkingLoopCount = 0;
static constexpr unsigned long ADC_INIT_STABILIZE_MS = 15UL;
static constexpr unsigned long ADC_POLL_WATCHDOG_MS = 60000UL;
static constexpr unsigned long ADC_RECOVER_COOLDOWN_MS = 5000UL;
static constexpr unsigned long ADC_NOT_READY_RETRY_MS = 5000UL;
static constexpr uint16_t ADC_STUCK_THRESHOLD_COUNTS = 10;
static constexpr uint16_t ADC_SAME_VALUE_LIMIT = 10;
static constexpr uint16_t ADC_STUCK_RECOVERY_TRIGGER = 5;
static constexpr unsigned long NETWORK_RESTART_COOLDOWN_MS = 15000UL;
static constexpr unsigned long MAX22530_ENABLE_FIELD_POWER_WAIT_MS = 15UL;
static constexpr unsigned long MAX22530_CLEAR_FILTERS_WAIT_MS = 2UL;
static constexpr unsigned long MAX22530_CLEAR_POR_WAIT_MS = 2UL;
static constexpr unsigned long MAX22530_SOFT_RESET_WAIT_MS = 10UL;
static constexpr unsigned long MAX22530_HARD_RESET_WAIT_MS = 20UL;
enum class AdcWorkflow : uint8_t {
  Idle,
  Init,
  Recovery,
  ManualEnableFieldPower,
  ManualHardReset,
};
enum class SpiControlAction : uint8_t {
  None,
  Stop,
  Start,
  Reinit,
};
static uint16_t lastAdcReading = 0xFFFFu;
static uint16_t sameAdcReadingCount = 0;
static uint16_t stuckCount = 0;
static AdcWorkflow adcWorkflow = AdcWorkflow::Idle;
static uint8_t adcWorkflowStep = 0;
static unsigned long adcWorkflowReadyAtMillis = 0;
static const char *adcWorkflowReason = nullptr;
static bool spiBusEnabled = true;
static SpiControlAction pendingSpiControlAction = SpiControlAction::None;
static volatile uint32_t httpResponsesSent = 0;
static volatile uint32_t adcStatusRequests = 0;
static volatile uint32_t adcNotReadyResponses = 0;
static volatile uint32_t networkingRestarts = 0;
static volatile uint32_t httpThrottledRequests = 0;
static volatile unsigned long lastHttpResponseMillis = 0;
static volatile unsigned long lastAdcStatusMillis = 0;
static volatile unsigned long lastHttpDiagPrintMillis = 0;
static constexpr unsigned long HTTP_DIAG_PRINT_INTERVAL_MS = 30000UL;
static constexpr unsigned long HTTP_RESPONSE_WATCHDOG_MS = 3000UL;
static constexpr unsigned long HTTP_RATE_WINDOW_MS = 1000UL;
static constexpr uint16_t HTTP_MAX_REQUESTS_PER_WINDOW = 40;
static volatile unsigned long httpRateWindowStartMillis = 0;
static volatile uint16_t httpRateWindowCount = 0;
static bool httpWatchdogArmed = false;

static void restartNetworkingService(const char *reason);

static void sendNoKeepAlive(AsyncWebServerRequest *request,
                            int code,
                            const char *contentType,
                            const char *content) {
  AsyncWebServerResponse *response = request->beginResponse(code, contentType, content);
  response->addHeader("Connection", "close");
  response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  httpResponsesSent++;
  lastHttpResponseMillis = millis();
  request->send(response);
}

static bool isHttpOverloaded() {
  const unsigned long now = millis();
  if (httpRateWindowStartMillis == 0 || (now - httpRateWindowStartMillis) >= HTTP_RATE_WINDOW_MS) {
    httpRateWindowStartMillis = now;
    httpRateWindowCount = 0;
  }

  if (httpRateWindowCount >= HTTP_MAX_REQUESTS_PER_WINDOW) {
    httpThrottledRequests++;
    return true;
  }

  httpRateWindowCount++;
  return false;
}

static bool rejectIfHttpOverloaded(AsyncWebServerRequest *request) {
  if (!isHttpOverloaded()) {
    return false;
  }

  sendNoKeepAlive(request, 503, "application/json", "{\"error\":\"busy\",\"retry_ms\":250}");
  return true;
}

static float adcCountsToVolts(int adcCounts) {
  if (adcCounts <= 0) {
    return 0.0f;
  }
  if (adcCounts >= static_cast<int>(ADC_MAX_CODE_12BIT)) {
    return ADC_REF_VOLTS;
  }
  return (static_cast<float>(adcCounts) * ADC_REF_VOLTS) / ADC_MAX_CODE_12BIT;
}

static float adcCountsToMilliamps(int adcCounts) {
  const float volts = adcCountsToVolts(adcCounts);
  return (volts / SHUNT_RESISTOR_OHMS) * 1000.0f;
}

static float teensyAdcCountsToVolts(int adcCounts) {
  if (adcCounts <= 0) {
    return 0.0f;
  }
  if (adcCounts >= static_cast<int>(ADC_MAX_CODE_12BIT)) {
    return TEENSY_ADC_REF_VOLTS;
  }
  return (static_cast<float>(adcCounts) * TEENSY_ADC_REF_VOLTS) / ADC_MAX_CODE_12BIT;
}

static float voltsToMilliamps(float volts, float shuntOhms) {
  if (shuntOhms <= 0.0f) {
    return 0.0f;
  }
  return (volts / shuntOhms) * 1000.0f;
}

static float loopCurrentToTempC(float currentMa) {
  const float clampedCurrent = std::max(LOOP_CURRENT_MIN_MA, std::min(currentMa, LOOP_CURRENT_MAX_MA));
  const float normalized = (clampedCurrent - LOOP_CURRENT_MIN_MA) / (LOOP_CURRENT_MAX_MA - LOOP_CURRENT_MIN_MA);
  return LOOP_TEMP_MIN_C + normalized * (LOOP_TEMP_MAX_C - LOOP_TEMP_MIN_C);
}

static int computeMedian(const int *samples, int sampleCount) {
  if (sampleCount <= 0) {
    return 0;
  }

  std::memcpy(a13MedianScratch, samples, static_cast<size_t>(sampleCount) * sizeof(int));
  int *begin = a13MedianScratch;
  int *mid = begin + (sampleCount / 2);
  int *end = begin + sampleCount;

  std::nth_element(begin, mid, end);
  const int upperMid = *mid;

  if ((sampleCount & 1) != 0) {
    return upperMid;
  }

  const int lowerMid = *std::max_element(begin, mid);
  return (lowerMid + upperMid) / 2;
}

// Timestamp (ms) of last periodic analog sample
static volatile unsigned long lastAnalogSampleMillis = 0;

// Header format: bits[7:2]=A[5:0], bit1=W/R, bit0=BURST.
// The MAX22530 on this board expects bit1 set for writes and clear for reads.
static uint8_t makeHeader(uint8_t addr, bool isWrite, bool burst) {
  return static_cast<uint8_t>(((addr & 0x3F) << 2) | (isWrite ? 0x02 : 0x00) | (burst ? 0x01 : 0x00));
}

static uint16_t transfer24(uint8_t header, uint16_t txPayload) {
  const uint8_t tx0 = header;
  const uint8_t tx1 = static_cast<uint8_t>((txPayload >> 8) & 0xFF);
  const uint8_t tx2 = static_cast<uint8_t>(txPayload & 0xFF);

  SPI.beginTransaction(max22530SPI);
  digitalWrite(ADC_CS_PIN, LOW);
  const uint8_t rx0 = SPI.transfer(tx0);
  const uint8_t rx1 = SPI.transfer(tx1);
  const uint8_t rx2 = SPI.transfer(tx2);
  (void)rx0;
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.endTransaction();

  return (static_cast<uint16_t>(rx1) << 8) | rx2;
}

static uint16_t readRegister(uint8_t regAddr) {
  return transfer24(makeHeader(regAddr, false, false), 0x0000);
}

static void writeRegister(uint8_t regAddr, uint16_t value) {
  (void)transfer24(makeHeader(regAddr, true, false), value);
}

static uint16_t adcCode12(uint16_t reg) {
  return static_cast<uint16_t>(reg & 0x0FFF);
}

static bool adcFresh(uint16_t reg) {
  // Datasheet: bit15 == 0 indicates updated data since last read.
  return (reg & 0x8000u) == 0;
}

static void clearAdcCachedData() {
  std::fill_n(a13Samples, ANALOG_MEDIAN_WINDOW, 0);
  std::fill_n(a13MedianScratch, ANALOG_MEDIAN_WINDOW, 0);
  a13SampleIndex = 0;
  a13SampleCount = 0;
  cachedShuntA0Counts = 0;
  cachedAdcFresh = false;
}

static void resetStuckTracking() {
  lastAdcReading = 0xFFFFu;
  sameAdcReadingCount = 0;
  stuckCount = 0;
}

static bool isStuck(uint16_t adcReading) {
  if (adcReading == lastAdcReading) {
    if (sameAdcReadingCount < 0xFFFFu) {
      sameAdcReadingCount++;
    }
  } else {
    lastAdcReading = adcReading;
    sameAdcReadingCount = 1;
  }

  return adcReading < ADC_STUCK_THRESHOLD_COUNTS || sameAdcReadingCount >= ADC_SAME_VALUE_LIMIT;
}

static void logInterruptStatus(uint16_t interruptStatus) {
  if (interruptStatus == 0) {
    Serial.println("  No interrupt or fault bits set.");
    return;
  }

  if ((interruptStatus & INT_STATUS_POR) != 0) {
    Serial.println("  POR: Power-on reset detected.");
  }
  if ((interruptStatus & INT_STATUS_SPICRC) != 0) {
    Serial.println("  SPICRC: Internal CRC failure threshold reached.");
  }
  if ((interruptStatus & INT_STATUS_SPIFRM) != 0) {
    Serial.println("  SPIFRM: SPI framing error detected.");
  }
  if ((interruptStatus & INT_STATUS_FLD) != 0) {
    Serial.println("  FLD: Field-side data stream lost.");
  }
  if ((interruptStatus & INT_STATUS_ADCF) != 0) {
    Serial.println("  ADCF: ADC functionality error flagged.");
  }
  if ((interruptStatus & INT_STATUS_EOC) != 0) {
    Serial.println("  EOC: End-of-conversion asserted.");
  }
}

static bool max22530Responsive(uint16_t prodId, uint16_t interruptStatus, uint16_t control) {
  if (prodId == MAX22530_EXPECTED_PROD_ID) {
    return true;
  }

  const bool allZero = (prodId == 0u) && (interruptStatus == 0u) && (control == 0u);
  const bool allOnes = (prodId == 0xFFFFu) && (interruptStatus == 0xFFFFu) && (control == 0xFFFFu);
  if (allZero || allOnes) {
    return false;
  }

  return interruptStatus != 0u || control != 0u;
}

static void setAdcWorkflow(AdcWorkflow workflow,
                           uint8_t step,
                           unsigned long delayMs = 0,
                           const char *reason = nullptr) {
  adcWorkflow = workflow;
  adcWorkflowStep = step;
  adcWorkflowReadyAtMillis = millis() + delayMs;
  adcWorkflowReason = reason;
}

static void clearAdcWorkflow() {
  adcWorkflow = AdcWorkflow::Idle;
  adcWorkflowStep = 0;
  adcWorkflowReadyAtMillis = 0;
  adcWorkflowReason = nullptr;
}

static void writeReconfigureRegisters() {
  writeRegister(REG_INTERRUPT_ENABLE, IE_EEOC);
  writeRegister(REG_CONTROL, CONTROL_CLR_POR);
}

static void writeEnableFieldPower() {
  const uint16_t control = readRegister(REG_CONTROL);
  writeRegister(REG_CONTROL, static_cast<uint16_t>(control & ~CONTROL_DISPWR));
}

static void writeClearAdcFilters() {
  writeRegister(REG_CONTROL, CONTROL_CLEAR_ALL_FILTERS);
}

static void writeClearPor() {
  const uint16_t control = readRegister(REG_CONTROL);
  writeRegister(REG_CONTROL, static_cast<uint16_t>(control | CONTROL_CLR_POR));
}

static void writeSoftReset() {
  writeRegister(REG_CONTROL, CONTROL_SRES);
}

static void writeHardReset() {
  writeRegister(REG_CONTROL, CONTROL_REST);
}

static bool readDiagnostics(bool forceAccess = false) {
  if (adcFieldPowerDisabled && !forceAccess) {
    return false;
  }

  const uint16_t prodId = readRegister(REG_PROD_ID);
  const uint16_t interruptStatus = readRegister(REG_INTERRUPT_STATUS);
  const uint16_t control = readRegister(REG_CONTROL);

  cachedProdId = prodId;
  cachedInterruptStatus = interruptStatus;
  cachedControl = control;

  Serial.printf("MAX22530 diagnostics: PROD_ID=0x%04X INT_STATUS=0x%04X CONTROL=0x%04X\n",
                prodId,
                interruptStatus,
                control);
  logInterruptStatus(interruptStatus);
  Serial.printf("  CONTROL DISPWR=%u CLR_POR=%u SRES=%u REST=%u\n",
                (control & CONTROL_DISPWR) != 0 ? 1u : 0u,
                (control & CONTROL_CLR_POR) != 0 ? 1u : 0u,
                (control & CONTROL_SRES) != 0 ? 1u : 0u,
                (control & CONTROL_REST) != 0 ? 1u : 0u);

  if (prodId != MAX22530_EXPECTED_PROD_ID) {
    Serial.printf("MAX22530 diagnostics warning: expected PROD_ID 0x%04X, got 0x%04X\n",
                  MAX22530_EXPECTED_PROD_ID,
                  prodId);
  }

  return max22530Responsive(prodId, interruptStatus, control);
}

static uint16_t readAdc1(bool *freshOut = nullptr, uint16_t *interruptStatusOut = nullptr, bool forceAccess = false) {
  if (adcFieldPowerDisabled && !forceAccess) {
    cachedAdcFresh = false;
    if (freshOut != nullptr) {
      *freshOut = false;
    }
    if (interruptStatusOut != nullptr) {
      *interruptStatusOut = cachedInterruptStatus;
    }
    return 0;
  }

  const uint16_t adcRegister = readRegister(REG_ADC1);
  const uint16_t prodId = readRegister(REG_PROD_ID);
  const uint16_t interruptStatus = readRegister(REG_INTERRUPT_STATUS);
  const uint16_t control = readRegister(REG_CONTROL);

  cachedAdcFresh = adcFresh(adcRegister);
  cachedProdId = prodId;
  cachedInterruptStatus = interruptStatus;
  cachedControl = control;
  lastAdcPollMillis = millis();

  if (freshOut != nullptr) {
    *freshOut = cachedAdcFresh;
  }
  if (interruptStatusOut != nullptr) {
    *interruptStatusOut = interruptStatus;
  }

  return adcCode12(adcRegister);
}

static bool checkRecoveryResult(const char *stepName) {
  const uint16_t adcReading = readAdc1(nullptr, nullptr, true);
  const bool stuck = isStuck(adcReading);
  cachedShuntA0Counts = adcReading;

  Serial.printf("MAX22530 recovery check after %s: raw=%u fresh=%u int_status=0x%04X stuck=%u\n",
                stepName,
                adcReading,
                cachedAdcFresh ? 1u : 0u,
                cachedInterruptStatus,
                stuck ? 1u : 0u);

  return !stuck;
}

static void completeAdcWorkflowSuccess() {
  adcFieldPowerDisabled = false;
  adcReady = true;
  lastFailedRecoveryMillis = 0;
  clearAdcCachedData();
  resetStuckTracking();
  clearAdcWorkflow();
}

static void completeAdcWorkflowFailure(const char *message) {
  if (message != nullptr) {
    Serial.println(message);
  }
  adcReady = readDiagnostics(true);
  lastFailedRecoveryMillis = millis();
  clearAdcWorkflow();
}

static void beginAdcRecovery(const char *reason) {
  Serial.printf("MAX22530 recovery triggered at %lu ms (loop=%lu)\n",
                millis(),
                static_cast<unsigned long>(networkingLoopCount));
  if (reason != nullptr) {
    Serial.printf("ADC recovery requested: %s\n", reason);
  }
  recoveryTriggerCount++;
  resetStuckTracking();
  setAdcWorkflow(AdcWorkflow::Recovery, 0, 0, reason);
}

static void beginMax22530Init() {
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin();
  Serial.printf("ADC SPI configured: CS=%d, ADC1_ADDR=0x%02X\n", ADC_CS_PIN, REG_ADC1);
  adcReady = false;
  setAdcWorkflow(AdcWorkflow::Init, 0, ADC_INIT_STABILIZE_MS, "startup");
}

static void serviceAdcWorkflow() {
  const unsigned long now = millis();
  if (adcWorkflow == AdcWorkflow::Idle || now < adcWorkflowReadyAtMillis) {
    return;
  }

  switch (adcWorkflow) {
    case AdcWorkflow::Init:
      switch (adcWorkflowStep) {
        case 0:
          if (!readDiagnostics(true)) {
            Serial.println("MAX22530 init diagnostics failed. Attempting staged recovery.");
            beginAdcRecovery("init diagnostics failed");
            return;
          }
          writeReconfigureRegisters();
          setAdcWorkflow(AdcWorkflow::Init, 1, MAX22530_ENABLE_FIELD_POWER_WAIT_MS, adcWorkflowReason);
          return;
        case 1:
          if (!readDiagnostics(true)) {
            Serial.println("MAX22530 register reconfiguration failed during init.");
            beginAdcRecovery("register reconfiguration failed");
            return;
          }
          Serial.println("Note: ensure ADC CS is not shared with Ethernet CS (typically pin 10).\n");
          completeAdcWorkflowSuccess();
          return;
      }
      break;
    case AdcWorkflow::Recovery:
      switch (adcWorkflowStep) {
        case 0:
          if (!readDiagnostics(true)) {
            completeAdcWorkflowFailure("MAX22530 recovery aborted: PROD_ID mismatch indicates an SPI communication problem.");
            return;
          }
          Serial.println("MAX22530 recovery step: enableFieldPower");
          writeEnableFieldPower();
          setAdcWorkflow(AdcWorkflow::Recovery, 1, MAX22530_ENABLE_FIELD_POWER_WAIT_MS, adcWorkflowReason);
          return;
        case 1:
          if (checkRecoveryResult("enableFieldPower")) {
            Serial.println("MAX22530 recovery resolved by enableFieldPower.");
            completeAdcWorkflowSuccess();
            return;
          }
          Serial.println("MAX22530 recovery step: clearADCFilters");
          writeClearAdcFilters();
          setAdcWorkflow(AdcWorkflow::Recovery, 2, MAX22530_CLEAR_FILTERS_WAIT_MS, adcWorkflowReason);
          return;
        case 2:
          if (checkRecoveryResult("clearADCFilters")) {
            Serial.println("MAX22530 recovery resolved by clearADCFilters.");
            completeAdcWorkflowSuccess();
            return;
          }
          Serial.println("MAX22530 recovery step: clearPOR");
          writeClearPor();
          setAdcWorkflow(AdcWorkflow::Recovery, 3, MAX22530_CLEAR_POR_WAIT_MS, adcWorkflowReason);
          return;
        case 3:
          if (checkRecoveryResult("clearPOR")) {
            Serial.println("MAX22530 recovery resolved by clearPOR.");
            completeAdcWorkflowSuccess();
            return;
          }
          Serial.println("MAX22530 recovery step: softReset");
          writeSoftReset();
          setAdcWorkflow(AdcWorkflow::Recovery, 4, MAX22530_SOFT_RESET_WAIT_MS, adcWorkflowReason);
          return;
        case 4:
          writeReconfigureRegisters();
          setAdcWorkflow(AdcWorkflow::Recovery, 5, MAX22530_ENABLE_FIELD_POWER_WAIT_MS, adcWorkflowReason);
          return;
        case 5:
          if (checkRecoveryResult("softReset")) {
            Serial.println("MAX22530 recovery resolved by softReset.");
            completeAdcWorkflowSuccess();
            return;
          }
          Serial.println("MAX22530 recovery step: hardReset");
          writeHardReset();
          setAdcWorkflow(AdcWorkflow::Recovery, 6, MAX22530_HARD_RESET_WAIT_MS, adcWorkflowReason);
          return;
        case 6:
          writeReconfigureRegisters();
          setAdcWorkflow(AdcWorkflow::Recovery, 7, MAX22530_ENABLE_FIELD_POWER_WAIT_MS, adcWorkflowReason);
          return;
        case 7:
          if (checkRecoveryResult("hardReset")) {
            Serial.println("MAX22530 recovery resolved by hardReset.");
            completeAdcWorkflowSuccess();
            return;
          }
          completeAdcWorkflowFailure("MAX22530 recovery failed: ADC remains stuck after all recovery steps.");
          return;
      }
      break;
    case AdcWorkflow::ManualEnableFieldPower:
      switch (adcWorkflowStep) {
        case 0:
          writeEnableFieldPower();
          setAdcWorkflow(AdcWorkflow::ManualEnableFieldPower, 1, MAX22530_ENABLE_FIELD_POWER_WAIT_MS, adcWorkflowReason);
          return;
        case 1:
          writeReconfigureRegisters();
          setAdcWorkflow(AdcWorkflow::ManualEnableFieldPower, 2, MAX22530_ENABLE_FIELD_POWER_WAIT_MS, adcWorkflowReason);
          return;
        case 2:
          adcFieldPowerDisabled = false;
          adcReady = readDiagnostics(true);
          if (!adcReady) {
            Serial.println("MAX22530 manual enable field power did not restore communications. Starting recovery.");
            lastFailedRecoveryMillis = 0;
            lastAdcRecoverMillis = 0;
            beginAdcRecovery("manual enable field power failed");
            return;
          }
          clearAdcCachedData();
          resetStuckTracking();
          clearAdcWorkflow();
          return;
      }
      break;
    case AdcWorkflow::ManualHardReset:
      switch (adcWorkflowStep) {
        case 0:
          writeHardReset();
          setAdcWorkflow(AdcWorkflow::ManualHardReset, 1, MAX22530_HARD_RESET_WAIT_MS, adcWorkflowReason);
          return;
        case 1:
          writeReconfigureRegisters();
          setAdcWorkflow(AdcWorkflow::ManualHardReset, 2, MAX22530_ENABLE_FIELD_POWER_WAIT_MS, adcWorkflowReason);
          return;
        case 2:
          adcFieldPowerDisabled = false;
          adcReady = readDiagnostics(true);
          if (!adcReady) {
            Serial.println("MAX22530 manual hard reset did not restore communications. Starting recovery.");
            lastFailedRecoveryMillis = 0;
            lastAdcRecoverMillis = 0;
            beginAdcRecovery("manual hard reset failed");
            return;
          }
          clearAdcCachedData();
          resetStuckTracking();
          clearAdcWorkflow();
          return;
      }
      break;
    case AdcWorkflow::Idle:
      return;
  }

  clearAdcWorkflow();
}

static void recoverMax22530IfDue(const char *reason) {
  const unsigned long now = millis();
  if (adcWorkflow != AdcWorkflow::Idle) {
    return;
  }
  if (lastFailedRecoveryMillis != 0 && (now - lastFailedRecoveryMillis) < ADC_RECOVER_COOLDOWN_MS) {
    return;
  }
  if (now - lastAdcRecoverMillis < 50UL) {
    return;
  }

  lastAdcRecoverMillis = now;
  beginAdcRecovery(reason);
}

static const char *spiControlActionName(SpiControlAction action) {
  switch (action) {
    case SpiControlAction::Stop:
      return "stop";
    case SpiControlAction::Start:
      return "start";
    case SpiControlAction::Reinit:
      return "reinit";
    case SpiControlAction::None:
    default:
      return "none";
  }
}

static void queueSpiControlAction(SpiControlAction action) {
  pendingSpiControlAction = action;
}

static void processPendingSpiControlAction() {
  const SpiControlAction action = pendingSpiControlAction;
  if (action == SpiControlAction::None) {
    return;
  }

  pendingSpiControlAction = SpiControlAction::None;

  if (action == SpiControlAction::Stop) {
    clearAdcWorkflow();
    adcReady = false;
    cachedAdcFresh = false;
    pinMode(ADC_CS_PIN, OUTPUT);
    digitalWrite(ADC_CS_PIN, HIGH);
    SPI.end();
    spiBusEnabled = false;
    Serial.println("SPI control: SPI.end() executed (manual stop). Ethernet may be unavailable until SPI is started.");
    return;
  }

  if (action == SpiControlAction::Start) {
    SPI.begin();
    spiBusEnabled = true;
    beginMax22530Init();
    restartNetworkingService("Manual SPI start");
    Serial.println("SPI control: SPI.begin() executed (manual start).");
    return;
  }

  if (action == SpiControlAction::Reinit) {
    clearAdcWorkflow();
    adcReady = false;
    cachedAdcFresh = false;
    pinMode(ADC_CS_PIN, OUTPUT);
    digitalWrite(ADC_CS_PIN, HIGH);
    SPI.end();
    SPI.begin();
    spiBusEnabled = true;
    beginMax22530Init();
    restartNetworkingService("Manual SPI reinit");
    Serial.println("SPI control: SPI bus reinitialized (manual reinit).");
  }
}

static void restartNetworkingService(const char *reason) {
  const unsigned long now = millis();
  if (now - lastNetworkRestartMillis < NETWORK_RESTART_COOLDOWN_MS) {
    return;
  }

  lastNetworkRestartMillis = now;
  networkingRestarts++;
  lastAdcPollMillis = now;
  lastHttpResponseMillis = now;
  // Only arm the HTTP stall watchdog when active API traffic is observed.
  httpWatchdogArmed = false;

  Serial.printf("Networking watchdog restart: %s\n", reason);

  IPAddress ip = constants::APP_IP;
  IPAddress netmask = constants::APP_NETMASK;
  IPAddress gw = constants::APP_GW;
  IPAddress dns = constants::APP_DNS;

  const bool ethOK = Ethernet.begin(ip, netmask, gw);
  if (!ethOK) {
    Serial.println("Ethernet restart failed (begin returned false)");
  } else {
    Ethernet.setDNSServerIP(dns);
    Serial.print("Ethernet restarted, IP: ");
    Serial.println(Ethernet.localIP());
  }

  if (!serverStarted) {
    server.begin();
    serverStarted = true;
  }
}

// OTA callback
static void onUpdateReady() {
  Serial.println("Update is ready - scheduling apply from networking_loop()");
  pendingUpdate = true;
}

// Handlers
static void handleRoot(AsyncWebServerRequest *request) {
  if (rejectIfHttpOverloaded(request)) {
    return;
  }

  const char html[] =
    "<!DOCTYPE html>"
    "<html><head><title>Teensy APP</title></head><body>"
    "<div style=\"position:fixed;right:8px;top:8px;\"><button onclick=\"location.href='/'\">Home</button></div>"
    "<h1>Teensy 4.1 APP</h1>"
    "<p>Firmware updates:</p>"
    "<button onclick=\"location.href='/flasherx'\">FlasherX OTA</button> "
    "<button onclick=\"location.href='/teensy_ota'\">Teensy OTA</button>"
      "<h3>Analog readings:</h3>"
      "<div>ADC raw code (12-bit): <span id=\"a13raw\">--</span></div>"
      "<div>ADC input voltage (AIN to GND): <span id=\"a13v\">--</span> V</div>"
      "<div>Shunt current (I = V / 47 ohm): <span id=\"a13ma\">--</span> mA</div>"
      "<div>Isolated ADC state: <span id=\"a13state\">--</span></div>"
      "<div>Isolated ADC self-heal: <span id=\"a13heal\">--</span></div>"
      "<hr>"
      "<div>A14 raw code (12-bit): <span id=\"a14raw\">--</span></div>"
      "<div>A14 voltage (AIN to GND): <span id=\"a14v\">--</span> V</div>"
      "<div>A14 loop current (560||560||560 = 186.67 ohm): <span id=\"a14ma\">--</span> mA</div>"
      "<div>A14 scaled temperature (4-20 mA => 250C-2000C): <span id=\"a14c\">--</span> C</div>"
      "<div>Local analog state: <span id=\"a14state\">--</span></div>"
      "<hr>"
      "<h3>ADC controls:</h3>"
      "<button onclick=\"triggerAdcAction('/adc/dispower','Set DISPWR bit and disable field power?')\">Trigger DISPWR</button> "
      "<button onclick=\"triggerAdcAction('/adc/enable_field_power','Clear DISPWR bit and re-enable field power?')\">Enable Field Power</button> "
      "<button onclick=\"triggerAdcAction('/adc/reset','Issue MAX22530 hard reset?')\">Trigger Reset</button>"
      "<h3>SPI bus controls:</h3>"
      "<button onclick=\"triggerAdcAction('/spi/stop','Call SPI.end() for the shared bus? Ethernet may drop until start/reinit.')\">SPI.end()</button> "
      "<button onclick=\"triggerAdcAction('/spi/start','Call SPI.begin() and re-init ADC/networking?')\">SPI.begin()</button> "
      "<button onclick=\"triggerAdcAction('/spi/reinit','Fully reinitialize SPI bus now?')\">SPI Reinit</button>"
      "<div>SPI bus state: <span id=\"spibus\">--</span></div>"
      "<div id=\"adcctl\">Idle</div>"
      "<div>ADC status: <a href=\"/adc/status\">/adc/status</a></div>"
      "<div>HTTP diagnostics: <a href=\"/diag/http\">/diag/http</a></div>"
      "<script>let adcPollBusy=false;async function pollAdc(){if(adcPollBusy)return;adcPollBusy=true;const ctl=new AbortController();const t=setTimeout(()=>ctl.abort(),1500);try{const r=await fetch('/adc/status?ts='+Date.now(),{cache:'no-store',keepalive:false,signal:ctl.signal});const s=await r.json();document.getElementById('a13raw').innerText=String(Number(s.raw)||0);document.getElementById('a13v').innerText=(Number(s.volts)||0).toFixed(6);document.getElementById('a13ma').innerText=(Number(s.current_ma)||0).toFixed(5);document.getElementById('a13state').innerText=s.field_power_disabled?'disabled by DISPWR':(s.isolated_adc_active?'active':'inactive');document.getElementById('a13heal').innerText=s.auto_recovery_enabled?'enabled':'paused';document.getElementById('a14raw').innerText=String(Number(s.raw_a14)||0);document.getElementById('a14v').innerText=(Number(s.volts_a14)||0).toFixed(6);document.getElementById('a14ma').innerText=(Number(s.current_ma_a14)||0).toFixed(5);document.getElementById('a14c').innerText=(Number(s.temp_c_a14)||0).toFixed(2);document.getElementById('a14state').innerText=s.local_analog_active?'active':'inactive';document.getElementById('spibus').innerText=s.spi_bus_enabled?(s.spi_action_pending&&s.spi_action_pending!=='none'?'enabled (pending '+s.spi_action_pending+')':'enabled'):(s.spi_action_pending&&s.spi_action_pending!=='none'?'disabled (pending '+s.spi_action_pending+')':'disabled');}catch(_e){}finally{clearTimeout(t);adcPollBusy=false;}}async function triggerAdcAction(path,msg){if(msg&&!confirm(msg))return;const out=document.getElementById('adcctl');out.innerText='Sending '+path+'...';try{const r=await fetch(path,{method:'POST',cache:'no-store'});const t=await r.text();out.innerText=t;}catch(e){out.innerText='Request failed';}}setInterval(pollAdc,1000);pollAdc();</script>"
    "</body></html>";

  sendNoKeepAlive(request, 200, "text/html", html);
}

// Teensy OTA helper page (links into the OTA endpoint but provides a return link)
static void handleTeensyOtaPage(AsyncWebServerRequest *request) {
  if (rejectIfHttpOverloaded(request)) {
    return;
  }

  const char html[] =
    "<!DOCTYPE html>"
    "<html><head><title>Teensy OTA</title></head><body>"
    "<div style=\"position:fixed;right:8px;top:8px;\"><button onclick=\"location.href='/'\">Home</button></div>"
    "<h1>Teensy OTA</h1>"
    "<p>Upload a firmware file using the form below. After upload the device may reboot.</p>"
    "<form method=POST action=\"/update\" enctype=\"multipart/form-data\">"
    "<input type=file name=file><br><br>"
    "<input type=submit value=Upload>"
    "</form>"
    "<p><a href=\"/\">Return to control panel</a></p>"
    "</body></html>";

  sendNoKeepAlive(request, 200, "text/html", html);
}

static void handleNotFound(AsyncWebServerRequest *request) {
  if (rejectIfHttpOverloaded(request)) {
    return;
  }

  const char html[] =
    "<!DOCTYPE html>"
    "<html><head><title>Not found</title></head><body>"
    "<div style=\"position:fixed;right:8px;top:8px;\"><button onclick=\"location.href='/'\">Home</button></div>"
    "<h1>404 - Not found</h1>"
    "<p><a href=\"/\">Go home</a></p>"
    "</body></html>";
  sendNoKeepAlive(request, 404, "text/html", html);
}

// FlasherX upload page
static void handleFlasherXPage(AsyncWebServerRequest *request) {
  if (rejectIfHttpOverloaded(request)) {
    return;
  }

  const char html[] =
    "<!DOCTYPE html>"
    "<html><head><title>FlasherX OTA Upload</title></head><body>"
    "<div style=\"position:fixed;right:8px;top:8px;\"><button onclick=\"location.href='/'\">Home</button></div>"
    "<h1>FlasherX OTA Upload</h1>"
    "<p>Upload an Intel HEX firmware file. The device will save it to the SD card and run FlasherX to apply the update.</p>"
    "<form method=POST action=\"/flasherx/upload\" enctype=\"multipart/form-data\">"
    "<input type=file name=file><br><br>"
    "<input type=submit value=Upload>"
    "</form>"
    "</body></html>";

  sendNoKeepAlive(request, 200, "text/html", html);
}

// Upload state
static File uploadFile;

// Upload callback: invoked with file chunks
static void handleFlasherXUpload(AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {
  (void)request; (void)filename;
  if (index == 0) {
    // start of upload - init SD and open file
    if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("SD init failed for FlasherX upload");
      return;
    }
    // remove any previous file
    if (SD.exists(constants::UPLOAD_NAME)) SD.remove(constants::UPLOAD_NAME);
    uploadFile = SD.open(constants::UPLOAD_NAME, FILE_WRITE);
    if (!uploadFile) {
      Serial.println("Failed to open file on SD for upload");
      return;
    }
    Serial.println("Started FlasherX upload to SD");
  }

  if (uploadFile) {
    uploadFile.write(data, len);
  }

  if (final) {
    // finished writing
    if (uploadFile) {
      uploadFile.close();
      Serial.println("FlasherX upload complete, invoking update_firmware()");

      // create flash buffer
      uint32_t buffer_addr = 0;
      uint32_t buffer_size = 0;
      if (firmware_buffer_init(&buffer_addr, &buffer_size) == 0) {
        Serial.println("unable to create firmware buffer");
        firmware_buffer_free(buffer_addr, buffer_size);
        return;
      }

      // open file for reading
      File hexFile = SD.open(constants::UPLOAD_NAME, FILE_READ);
      if (!hexFile) {
        Serial.println("Failed to open uploaded hex file for reading");
        firmware_buffer_free(buffer_addr, buffer_size);
        return;
      }

      // call FlasherX update using SD file as Stream* input and Serial as output
      update_firmware(&hexFile, &Serial, buffer_addr, buffer_size);

      // cleanup (update_firmware() usually reboots on success)
      hexFile.close();
      firmware_buffer_free(buffer_addr, buffer_size);
      Serial.println("FlasherX update finished (or aborted)");
    }
  }
}



// Generic analog handler helper
static void handleAnalogGeneric(AsyncWebServerRequest *request, int analogNum) {
  if (rejectIfHttpOverloaded(request)) {
    return;
  }

  if (!adcReady && analogNum == 13) {
    adcNotReadyResponses++;
    sendNoKeepAlive(request, 503, "text/plain", "ADC_NOT_READY");
    return;
  }

  float output = 0.0f;
  switch (analogNum) {
    case 13:
      output = adcCountsToMilliamps(cachedShuntA0Counts);
      break;
    case 14: {
      const float volts = teensyAdcCountsToVolts(cachedA14Counts);
      output = voltsToMilliamps(volts, A14_SHUNT_RESISTOR_OHMS);
      break;
    }
    default:
      sendNoKeepAlive(request, 404, "text/plain", "BAD_ANALOG_CHANNEL");
      return;
  }
  
  char buf[16];
  snprintf(buf, sizeof(buf), "%.5f", output);
  sendNoKeepAlive(request, 200, "text/plain", buf);
}

static void handleAnalog13(AsyncWebServerRequest *r) { handleAnalogGeneric(r, 13); }
static void handleAnalog14(AsyncWebServerRequest *r) { handleAnalogGeneric(r, 14); }

static void handleAdcStatus(AsyncWebServerRequest *request) {
  if (rejectIfHttpOverloaded(request)) {
    return;
  }

  httpWatchdogArmed = true;
  adcStatusRequests++;
  lastAdcStatusMillis = millis();
  const int raw = cachedShuntA0Counts;
  const float volts = adcCountsToVolts(raw);
  const float currentMa = adcCountsToMilliamps(raw);

  const int rawA14 = cachedA14Counts;
  const float voltsA14 = teensyAdcCountsToVolts(rawA14);
  const float currentMaA14 = voltsToMilliamps(voltsA14, A14_SHUNT_RESISTOR_OHMS);
  const float tempCA14 = loopCurrentToTempC(currentMaA14);

  String json;
  json.reserve(512);
  json += "{\"ready\":";
  json += adcReady ? "true" : "false";
  json += ",\"fresh\":";
  json += cachedAdcFresh ? "true" : "false";
  json += ",\"interface\":\"SPI\",\"cs_pin\":";
  json += ADC_CS_PIN;
  json += ",\"adc1_reg\":\"0x";
  char regBuf[5];
  snprintf(regBuf, sizeof(regBuf), "%02X", REG_ADC1);
  json += regBuf;
  json += "\",\"prod_id\":";
  json += cachedProdId;
  json += ",\"control\":";
  json += cachedControl;
  json += ",\"field_power_disabled\":";
  json += adcFieldPowerDisabled ? "true" : "false";
  json += ",\"auto_recovery_enabled\":";
  json += adcFieldPowerDisabled ? "false" : "true";
  json += ",\"isolated_adc_active\":";
  json += (adcReady && !adcFieldPowerDisabled) ? "true" : "false";
  json += ",\"local_analog_active\":true";
  json += ",\"spi_bus_enabled\":";
  json += spiBusEnabled ? "true" : "false";
  json += ",\"spi_action_pending\":\"";
  json += spiControlActionName(pendingSpiControlAction);
  json += "\"";
  json += ",\"raw\":";
  json += raw;
  json += ",\"volts\":";
  json += String(volts, 6);
  json += ",\"current_ma\":";
  json += String(currentMa, 5);
  json += ",\"shunt_ohms\":";
  json += String(SHUNT_RESISTOR_OHMS, 1);
  json += ",\"vref\":";
  json += String(ADC_REF_VOLTS, 3);
  json += ",\"int_status\":";
  json += cachedInterruptStatus;
  json += ",\"raw_a14\":";
  json += rawA14;
  json += ",\"volts_a14\":";
  json += String(voltsA14, 6);
  json += ",\"current_ma_a14\":";
  json += String(currentMaA14, 5);
  json += ",\"shunt_ohms_a14\":";
  json += String(A14_SHUNT_RESISTOR_OHMS, 2);
  json += ",\"temp_c_a14\":";
  json += String(tempCA14, 2);
  json += ",\"temp_scale_min_c\":";
  json += String(LOOP_TEMP_MIN_C, 1);
  json += ",\"temp_scale_max_c\":";
  json += String(LOOP_TEMP_MAX_C, 1);
  json += '}';
  sendNoKeepAlive(request, 200, "application/json", json.c_str());
}

static void handleHttpDiag(AsyncWebServerRequest *request) {
  char json[256];
  const unsigned long now = millis();
  snprintf(json, sizeof(json),
           "{\"uptime_ms\":%lu,\"http_responses\":%lu,\"adc_status_requests\":%lu,\"adc_not_ready_responses\":%lu,\"http_throttled_requests\":%lu,\"network_restarts\":%lu,\"http_watchdog_armed\":%s,\"http_watchdog_timeout_ms\":%lu,\"last_http_response_ms_ago\":%lu,\"last_adc_status_ms_ago\":%lu}",
           now,
           static_cast<unsigned long>(httpResponsesSent),
           static_cast<unsigned long>(adcStatusRequests),
           static_cast<unsigned long>(adcNotReadyResponses),
           static_cast<unsigned long>(httpThrottledRequests),
           static_cast<unsigned long>(networkingRestarts),
           httpWatchdogArmed ? "true" : "false",
           HTTP_RESPONSE_WATCHDOG_MS,
           (lastHttpResponseMillis == 0) ? 0UL : (now - lastHttpResponseMillis),
           (lastAdcStatusMillis == 0) ? 0UL : (now - lastAdcStatusMillis));
  sendNoKeepAlive(request, 200, "application/json", json);
}

static void handleAdcDispower(AsyncWebServerRequest *request) {
  if (rejectIfHttpOverloaded(request)) {
    return;
  }

  clearAdcWorkflow();
  const uint16_t control = readRegister(REG_CONTROL);
  const uint16_t newControl = static_cast<uint16_t>(control | CONTROL_DISPWR);
  writeRegister(REG_CONTROL, newControl);
  adcFieldPowerDisabled = true;
  adcReady = false;
  cachedControl = newControl;
  cachedAdcFresh = false;
  clearAdcCachedData();
  resetStuckTracking();

  Serial.printf("MAX22530 manual DISPWR request: CONTROL 0x%04X -> 0x%04X\n",
                control,
                newControl);

  String json;
  json.reserve(160);
  json += "{\"action\":\"dispower\",\"ok\":";
  json += "true";
  json += ",\"control\":";
  json += cachedControl;
  json += ",\"field_power_disabled\":true";
  json += ",\"auto_recovery_enabled\":false";
  json += ",\"prod_id\":";
  json += cachedProdId;
  json += ",\"int_status\":";
  json += cachedInterruptStatus;
  json += '}';
  sendNoKeepAlive(request, 200, "application/json", json.c_str());
}

static void handleAdcEnableFieldPower(AsyncWebServerRequest *request) {
  if (rejectIfHttpOverloaded(request)) {
    return;
  }

  Serial.println("MAX22530 manual enable field power requested");
  clearAdcWorkflow();
  adcFieldPowerDisabled = false;
  adcReady = false;
  lastFailedRecoveryMillis = 0;
  lastAdcRecoverMillis = 0;
  clearAdcCachedData();
  resetStuckTracking();
  setAdcWorkflow(AdcWorkflow::ManualEnableFieldPower, 0, 0, "manual enable field power");

  String json;
  json.reserve(224);
  json += "{\"action\":\"enable_field_power\",\"ok\":true";
  json += ",\"queued\":true";
  json += ",\"control\":";
  json += cachedControl;
  json += ",\"field_power_disabled\":false";
  json += ",\"auto_recovery_enabled\":true";
  json += ",\"prod_id\":";
  json += cachedProdId;
  json += ",\"int_status\":";
  json += cachedInterruptStatus;
  json += '}';
  sendNoKeepAlive(request, 200, "application/json", json.c_str());
}

static void handleAdcManualReset(AsyncWebServerRequest *request) {
  if (rejectIfHttpOverloaded(request)) {
    return;
  }

  Serial.println("MAX22530 manual hard reset requested");
  clearAdcWorkflow();
  adcFieldPowerDisabled = false;
  adcReady = false;
  lastFailedRecoveryMillis = 0;
  lastAdcRecoverMillis = 0;
  clearAdcCachedData();
  resetStuckTracking();
  setAdcWorkflow(AdcWorkflow::ManualHardReset, 0, 0, "manual hard reset");

  String json;
  json.reserve(192);
  json += "{\"action\":\"hard_reset\",\"ok\":true";
  json += ",\"queued\":true";
  json += ",\"control\":";
  json += cachedControl;
  json += ",\"field_power_disabled\":false";
  json += ",\"auto_recovery_enabled\":true";
  json += ",\"prod_id\":";
  json += cachedProdId;
  json += ",\"int_status\":";
  json += cachedInterruptStatus;
  json += '}';
  sendNoKeepAlive(request, 200, "application/json", json.c_str());
}

static void handleSpiStop(AsyncWebServerRequest *request) {
  if (rejectIfHttpOverloaded(request)) {
    return;
  }

  queueSpiControlAction(SpiControlAction::Stop);
  const char json[] = "{\"action\":\"spi_stop\",\"queued\":true,\"warning\":\"SPI bus is shared; Ethernet may stop until SPI start/reinit\"}";
  sendNoKeepAlive(request, 200, "application/json", json);
}

static void handleSpiStart(AsyncWebServerRequest *request) {
  if (rejectIfHttpOverloaded(request)) {
    return;
  }

  queueSpiControlAction(SpiControlAction::Start);
  const char json[] = "{\"action\":\"spi_start\",\"queued\":true}";
  sendNoKeepAlive(request, 200, "application/json", json);
}

static void handleSpiReinit(AsyncWebServerRequest *request) {
  if (rejectIfHttpOverloaded(request)) {
    return;
  }

  queueSpiControlAction(SpiControlAction::Reinit);
  const char json[] = "{\"action\":\"spi_reinit\",\"queued\":true}";
  sendNoKeepAlive(request, 200, "application/json", json);
}

// -----------------------------------------------------------------------------
// networking init / loop
// -----------------------------------------------------------------------------

void networking_init() {
  analogReadResolution(12);
  pinMode(ANALOG_PIN_14, INPUT);

  beginMax22530Init();
  lastAdcPollMillis = millis();
  Serial.println("MAX22530 SPI ADC init scheduled");

  Serial.println("Starting Ethernet with static IP (APP)...");
  IPAddress ip = constants::APP_IP;
  IPAddress netmask = constants::APP_NETMASK;
  IPAddress gw = constants::APP_GW;
  IPAddress dns = constants::APP_DNS;

  bool ethOK = Ethernet.begin(ip, netmask, gw);
  if (!ethOK) {
    Serial.println("Ethernet.begin(ip, netmask, gw) FAILED - continuing without Ethernet");
  } else {
    Serial.println("Ethernet.begin(ip, netmask, gw) OK");
  }

  const unsigned long linkTimeout = constants::LINK_TIMEOUT_MS;
  Serial.println("Waiting for link (up to 5s)...");
  if (!Ethernet.waitForLink(linkTimeout)) {
    Serial.println("No Ethernet link detected after 5s - web/OTA may be unavailable.");
  } else {
    Serial.println("Ethernet link is UP.");
  }

  Ethernet.setDNSServerIP(dns);

  IPAddress cur = Ethernet.localIP();
  Serial.print("Current IP address (APP): ");
  Serial.println(cur);

  // Register OTA callback
  ota.registerCallback(onUpdateReady);
  Serial.println("OTA callback registered");

  // Apply connection behavior globally so every HTTP response closes cleanly.
  if (!defaultHeadersConfigured) {
    DefaultHeaders::Instance().addHeader("Connection", "close");
    DefaultHeaders::Instance().addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    defaultHeadersConfigured = true;
  }

  // Routes only need registration once.
  if (!routesRegistered) {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/flasherx", HTTP_GET, handleFlasherXPage);
    server.on("/teensy_ota", HTTP_GET, handleTeensyOtaPage);
    server.on("/update", HTTP_GET, handleTeensyOtaPage);

    server.on("/analog/13", HTTP_GET, handleAnalog13);
    server.on("/analog/14", HTTP_GET, handleAnalog14);
    server.on("/adc/status", HTTP_GET, handleAdcStatus);
    server.on("/adc/dispower", HTTP_POST, handleAdcDispower);
    server.on("/adc/enable_field_power", HTTP_POST, handleAdcEnableFieldPower);
    server.on("/adc/reset", HTTP_POST, handleAdcManualReset);
    server.on("/spi/stop", HTTP_POST, handleSpiStop);
    server.on("/spi/start", HTTP_POST, handleSpiStart);
    server.on("/spi/reinit", HTTP_POST, handleSpiReinit);
    server.on("/diag/http", HTTP_GET, handleHttpDiag);

    server.on("/flasherx/upload", HTTP_POST,
              [](AsyncWebServerRequest *request){
                const char resp[] = "<!DOCTYPE html><html><body><h1>Upload received</h1><p><a href='/'>Return to control panel</a></p></body></html>";
                sendNoKeepAlive(request, 200, "text/html", resp);
              },
              handleFlasherXUpload);

    server.onNotFound(handleNotFound);
    routesRegistered = true;
  }

  if (!serverStarted) {
    server.begin();
    serverStarted = true;
  }
  Serial.print("HTTP APP server started at http://");
  Serial.print(cur[0]); Serial.print(".");
  Serial.print(cur[1]); Serial.print(".");
  Serial.print(cur[2]); Serial.print(".");
  Serial.print(cur[3]); Serial.println("/");
  Serial.println("Use /update for firmware upload.");
}

void networking_loop() {
  const unsigned long now = millis();
  networkingLoopCount++;

  processPendingSpiControlAction();

  serviceAdcWorkflow();

  if (!adcFieldPowerDisabled && !adcReady && (now - lastAdcRecoverMillis >= ADC_NOT_READY_RETRY_MS)) {
    recoverMax22530IfDue("MAX22530 not ready; retrying recovery");
  }

  if (now - lastAnalogSampleMillis >= ANALOG_POLL_INTERVAL_MS) {
    lastAnalogSampleMillis = now;
    int a13 = 0;
    const int a14 = analogRead(ANALOG_PIN_14);
    if (adcReady && !adcFieldPowerDisabled) {
      a13 = static_cast<int>(readAdc1());
      if (isStuck(static_cast<uint16_t>(a13))) {
        if (stuckCount < 0xFFFFu) {
          stuckCount++;
        }
        if (stuckCount >= ADC_STUCK_RECOVERY_TRIGGER) {
          recoverMax22530IfDue("MAX22530 ADC reading is stuck");
        }
      } else {
        stuckCount = 0;
      }
    }

    a13Samples[a13SampleIndex] = a13;

    a13SampleIndex = (a13SampleIndex + 1) % ANALOG_MEDIAN_WINDOW;
    if (a13SampleCount < ANALOG_MEDIAN_WINDOW) {
      a13SampleCount++;
    }

    cachedShuntA0Counts = computeMedian(a13Samples, a13SampleCount);
    cachedA14Counts = a14;
  }

  if (adcReady && !adcFieldPowerDisabled && (now - lastAdcPollMillis >= ADC_POLL_WATCHDOG_MS)) {
    restartNetworkingService("ADC poll timeout > 60s");
  }

  if (httpWatchdogArmed &&
      lastHttpResponseMillis != 0 &&
      (now - lastHttpResponseMillis >= HTTP_RESPONSE_WATCHDOG_MS)) {
    restartNetworkingService("HTTP response stall > 3s");
  }

  if ((adcReady || adcFieldPowerDisabled) && (now - lastAnalogPrintMillis >= ANALOG_PRINT_INTERVAL_MS)) {
    lastAnalogPrintMillis = now;
    const int raw = cachedShuntA0Counts;
    const float volts = adcCountsToVolts(raw);
    const float currentMa = adcCountsToMilliamps(raw);
    const int rawA14 = cachedA14Counts;
    const float voltsA14 = teensyAdcCountsToVolts(rawA14);
    const float currentMaA14 = voltsToMilliamps(voltsA14, A14_SHUNT_RESISTOR_OHMS);
    const float tempCA14 = loopCurrentToTempC(currentMaA14);
    Serial.printf("ADC13 raw=%d voltage=%.6f V current=%.5f mA fresh=%u field_power_disabled=%u stuck_count=%u same_value_count=%u int_status=0x%04X | A14 raw=%d voltage=%.6f V current=%.5f mA temp=%.2f C\n",
                  raw,
                  volts,
                  currentMa,
                  cachedAdcFresh ? 1u : 0u,
            adcFieldPowerDisabled ? 1u : 0u,
                  stuckCount,
                  sameAdcReadingCount,
                  cachedInterruptStatus,
                  rawA14,
                  voltsA14,
                  currentMaA14,
                  tempCA14);
  }

  if (pendingUpdate) {
    pendingUpdate = false;
    Serial.println("Applying update now (networking)");
    ota.applyUpdate();
  }

  if (now - lastHttpDiagPrintMillis >= HTTP_DIAG_PRINT_INTERVAL_MS) {
    lastHttpDiagPrintMillis = now;
    Serial.printf("HTTP diag: responses=%lu adc_status=%lu adc_not_ready=%lu throttled=%lu restarts=%lu watchdog=%u last_http_ms_ago=%lu\n",
                  static_cast<unsigned long>(httpResponsesSent),
                  static_cast<unsigned long>(adcStatusRequests),
                  static_cast<unsigned long>(adcNotReadyResponses),
                  static_cast<unsigned long>(httpThrottledRequests),
                  static_cast<unsigned long>(networkingRestarts),
                  httpWatchdogArmed ? 1u : 0u,
                  (lastHttpResponseMillis == 0) ? 0UL : (now - lastHttpResponseMillis));
  }
}
