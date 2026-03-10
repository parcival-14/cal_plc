#include <Arduino.h>
#include <SPI.h>

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

// MAX22530/31-style SPI ADC access
static SPISettings max22530SPI(1000000, MSBFIRST, SPI_MODE0);
static bool adcReady = false;
static constexpr float ADC_REF_VOLTS = 1.8f;
static constexpr float ADC_MAX_CODE_12BIT = 4095.0f;
static constexpr float SHUNT_RESISTOR_OHMS = 90.6f;

// MAX2253x register map and control bits used by this app.
static constexpr uint8_t REG_PROD_ID = 0x00;
static constexpr uint8_t REG_ADC1 = 0x01;
static constexpr uint8_t REG_INTERRUPT_STATUS = 0x12;
static constexpr uint8_t REG_INTERRUPT_ENABLE = 0x13;
static constexpr uint8_t REG_CONTROL = 0x14;
static constexpr uint16_t CONTROL_CLRPOR = (1u << 2);
static constexpr uint16_t IE_EEOC = (1u << 12);

// Cached analog values (kept in sync with Serial prints)
static volatile int cachedShuntA0Counts = 0;
static constexpr int ANALOG_SMOOTH_WINDOW = 100;
static int a13Samples[ANALOG_SMOOTH_WINDOW] = {0};
static int a13SampleIndex = 0;
static int a13SampleCount = 0;
static long a13SampleSum = 0;
static volatile unsigned long lastAnalogPrintMillis = 0;
static constexpr unsigned long ANALOG_PRINT_INTERVAL_MS = 500UL;
static int lastRawPrintValue = -1;
static uint8_t sameRawPrintCount = 0;
static volatile bool cachedAdcFresh = false;
static volatile uint16_t cachedInterruptStatus = 0;
static volatile unsigned long lastAdcPollMillis = 0;
static volatile unsigned long lastNetworkRestartMillis = 0;
static constexpr unsigned long ADC_POLL_WATCHDOG_MS = 60000UL;
static constexpr unsigned long NETWORK_RESTART_COOLDOWN_MS = 15000UL;

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

// Timestamp (ms) of last periodic analog sample
static volatile unsigned long lastAnalogSampleMillis = 0;

// Header format: bits[7:2]=A[5:0], bit1=W/R, bit0=BURST.
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

static uint16_t max22530ReadRegister(uint8_t regAddr) {
  return transfer24(makeHeader(regAddr, false, false), 0x0000);
}

static void max22530WriteRegister(uint8_t regAddr, uint16_t value) {
  (void)transfer24(makeHeader(regAddr, true, false), value);
}

// Reads ADC1..ADC4 and interrupt status in one transaction (11 bytes, no CRC).
static void max22530BurstRead(uint8_t startAddr, uint16_t outRegs[5]) {
  uint8_t tx[11] = {0};
  uint8_t rx[11] = {0};
  tx[0] = makeHeader(startAddr, false, true);

  SPI.beginTransaction(max22530SPI);
  digitalWrite(ADC_CS_PIN, LOW);
  for (int i = 0; i < 11; ++i) {
    rx[i] = SPI.transfer(tx[i]);
  }
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.endTransaction();

  outRegs[0] = (static_cast<uint16_t>(rx[1]) << 8) | rx[2];
  outRegs[1] = (static_cast<uint16_t>(rx[3]) << 8) | rx[4];
  outRegs[2] = (static_cast<uint16_t>(rx[5]) << 8) | rx[6];
  outRegs[3] = (static_cast<uint16_t>(rx[7]) << 8) | rx[8];
  outRegs[4] = (static_cast<uint16_t>(rx[9]) << 8) | rx[10];
}

static uint16_t adcCode12(uint16_t reg) {
  return static_cast<uint16_t>(reg & 0x0FFF);
}

static bool adcFresh(uint16_t reg) {
  // Datasheet: bit15 == 0 indicates updated data since last read.
  return (reg & 0x8000u) == 0;
}

static bool initMax22530Spi() {
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin();

  const uint16_t prod = max22530ReadRegister(REG_PROD_ID);
  const uint16_t ist = max22530ReadRegister(REG_INTERRUPT_STATUS);
  max22530WriteRegister(REG_INTERRUPT_ENABLE, IE_EEOC);
  max22530WriteRegister(REG_CONTROL, CONTROL_CLRPOR);

  Serial.printf("ADC SPI configured: CS=%d, ADC1_ADDR=0x%02X\n", ADC_CS_PIN, REG_ADC1);
  Serial.printf("MAX2253x PROD_ID=0x%04X INT_STATUS=0x%04X\n", prod, ist);
  Serial.println("Note: ensure ADC CS is not shared with Ethernet CS (typically pin 10).\n");
  // Optional: add a readback/ID verification here if your register map supports it.
  return true;
}

static void restartNetworkingService(const char *reason) {
  const unsigned long now = millis();
  if (now - lastNetworkRestartMillis < NETWORK_RESTART_COOLDOWN_MS) {
    return;
  }

  lastNetworkRestartMillis = now;
  lastAdcPollMillis = now;

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
      "<div>Shunt current (90.6 ohm): <span id=\"a13ma\">--</span> mA</div>"
      "<div>ADC status: <a href=\"/adc/status\">/adc/status</a></div>"
      "<script>function pollAdc(){fetch('/adc/status').then(r=>r.json()).then(s=>{document.getElementById('a13raw').innerText=String(Number(s.raw)||0);document.getElementById('a13v').innerText=(Number(s.volts)||0).toFixed(6);document.getElementById('a13ma').innerText=(Number(s.current_ma)||0).toFixed(5);}).catch(()=>{});}setInterval(pollAdc,1000);pollAdc();</script>"
    "</body></html>";

  request->send(200, "text/html", html);
}

// Teensy OTA helper page (links into the OTA endpoint but provides a return link)
static void handleTeensyOtaPage(AsyncWebServerRequest *request) {
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

  request->send(200, "text/html", html);
}

static void handleNotFound(AsyncWebServerRequest *request) {
  const char html[] =
    "<!DOCTYPE html>"
    "<html><head><title>Not found</title></head><body>"
    "<div style=\"position:fixed;right:8px;top:8px;\"><button onclick=\"location.href='/'\">Home</button></div>"
    "<h1>404 - Not found</h1>"
    "<p><a href=\"/\">Go home</a></p>"
    "</body></html>";
  request->send(404, "text/html", html);
}

// FlasherX upload page
static void handleFlasherXPage(AsyncWebServerRequest *request) {
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

  request->send(200, "text/html", html);
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
  if (!adcReady) {
    request->send(503, "text/plain", "ADC_NOT_READY");
    return;
  }

  int adc = 0;
  switch (analogNum) {
    case 13: adc = cachedShuntA0Counts; break;
  
    default: adc = 10000; break;
  }


  
  const float output = adcCountsToMilliamps(adc);
  
  char buf[16];
  snprintf(buf, sizeof(buf), "%.5f", output);
  request->send(200, "text/plain", buf);
}

static void handleAnalog13(AsyncWebServerRequest *r) { handleAnalogGeneric(r, 13); }

static void handleAdcStatus(AsyncWebServerRequest *request) {
  const int raw = cachedShuntA0Counts;
  const float volts = adcCountsToVolts(raw);
  const float currentMa = adcCountsToMilliamps(raw);
  char json[256];
  snprintf(json, sizeof(json),
           "{\"ready\":%s,\"fresh\":%s,\"interface\":\"SPI\",\"cs_pin\":%d,\"adc1_reg\":\"0x%02X\",\"raw\":%d,\"volts\":%.6f,\"current_ma\":%.5f,\"shunt_ohms\":%.1f,\"vref\":%.3f,\"int_status\":%u}",
           adcReady ? "true" : "false", cachedAdcFresh ? "true" : "false", ADC_CS_PIN, REG_ADC1, raw, volts, currentMa, SHUNT_RESISTOR_OHMS, ADC_REF_VOLTS, cachedInterruptStatus);
  request->send(200, "application/json", json);
}

// -----------------------------------------------------------------------------
// networking init / loop
// -----------------------------------------------------------------------------

void networking_init() {
  adcReady = initMax22530Spi();
  lastAdcPollMillis = millis();
  if (adcReady) {
    Serial.printf("MAX22530 SPI ADC initialized (CS=%d, ADC1 reg=0x%02X)\n", ADC_CS_PIN, REG_ADC1);
  } else {
    Serial.println("MAX22530 SPI ADC init failed");
  }

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

  // Routes only need registration once.
  if (!routesRegistered) {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/flasherx", HTTP_GET, handleFlasherXPage);
    server.on("/teensy_ota", HTTP_GET, handleTeensyOtaPage);
    server.on("/update", HTTP_GET, handleTeensyOtaPage);

    server.on("/analog/13", HTTP_GET, handleAnalog13);
    server.on("/adc/status", HTTP_GET, handleAdcStatus);

    server.on("/flasherx/upload", HTTP_POST,
              [](AsyncWebServerRequest *request){
                const char resp[] = "<!DOCTYPE html><html><body><h1>Upload received</h1><p><a href='/'>Return to control panel</a></p></body></html>";
                request->send(200, "text/html", resp);
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

  if (now - lastAnalogSampleMillis >= ANALOG_POLL_INTERVAL_MS) {
    lastAnalogSampleMillis = now;
    int a13 = 0;
    if (adcReady) {
      uint16_t regs[5] = {0, 0, 0, 0, 0};
      max22530BurstRead(REG_ADC1, regs);
      cachedAdcFresh = adcFresh(regs[0]);
      cachedInterruptStatus = regs[4];
      a13 = static_cast<int>(adcCode12(regs[0]));
      lastAdcPollMillis = now;
    }

    a13SampleSum -= a13Samples[a13SampleIndex];
    a13Samples[a13SampleIndex] = a13;
    a13SampleSum += a13;

    a13SampleIndex = (a13SampleIndex + 1) % ANALOG_SMOOTH_WINDOW;
    if (a13SampleCount < ANALOG_SMOOTH_WINDOW) {
      a13SampleCount++;
    }

    if (a13SampleCount > 0) {
      cachedShuntA0Counts = static_cast<int>(a13SampleSum / a13SampleCount);
    }
  }

  if (adcReady && (now - lastAdcPollMillis >= ADC_POLL_WATCHDOG_MS)) {
    restartNetworkingService("ADC poll timeout > 60s");
  }

  if (adcReady && (now - lastAnalogPrintMillis >= ANALOG_PRINT_INTERVAL_MS)) {
    lastAnalogPrintMillis = now;
    const int raw = cachedShuntA0Counts;
    const float volts = adcCountsToVolts(raw);
    const float currentMa = adcCountsToMilliamps(raw);
    Serial.printf("ADC raw=%d voltage=%.6f V current=%.5f mA\n", raw, volts, currentMa);

    if (raw == lastRawPrintValue) {
      if (sameRawPrintCount < 255) {
        sameRawPrintCount++;
      }
    } else {
      lastRawPrintValue = raw;
      sameRawPrintCount = 0;
    }

    if (sameRawPrintCount == 20 && (raw == 0 || raw == 65535)) {
      Serial.println("ADC warning: value is stuck at rail. Check CS pin, SPI wiring, and ADC register/config.");
    }
  }

  if (pendingUpdate) {
    pendingUpdate = false;
    Serial.println("Applying update now (networking)");
    ota.applyUpdate();
  }
}
