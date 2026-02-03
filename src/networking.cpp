#include <Arduino.h>

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

// OTA callback
static void onUpdateReady() {
  Serial.println("Update is ready - scheduling apply from networking_loop()");
  hw_setBlinkEnabled(true);
  pendingUpdate = true;
}

// Handlers
static void handleRoot(AsyncWebServerRequest *request) {
  const char html[] =
    "<!DOCTYPE html>"
    "<html><head><title>Teensy APP</title></head><body>"
    "<div style=\"position:fixed;right:8px;top:8px;\"><button onclick=\"location.href='/'\">Home</button></div>"
    "<h1>Teensy 4.1 APP</h1>"
    "<p>LED control:</p>"
    "<ul>"
    "<li><a href=\"/led/on\">/led/on</a></li>"
    "<li><a href=\"/led/off\">/led/off</a></li>"
    "</ul>"
    "<p>Relay control:</p>"
    "<ul>"
    "<li><a href=\"/relay/1/on\">Relay 1 ON</a> | "
    "<a href=\"/relay/1/off\">OFF</a></li>"
    "<li><a href=\"/relay/2/on\">Relay 2 ON</a> | "
    "<a href=\"/relay/2/off\">OFF</a></li>"
    "<li><a href=\"/relay/3/on\">Relay 3 ON</a> | "
    "<a href=\"/relay/3/off\">OFF</a></li>"
    "<li><a href=\"/relay/4/on\">Relay 4 ON</a> | "
    "<a href=\"/relay/4/off\">OFF</a></li>"
    "</ul>"
    "<p>Firmware updates:</p>"
    "<button onclick=\"location.href='/flasherx'\">FlasherX OTA</button> "
    "<button onclick=\"location.href='/teensy_ota'\">Teensy OTA</button>"
    "<h3>Analog A0:</h3>"
    "<div id=\"a0val\">--</div>"
    "<script>function pollA0(){fetch('/analog').then(r=>r.text()).then(t=>document.getElementById('a0val').innerText=t);}setInterval(pollA0,1000);pollA0();</script>"
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

// LED control
static void handleLedOn(AsyncWebServerRequest *request) {
  hw_setBlinkEnabled(false);
  hw_setLed(true);
  request->send(200, "text/plain", "LED ON");
}
static void handleLedOff(AsyncWebServerRequest *request) {
  hw_setBlinkEnabled(false);
  hw_setLed(false);
  request->send(200, "text/plain", "LED OFF");
}

// Relay control helpers
static void handleRelay(AsyncWebServerRequest *request, int relay, bool on) {
  hw_setRelay(relay, on);
  char buf[32];
  snprintf(buf, sizeof(buf), "Relay %d %s", relay, on ? "ON" : "OFF");
  request->send(200, "text/plain", buf);
}
static void handleRelay1On(AsyncWebServerRequest *r){ handleRelay(r, 1, true); }
static void handleRelay1Off(AsyncWebServerRequest *r){ handleRelay(r, 1, false); }
static void handleRelay2On(AsyncWebServerRequest *r){ handleRelay(r, 2, true); }
static void handleRelay2Off(AsyncWebServerRequest *r){ handleRelay(r, 2, false); }
static void handleRelay3On(AsyncWebServerRequest *r){ handleRelay(r, 3, true); }
static void handleRelay3Off(AsyncWebServerRequest *r){ handleRelay(r, 3, false); }
static void handleRelay4On(AsyncWebServerRequest *r){ handleRelay(r, 4, true); }
static void handleRelay4Off(AsyncWebServerRequest *r){ handleRelay(r, 4, false); }

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

// Analog read endpoint for A0
static void handleAnalog(AsyncWebServerRequest *request) {
  int val = analogRead(A0);
  String s = String(val);
  request->send(200, "text/plain", s);
}

// -----------------------------------------------------------------------------
// networking init / loop
// -----------------------------------------------------------------------------

void networking_init() {
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

  // Routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/flasherx", HTTP_GET, handleFlasherXPage);
  server.on("/teensy_ota", HTTP_GET, handleTeensyOtaPage);
  server.on("/update", HTTP_GET, handleTeensyOtaPage);
  server.on("/analog", HTTP_GET, handleAnalog);
  server.on("/flasherx/upload", HTTP_POST,
            [](AsyncWebServerRequest *request){
              const char resp[] = "<!DOCTYPE html><html><body><h1>Upload received</h1><p><a href='/'>Return to control panel</a></p></body></html>";
              request->send(200, "text/html", resp);
            },
            handleFlasherXUpload);
  server.on("/led/on",  HTTP_GET, handleLedOn);
  server.on("/led/off", HTTP_GET, handleLedOff);

  server.on("/relay/1/on",  HTTP_GET, handleRelay1On);
  server.on("/relay/1/off", HTTP_GET, handleRelay1Off);
  server.on("/relay/2/on",  HTTP_GET, handleRelay2On);
  server.on("/relay/2/off", HTTP_GET, handleRelay2Off);
  server.on("/relay/3/on",  HTTP_GET, handleRelay3On);
  server.on("/relay/3/off", HTTP_GET, handleRelay3Off);
  server.on("/relay/4/on",  HTTP_GET, handleRelay4On);
  server.on("/relay/4/off", HTTP_GET, handleRelay4Off);

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.print("HTTP APP server started at http://");
  Serial.print(cur[0]); Serial.print(".");
  Serial.print(cur[1]); Serial.print(".");
  Serial.print(cur[2]); Serial.print(".");
  Serial.print(cur[3]); Serial.println("/");
  Serial.println("Use /update for firmware upload.");
}

void networking_loop() {
  if (pendingUpdate) {
    pendingUpdate = false;
    Serial.println("Applying update now (networking)");
    ota.applyUpdate();
  }
}
