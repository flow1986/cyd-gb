#include <Arduino.h>
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <WebServer.h>
#include <WiFi.h>
#include <time.h>

#include "display.h"
#include "hw_config.h"
#include "touch_input.h"

namespace {

constexpr uint8_t kStationCount = 5;
constexpr uint32_t kScanIntervalMs = 5000;
constexpr uint32_t kSampleIntervalMs = 60000;
constexpr uint8_t kOfflineAfterMisses = 3;
constexpr uint8_t kMaxLogFiles = 10;
constexpr uint8_t kVisibleLogLines = 7;
constexpr const char* kLogDirectory = "/logs";
constexpr const char* kLogPath = "/logs/btlogger.csv";
constexpr const char* kBerlinTz = "CET-1CEST,M3.5.0/2,M10.5.0/3";

struct BeaconStation {
  char name[24];
  char mac[18];
  int rssi;
  uint8_t missedScans;
  bool online;
};

struct LogFileEntry {
  char path[64];
  char name[32];
};

SPIClass sdSpi(VSPI);
BLEScan* bleScan = nullptr;
Preferences preferences;
BeaconStation stations[kStationCount];
WebServer server(80);
uint32_t lastScanMs = 0;
uint32_t lastSampleMs = 0;
bool timeSynced = false;
bool webServerStarted = false;

uint8_t readLogFiles(LogFileEntry* files);

void maybeRunTouchCalibration() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("BT Logger", SCREEN_W / 2, 100, 4);
  tft.drawString("Touch halten fuer", SCREEN_W / 2, 144, 2);
  tft.drawString("Kalibrierung", SCREEN_W / 2, 168, 2);
  tft.setTextColor(0x7BEF, TFT_BLACK);
  tft.drawString("Startet gleich...", SCREEN_W / 2, 220, 2);

  uint32_t startMs = millis();
  uint32_t holdSince = 0;
  bool calibrationRequested = false;

  while (millis() - startMs < 1400) {
    touch_update();
    if (touch_is_pressed()) {
      if (holdSince == 0) holdSince = millis();
      if (millis() - holdSince >= 900) {
        calibrationRequested = true;
        break;
      }
    } else {
      holdSince = 0;
    }
    delay(10);
  }

  if (calibrationRequested) {
    while (touch_is_pressed()) {
      touch_update();
      delay(10);
    }
    touch_run_calibration();
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("Kalibrierung gespeichert", SCREEN_W / 2, 150, 2);
    delay(800);
  }
}

void setDefaultStations() {
  for (uint8_t index = 0; index < kStationCount; ++index) {
    snprintf(stations[index].name, sizeof(stations[index].name), "Station %u", index + 1);
    snprintf(stations[index].mac, sizeof(stations[index].mac), "AA:BB:CC:DD:EE:%02u", index + 1);
    stations[index].rssi = -127;
    stations[index].missedScans = 0;
    stations[index].online = false;
  }
}

bool hasSavedStations() {
  preferences.begin("bt", true);
  bool valid = preferences.getBool("valid", false);
  preferences.end();
  return valid;
}

void loadStations() {
  setDefaultStations();
  preferences.begin("bt", true);
  if (preferences.getBool("valid", false)) {
    for (uint8_t index = 0; index < kStationCount; ++index) {
      String nameKey = "n" + String(index);
      String macKey = "m" + String(index);
      String name = preferences.getString(nameKey.c_str(), stations[index].name);
      String mac = preferences.getString(macKey.c_str(), stations[index].mac);
      name.trim();
      mac.trim();
      name.toCharArray(stations[index].name, sizeof(stations[index].name));
      mac.toUpperCase();
      mac.toCharArray(stations[index].mac, sizeof(stations[index].mac));
    }
  }
  preferences.end();
}

bool hasSavedWifi() {
  preferences.begin("btwifi", true);
  bool valid = preferences.getString("ssid", "").length() > 0;
  preferences.end();
  return valid;
}

bool initSd() {
  sdSpi.begin(SD_PIN_SCK, SD_PIN_MISO, SD_PIN_MOSI, SD_PIN_CS);
  if (!SD.begin(SD_PIN_CS, sdSpi, 20000000)) return false;
  if (!SD.exists(kLogDirectory) && !SD.mkdir(kLogDirectory)) return false;
  if (!SD.exists(kLogPath)) {
    File logFile = SD.open(kLogPath, FILE_WRITE);
    if (!logFile) return false;
    logFile.println("timestamp_berlin,epoch_utc,uptime_ms,event,station,mac,rssi");
    logFile.close();
  }
  return true;
}

void drawCentered(const char* title, const char* detail = "") {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(title, SCREEN_W / 2, 120, 4);
  tft.setTextColor(0x7BEF, TFT_BLACK);
  tft.drawString(detail, SCREEN_W / 2, 160, 2);
}

bool waitForTap(int16_t* tapX, int16_t* tapY) {
  bool pressed = false;
  while (true) {
    touch_update();
    if (touch_is_pressed()) {
      pressed = true;
      *tapX = touch_get_x();
      *tapY = touch_get_y();
    } else if (pressed) {
      return true;
    }
    delay(15);
  }
}

void drawKeyboard(const char* title, const String& value, bool hidden, uint8_t layout) {
  const char* rows[3];
  if (layout == 1) {
    rows[0] = "qwertzuiop";
    rows[1] = "asdfghjkl";
    rows[2] = "yxcvbnm";
  } else if (layout == 2) {
    rows[0] = "1234567890";
    rows[1] = "!@#$%^&*-_";
    rows[2] = ".?/+:;,()=";
  } else {
    rows[0] = "QWERTZUIOP";
    rows[1] = "ASDFGHJKL";
    rows[2] = "YXCVBNM";
  }

  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(title, SCREEN_W / 2, 16, 2);
  tft.setTextDatum(TL_DATUM);
  tft.fillRect(8, 34, SCREEN_W - 16, 32, 0x2104);
  String shown = value;
  if (hidden) {
    shown = "";
    for (uint8_t index = 0; index < value.length(); ++index) shown += '*';
  }
  tft.setTextColor(TFT_WHITE, 0x2104);
  tft.drawString(shown, 12, 43, 2);

  for (uint8_t row = 0; row < 3; ++row) {
    uint8_t count = strlen(rows[row]);
    int width = (SCREEN_W - 16) / count;
    for (uint8_t column = 0; column < count; ++column) {
      int x = 8 + column * width;
      int y = 78 + row * 48;
      tft.drawRect(x, y, width - 2, 42, 0x7BEF);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setTextDatum(MC_DATUM);
      char key[2] = {rows[row][column], 0};
      tft.drawString(key, x + (width - 2) / 2, y + 21, 2);
    }
  }

  const char* action0Label = (layout == 0) ? "abc" : ((layout == 1) ? "123" : "ABC");
  const char* actions[] = {action0Label, "SP", "<", "OK"};
  for (uint8_t index = 0; index < 4; ++index) {
    int x = 8 + index * 58;
    tft.drawRect(x, 230, 54, 42, index == 3 ? TFT_GREEN : 0x7BEF);
    tft.setTextColor(index == 3 ? TFT_GREEN : TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(actions[index], x + 27, 251, 2);
  }
}

String editField(const char* title, String value, uint8_t maxLength, bool hidden = false) {
  uint8_t layout = 0;
  while (true) {
    drawKeyboard(title, value, hidden, layout);
    int16_t tapX = 0;
    int16_t tapY = 0;
    waitForTap(&tapX, &tapY);
    if (tapY >= 230) {
      uint8_t action = constrain((tapX - 8) / 58, 0, 3);
      if (action == 0) layout = (layout + 1) % 3;
      if (action == 1 && value.length() < maxLength) value += ' ';
      if (action == 2 && value.length()) value.remove(value.length() - 1);
      if (action == 3 && value.length()) return value;
      continue;
    }
    if (tapY < 78 || tapY >= 222) continue;
    const char* rows[3];
    if (layout == 1) {
      rows[0] = "qwertzuiop";
      rows[1] = "asdfghjkl";
      rows[2] = "yxcvbnm";
    } else if (layout == 2) {
      rows[0] = "1234567890";
      rows[1] = "!@#$%^&*-_";
      rows[2] = ".?/+:;,()=";
    } else {
      rows[0] = "QWERTZUIOP";
      rows[1] = "ASDFGHJKL";
      rows[2] = "YXCVBNM";
    }
    uint8_t row = (tapY - 78) / 48;
    uint8_t count = strlen(rows[row]);
    int width = (SCREEN_W - 16) / count;
    uint8_t column = constrain((tapX - 8) / width, 0, count - 1);
    if (value.length() < maxLength) value += rows[row][column];
  }
}

String selectWifiNetwork(const String& currentSsid) {
  while (true) {
    drawCentered("WLAN Suche...", "Bitte warten");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    int count = WiFi.scanNetworks();

    bool rescanning = false;
    while (!rescanning) {
      tft.fillScreen(TFT_BLACK);
      tft.setTextDatum(MC_DATUM);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString("WLAN waehlen", SCREEN_W / 2, 16, 2);

      tft.drawRect(8, 34, 108, 30, 0x7BEF);
      tft.drawString("+ Manuell", 62, 49, 1);
      tft.drawRect(124, 34, 108, 30, 0x7BEF);
      tft.drawString("Neu suchen", 178, 49, 1);

      int displayCount = (count > 0) ? min(count, 6) : 0;
      if (displayCount == 0) {
        tft.setTextColor(0x7BEF, TFT_BLACK);
        tft.drawString("Keine WLANs gefunden", SCREEN_W / 2, 140, 2);
      } else {
        for (int i = 0; i < displayCount; ++i) {
          int y = 72 + i * 34;
          String ssidName = WiFi.SSID(i);
          int32_t rssi = WiFi.RSSI(i);
          bool isEncrypted = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);

          tft.drawRect(8, y, SCREEN_W - 16, 30, 0x7BEF);
          tft.setTextDatum(TL_DATUM);
          tft.setTextColor(TFT_WHITE, TFT_BLACK);
          String truncSsid = ssidName;
          if (truncSsid.length() > 18) truncSsid = truncSsid.substring(0, 18);
          tft.drawString(truncSsid, 14, y + 8, 1);

          tft.setTextDatum(TR_DATUM);
          tft.setTextColor(0x7BEF, TFT_BLACK);
          String info = String(rssi) + "dBm " + (isEncrypted ? "*" : "");
          tft.drawString(info, SCREEN_W - 14, y + 8, 1);
        }
      }

      int16_t tapX = 0;
      int16_t tapY = 0;
      waitForTap(&tapX, &tapY);

      if (tapY >= 34 && tapY <= 64) {
        if (tapX >= 8 && tapX <= 116) {
          return editField("WLAN Name", currentSsid, 32);
        }
        if (tapX >= 124 && tapX <= 232) {
          rescanning = true;
          break;
        }
      }

      if (displayCount > 0 && tapY >= 72 && tapY < 72 + displayCount * 34) {
        int index = (tapY - 72) / 34;
        if (index >= 0 && index < displayCount) {
          return WiFi.SSID(index);
        }
      }
    }
  }
}

void saveSetup(const String& ssid, const String& password) {
  preferences.begin("btwifi", false);
  preferences.putString("ssid", ssid);
  preferences.putString("pass", password);
  preferences.end();
  preferences.begin("bt", false);
  preferences.putBool("valid", true);
  for (uint8_t index = 0; index < kStationCount; ++index) {
    String nameKey = "n" + String(index);
    String macKey = "m" + String(index);
    preferences.putString(nameKey.c_str(), stations[index].name);
    preferences.putString(macKey.c_str(), stations[index].mac);
  }
  preferences.end();
}

void runSetupWizard() {
  preferences.begin("btwifi", true);
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("pass", "");
  preferences.end();
  ssid = selectWifiNetwork(ssid);
  password = editField("WLAN Passwort", password, 63, true);
  for (uint8_t index = 0; index < kStationCount; ++index) {
    char title[24];
    snprintf(title, sizeof(title), "Station %u Name", index + 1);
    String name = editField(title, stations[index].name, sizeof(stations[index].name) - 1);
    name.toCharArray(stations[index].name, sizeof(stations[index].name));
    snprintf(title, sizeof(title), "Station %u MAC", index + 1);
    String mac = editField(title, stations[index].mac, 17);
    mac.toUpperCase();
    mac.toCharArray(stations[index].mac, sizeof(stations[index].mac));
  }
  saveSetup(ssid, password);
  drawCentered("Gespeichert", "Verbinde mit WLAN...");
  delay(800);
}

void timestamp(char* output, size_t outputSize, time_t* epoch) {
  struct tm timeInfo {};
  if (getLocalTime(&timeInfo, 10)) {
    strftime(output, outputSize, "%Y-%m-%dT%H:%M:%S%z", &timeInfo);
    *epoch = mktime(&timeInfo);
    timeSynced = true;
    return;
  }
  strncpy(output, "UNSYNCED", outputSize);
  output[outputSize - 1] = 0;
  *epoch = 0;
}

void logEvent(const char* event, const BeaconStation& station) {
  char timestampText[32];
  time_t epoch = 0;
  timestamp(timestampText, sizeof(timestampText), &epoch);
  File logFile = SD.open(kLogPath, FILE_APPEND);
  if (!logFile) return;
  logFile.printf("%s,%lld,%lu,%s,%s,%s,%d\n", timestampText,
                 static_cast<long long>(epoch), millis(), event, station.name,
                 station.mac, station.rssi);
  logFile.close();
}

bool connectWifiAndSyncTime() {
  preferences.begin("btwifi", true);
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("pass", "");
  preferences.end();
  if (!ssid.length()) return false;

  drawCentered("WLAN Verbinden", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());
  uint32_t deadline = millis() + 12000;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline) delay(250);

  if (WiFi.status() != WL_CONNECTED) {
    drawCentered("Verbindung fehlgeschlagen", "WLAN-Scan startet...");
    delay(1000);
    String newSsid = selectWifiNetwork(ssid);
    String newPass = editField("WLAN Passwort", password, 63, true);

    preferences.begin("btwifi", false);
    preferences.putString("ssid", newSsid);
    preferences.putString("pass", newPass);
    preferences.end();

    drawCentered("WLAN Verbinden", newSsid.c_str());
    WiFi.begin(newSsid.c_str(), newPass.c_str());
    deadline = millis() + 15000;
    while (WiFi.status() != WL_CONNECTED && millis() < deadline) delay(250);
    if (WiFi.status() != WL_CONNECTED) return false;
  }

  configTzTime(kBerlinTz, "pool.ntp.org", "time.cloudflare.com");
  char timestampText[32];
  time_t epoch = 0;
  for (uint8_t attempt = 0; attempt < 20; ++attempt) {
    timestamp(timestampText, sizeof(timestampText), &epoch);
    if (timeSynced) {
      String ipMsg = "IP: " + WiFi.localIP().toString();
      drawCentered("WLAN Verbunden!", ipMsg.c_str());
      delay(1200);
      return true;
    }
    delay(250);
  }
  return false;
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>BT Logger Dashboard</title>"
                "<style>"
                "body{font-family:sans-serif;background:#121212;color:#eee;margin:0;padding:20px}"
                "h1,h2{color:#4caf50}a{color:#64b5f6;text-decoration:none}"
                "a:hover{text-decoration:underline}"
                ".card{background:#1e1e1e;padding:15px;margin-bottom:15px;border-radius:8px}"
                "table{width:100%;border-collapse:collapse;margin-top:10px}"
                "th,td{border:1px solid #333;padding:8px;text-align:left}"
                "th{background:#2a2a2a}"
                ".online{color:#4caf50;font-weight:bold}"
                ".offline{color:#f44336;font-weight:bold}"
                ".btn{display:inline-block;background:#4caf50;color:#fff;padding:6px 12px;border-radius:4px;margin-right:5px;font-size:14px}"
                "</style></head><body>"
                "<h1>BT Logger Dashboard</h1>"
                "<div class='card'><h2>Status</h2>"
                "<p><b>IP-Adresse:</b> " + WiFi.localIP().toString() + "</p>"
                "<p><b>Zeit-Sync:</b> " + (timeSynced ? "<span class='online'>OK (Berlin)</span>" : "<span class='offline'>Fehlt</span>") + "</p>"
                "<h3>Beacons</h3><table><tr><th>Station</th><th>MAC</th><th>RSSI</th><th>Status</th></tr>";
  for (uint8_t index = 0; index < kStationCount; ++index) {
    html += "<tr><td>" + String(stations[index].name) + "</td><td>" + String(stations[index].mac) + "</td><td>" +
            (stations[index].rssi > -127 ? String(stations[index].rssi) + " dBm" : "--") + "</td><td class='" +
            (stations[index].online ? "online'>Online" : "offline'>Offline") + "</td></tr>";
  }
  html += "</table></div>";

  html += "<div class='card'><h2>SD Logdateien</h2>";
  LogFileEntry files[kMaxLogFiles] {};
  uint8_t count = readLogFiles(files);
  if (count == 0) {
    html += "<p>Keine Logdateien auf SD gefunden.</p>";
  } else {
    html += "<table><tr><th>Dateiname</th><th>Aktionen</th></tr>";
    for (uint8_t index = 0; index < count; ++index) {
      String path = String(files[index].path);
      html += "<tr><td>" + String(files[index].name) + "</td><td>"
              "<a class='btn' href='/view?file=" + path + "'>Ansehen</a>"
              "<a class='btn' href='/download?file=" + path + "'>Herunterladen</a>"
              "</td></tr>";
    }
    html += "</table>";
  }
  html += "</div></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

void handleView() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }
  String path = server.arg("file");
  if (!path.startsWith("/logs/") || path.indexOf("..") >= 0) {
    server.send(403, "text/plain", "Forbidden");
    return;
  }
  File logFile = SD.open(path.c_str(), FILE_READ);
  if (!logFile) {
    server.send(404, "text/plain", "File not found");
    return;
  }

  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>Log View - " + path + "</title>"
                "<style>"
                "body{font-family:monospace;background:#121212;color:#eee;margin:0;padding:20px}"
                "a{color:#64b5f6;text-decoration:none}pre{background:#1e1e1e;padding:15px;border-radius:8px;overflow-x:auto}"
                ".btn{display:inline-block;background:#4caf50;color:#fff;padding:8px 16px;border-radius:4px;margin-bottom:15px}"
                "</style></head><body>"
                "<a class='btn' href='/'>&larr; Zurueck</a> "
                "<a class='btn' href='/download?file=" + path + "'>CSV Herunterladen</a>"
                "<h1>Datei: " + path + "</h1><pre>";

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8", html);

  char buf[256];
  while (logFile.available()) {
    int len = logFile.readBytes(buf, sizeof(buf) - 1);
    buf[len] = 0;
    String chunk = String(buf);
    chunk.replace("&", "&amp;");
    chunk.replace("<", "&lt;");
    chunk.replace(">", "&gt;");
    server.sendContent(chunk);
  }
  logFile.close();
  server.sendContent("</pre></body></html>");
}

void handleDownload() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }
  String path = server.arg("file");
  if (!path.startsWith("/logs/") || path.indexOf("..") >= 0) {
    server.send(403, "text/plain", "Forbidden");
    return;
  }
  File logFile = SD.open(path.c_str(), FILE_READ);
  if (!logFile) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  server.streamFile(logFile, "text/csv");
  logFile.close();
}

void setupWebServer() {
  if (webServerStarted) return;
  server.on("/", handleRoot);
  server.on("/view", handleView);
  server.on("/download", handleDownload);
  server.begin();
  webServerStarted = true;
  Serial.printf("[HTTP] Webserver gestartet: http://%s/\n", WiFi.localIP().toString().c_str());
}

void initBle() {
  BLEDevice::init("btlogger");
  bleScan = BLEDevice::getScan();
  bleScan->setActiveScan(true);
  bleScan->setInterval(100);
  bleScan->setWindow(80);
}

void scanAndLog() {
  int bestRssi[kStationCount];
  for (uint8_t index = 0; index < kStationCount; ++index) bestRssi[index] = -127;
  BLEScanResults results = bleScan->start(1, false);
  for (int deviceIndex = 0; deviceIndex < results.getCount(); ++deviceIndex) {
    BLEAdvertisedDevice device = results.getDevice(deviceIndex);
    String foundMac = device.getAddress().toString().c_str();
    foundMac.toUpperCase();
    for (uint8_t stationIndex = 0; stationIndex < kStationCount; ++stationIndex) {
      if (foundMac == stations[stationIndex].mac) {
        bestRssi[stationIndex] = max(bestRssi[stationIndex], device.getRSSI());
      }
    }
  }
  bleScan->clearResults();
  bool writeSample = millis() - lastSampleMs >= kSampleIntervalMs;
  for (uint8_t index = 0; index < kStationCount; ++index) {
    BeaconStation& station = stations[index];
    bool seen = bestRssi[index] > -127;
    station.rssi = bestRssi[index];
    if (seen) station.missedScans = 0;
    else if (station.missedScans < UINT8_MAX) ++station.missedScans;
    bool wasOnline = station.online;
    station.online = seen || station.missedScans < kOfflineAfterMisses;
    if (station.online != wasOnline) logEvent(station.online ? "online" : "offline", station);
    else if (writeSample) logEvent(seen ? "sample_seen" : "sample_missing", station);
  }
  if (writeSample) lastSampleMs = millis();
  lastScanMs = millis();
}

void drawStatus() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("BT Logger", SCREEN_W / 2, 14, 2);
  tft.setTextColor(timeSynced ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(timeSynced ? "Berlin Zeit OK" : "Zeit nicht synchron", SCREEN_W / 2, 32, 1);
  if (WiFi.status() == WL_CONNECTED) {
    String ipStr = "IP: " + WiFi.localIP().toString();
    tft.setTextColor(0x07FF, TFT_BLACK); // Bright Cyan
    tft.drawString(ipStr, SCREEN_W / 2, 48, 2);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("IP: Offline", SCREEN_W / 2, 48, 1);
  }
  for (uint8_t index = 0; index < kStationCount; ++index) {
    int y = 70 + index * 41;
    tft.fillCircle(14, y, 6, stations[index].online ? TFT_GREEN : TFT_RED);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(stations[index].name, 28, y - 9, 2);
    char rssi[16];
    snprintf(rssi, sizeof(rssi), "%d dBm", stations[index].rssi);
    tft.setTextColor(0x7BEF, TFT_BLACK);
    tft.drawString(rssi, 28, y + 10, 1);
  }
  tft.drawRect(10, 286, 104, 26, 0x7BEF);
  tft.drawRect(126, 286, 104, 26, 0x7BEF);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("LOGS", 62, 299, 2);
  tft.drawString("SETUP", 178, 299, 2);
}

bool bottomButtonTapped(int16_t left, int16_t right) {
  touch_update();
  if (!touch_is_pressed()) return false;
  int16_t x = touch_get_x();
  int16_t y = touch_get_y();
  if (x < left || x > right || y < 286 || y > 312) return false;
  while (touch_is_pressed()) {
    touch_update();
    delay(15);
  }
  return true;
}

uint8_t readLogFiles(LogFileEntry* files) {
  File directory = SD.open(kLogDirectory);
  if (!directory || !directory.isDirectory()) return 0;
  uint8_t count = 0;
  File file;
  while ((file = directory.openNextFile()) && count < kMaxLogFiles) {
    if (!file.isDirectory()) {
      String fullPath = file.name();
      String fileName = fullPath.substring(fullPath.lastIndexOf('/') + 1);
      fullPath.toCharArray(files[count].path, sizeof(files[count].path));
      fileName.toCharArray(files[count].name, sizeof(files[count].name));
      ++count;
    }
    file.close();
  }
  directory.close();
  return count;
}

String csvField(const String& line, uint8_t wantedField) {
  int fieldStart = 0;
  for (uint8_t field = 0; field < wantedField; ++field) {
    fieldStart = line.indexOf(',', fieldStart);
    if (fieldStart < 0) return "";
    ++fieldStart;
  }
  int fieldEnd = line.indexOf(',', fieldStart);
  return fieldEnd < 0 ? line.substring(fieldStart) : line.substring(fieldStart, fieldEnd);
}

void drawLogFile(const char* path, const char* name) {
  String lines[kVisibleLogLines];
  File logFile = SD.open(path, FILE_READ);
  if (logFile) {
    logFile.readStringUntil('\n');
    uint8_t lineIndex = 0;
    while (logFile.available()) {
      String line = logFile.readStringUntil('\n');
      line.trim();
      if (line.length()) {
        lines[lineIndex] = line;
        lineIndex = (lineIndex + 1) % kVisibleLogLines;
      }
    }
    logFile.close();
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(name, SCREEN_W / 2, 14, 2);
    for (uint8_t index = 0; index < kVisibleLogLines; ++index) {
      const String& line = lines[(lineIndex + index) % kVisibleLogLines];
      if (!line.length()) continue;
      String timestampText = csvField(line, 0);
      timestampText.replace('T', ' ');
      if (timestampText.length() > 16) timestampText.remove(16);
      String event = csvField(line, 3);
      String station = csvField(line, 4);
      String rssi = csvField(line, 6);
      String displayed = timestampText + " " + event + " " + station + " " + rssi;
      tft.setTextDatum(TL_DATUM);
      tft.setTextColor(0x7BEF, TFT_BLACK);
      tft.drawString(displayed, 4, 42 + index * 30, 1);
    }
  }
  tft.drawRect(68, 286, 104, 26, 0x7BEF);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("ZURUECK", SCREEN_W / 2, 299, 1);
}

void showLogFile(const char* path, const char* name) {
  while (true) {
    drawLogFile(path, name);
    int16_t tapX = 0;
    int16_t tapY = 0;
    waitForTap(&tapX, &tapY);
    if (tapY >= 286) return;
  }
}

void showLogBrowser() {
  LogFileEntry files[kMaxLogFiles] {};
  uint8_t count = readLogFiles(files);
  while (true) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("SD Logs", SCREEN_W / 2, 16, 2);
    if (!count) {
      tft.setTextColor(0x7BEF, TFT_BLACK);
      tft.drawString("Keine Logdateien", SCREEN_W / 2, 140, 2);
    }
    for (uint8_t index = 0; index < count; ++index) {
      int y = 40 + index * 24;
      tft.drawRect(8, y, SCREEN_W - 16, 20, 0x7BEF);
      tft.setTextDatum(TL_DATUM);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString(files[index].name, 14, y + 4, 1);
    }
    tft.drawRect(68, 286, 104, 26, 0x7BEF);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("ZURUECK", SCREEN_W / 2, 299, 1);
    int16_t tapX = 0;
    int16_t tapY = 0;
    waitForTap(&tapX, &tapY);
    if (tapY >= 286) return;
    if (tapY >= 40 && tapY < 40 + count * 24) {
      uint8_t index = (tapY - 40) / 24;
      showLogFile(files[index].path, files[index].name);
    }
  }
}

void editWifiConfig() {
  preferences.begin("btwifi", true);
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("pass", "");
  preferences.end();

  ssid = selectWifiNetwork(ssid);
  password = editField("WLAN Passwort", password, 63, true);

  preferences.begin("btwifi", false);
  preferences.putString("ssid", ssid);
  preferences.putString("pass", password);
  preferences.end();
}

void editStationConfig(uint8_t index) {
  char title[32];
  snprintf(title, sizeof(title), "Station %u Name", index + 1);
  String name = editField(title, stations[index].name, sizeof(stations[index].name) - 1);
  name.toCharArray(stations[index].name, sizeof(stations[index].name));

  snprintf(title, sizeof(title), "Station %u MAC", index + 1);
  String mac = editField(title, stations[index].mac, 17);
  mac.toUpperCase();
  mac.toCharArray(stations[index].mac, sizeof(stations[index].mac));

  preferences.begin("bt", false);
  preferences.putBool("valid", true);
  String nameKey = "n" + String(index);
  String macKey = "m" + String(index);
  preferences.putString(nameKey.c_str(), stations[index].name);
  preferences.putString(macKey.c_str(), stations[index].mac);
  preferences.end();
}

void showSetupMenu() {
  while (true) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Einstellungen", SCREEN_W / 2, 16, 2);

    // Button 0: WLAN Config
    tft.drawRect(8, 38, SCREEN_W - 16, 32, 0x7BEF);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("WLAN einstellen", 14, 46, 2);

    // Buttons 1-5: Station 1-5
    for (uint8_t index = 0; index < kStationCount; ++index) {
      int y = 76 + index * 36;
      tft.drawRect(8, y, SCREEN_W - 16, 30, 0x7BEF);
      tft.setTextDatum(TL_DATUM);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString(stations[index].name, 14, y + 7, 2);
      tft.setTextDatum(TR_DATUM);
      tft.setTextColor(0x7BEF, TFT_BLACK);
      tft.drawString(stations[index].mac, SCREEN_W - 14, y + 7, 1);
    }

    tft.drawRect(68, 286, 104, 26, 0x7BEF);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("FERTIG", SCREEN_W / 2, 299, 2);

    int16_t tapX = 0;
    int16_t tapY = 0;
    waitForTap(&tapX, &tapY);

    if (tapY >= 286) return;

    if (tapY >= 38 && tapY < 70) {
      editWifiConfig();
      timeSynced = false;
      WiFi.disconnect();
      if (connectWifiAndSyncTime()) setupWebServer();
    } else if (tapY >= 76 && tapY < 76 + kStationCount * 36) {
      uint8_t index = (tapY - 76) / 36;
      if (index < kStationCount) {
        editStationConfig(index);
      }
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  display_init();
  touch_init();
  maybeRunTouchCalibration();
  drawCentered("BT Logger", "SD wird vorbereitet...");
  if (!initSd()) {
    drawCentered("SD Fehler", "FAT32 Karte einlegen");
    while (true) delay(1000);
  }
  loadStations();
  if (!hasSavedWifi()) {
    editWifiConfig();
  }
  if (connectWifiAndSyncTime()) setupWebServer();
  initBle();
  for (const BeaconStation& station : stations) logEvent("boot", station);
  drawStatus();
}

void loop() {
  if (webServerStarted) {
    server.handleClient();
  }
  if (bottomButtonTapped(126, 230)) {
    showSetupMenu();
    drawStatus();
  }
  if (bottomButtonTapped(10, 114)) {
    showLogBrowser();
    drawStatus();
  }
  if (millis() - lastScanMs >= kScanIntervalMs) {
    scanAndLog();
    drawStatus();
  }
  delay(20);
}