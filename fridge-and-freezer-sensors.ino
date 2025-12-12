#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <time.h>

// Sketch version information
const char* SKETCH_NAME    = "fridge-and-freezer-sensors";
const char* SKETCH_FOLDER  = "/Users/ian/Documents/Arduino/fridge-and-freezer-sensors";
const char* SKETCH_VERSION = "1.0.10";  // Fixed MQTT topic publishing

// Serial output verbosity
const bool SERIAL_VERBOSE = false;

const char* ssid     = "Falconeer48_EXT";
const char* password = "0795418296";

const char* mqtt_server = "192.168.50.232";
const char* mqtt_user   = "ian";
const char* mqtt_pass   = "Falcon1959";

// NTP / time (UTC+2)
const char* ntpServer          = "pool.ntp.org";
const long  gmtOffset_sec      = 2 * 3600;
const int   daylightOffset_sec = 0;

WiFiClient espClient;
PubSubClient client(espClient);
WebServer server(80);

#define ONE_WIRE_BUS 18
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Thresholds
const float EXTERNAL_MAX_TEMP = 35.0;
const float EXTERNAL_MIN_TEMP = 0.0;
const float FREEZER_MAX_TEMP  = -12.0;
const float FREEZER_MIN_TEMP  = -25.0;
const float FRIDGE_MAX_TEMP   = 8.0;
const float FRIDGE_MIN_TEMP   = 0.0;

// Alert state
bool externalAlert      = false;
bool freezerAlert       = false;
bool fridgeAlert        = false;
bool sensorFailureAlert = false;

// Sensor mapping - MAC addresses to names
// Sensor #1: 28:89:89:87:00:3C:05:5A → Freezer (Coldest, ~-24°C)
DeviceAddress freezerSensor  = { 0x28, 0x89, 0x89, 0x87, 0x00, 0x3C, 0x05, 0x5A };
// Sensor #2: 28:7D:B0:B0:0F:00:00:4D → External (Warmest, ~23°C)
DeviceAddress externalSensor = { 0x28, 0x7D, 0xB0, 0xB0, 0x0F, 0x00, 0x00, 0x4D };
// Sensor #3: 28:4F:2E:AF:0F:00:00:79 → Fridge (Middle, ~7°C)
DeviceAddress fridgeSensor   = { 0x28, 0x4F, 0x2E, 0xAF, 0x0F, 0x00, 0x00, 0x79 };

// MQTT topics
const char* topic_external     = "home/kitchen/external/temperature";
const char* topic_freezer      = "home/kitchen/freezer/temperature";
const char* topic_fridge       = "home/kitchen/fridge/temperature";
const char* topic_availability = "home/kitchen/sensors/availability";

unsigned long lastRead  = 0;
const unsigned long interval = 60000UL;
bool otaInProgress      = false;

// WiFi reconnection
unsigned long lastWiFiCheck            = 0;
const unsigned long WIFI_CHECK_INTERVAL = 10000UL;
unsigned long lastWiFiReconnectAttempt  = 0;
const unsigned long WIFI_RECONNECT_INTERVAL = 30000UL;
int wifiReconnectAttempts = 0;
const int MAX_WIFI_RECONNECT_ATTEMPTS = 5;

// WiFi uptime tracking
unsigned long wifiConnectedTime      = 0;
unsigned long wifiLastDisconnectTime = 0;
bool wifiWasConnected                = false;

// WiFi dropout logging
struct WiFiDropoutLog {
  unsigned long timestamp;  // epoch seconds
  int statusCode;          // WiFi.status() code
  int rssi;                // RSSI at time of dropout
  unsigned long connectedDuration;  // How long it was connected before dropout (seconds)
  bool reconnected;        // Whether it successfully reconnected
  unsigned long reconnectTime;  // How long reconnection took (seconds)
};

const int MAX_WIFI_LOGS = 20;  // Keep last 20 dropouts
WiFiDropoutLog wifiLogs[MAX_WIFI_LOGS];
int wifiLogIndex = 0;
int wifiLogCount = 0;

// Temperatures
float tExternal = -127;
float tFreezer  = -127;
float tFridge   = -127;

bool externalOk = false;
bool freezerOk  = false;
bool fridgeOk   = false;

// Graph storage – 24h at 3min intervals (480 readings)
const int MAX_READINGS = 480;
const unsigned long GRAPH_INTERVAL = 180000UL;

struct TempReading {
  unsigned long timestamp;
  float tExternal;
  float tFreezer;
  float tFridge;
};

TempReading readings[MAX_READINGS];
int readingIndex   = 0;
int readingCount   = 0;
unsigned long lastGraphSave = 0;

// MQTT discovery for fridge sensors
void publish_mqtt_discovery() {
  // Fridge
  client.publish(
    "homeassistant/sensor/kitchen_fridge_temperature/config",
    "{\"name\":\"Fridge Temperature\",\"state_topic\":\"home/kitchen/fridge/temperature\",\"unit_of_measurement\":\"°C\",\"device_class\":\"temperature\",\"state_class\":\"measurement\",\"unique_id\":\"kitchen_fridge_temperature\",\"device\":{\"identifiers\":[\"kitchen_fridge_freezer\"],\"name\":\"Fridge/Freezer Sensors\",\"manufacturer\":\"Ian\",\"model\":\"ESP32 DS18B20\"}}",
    true
  );

  // Freezer
  client.publish(
    "homeassistant/sensor/kitchen_freezer_temperature/config",
    "{\"name\":\"Freezer Temperature\",\"state_topic\":\"home/kitchen/freezer/temperature\",\"unit_of_measurement\":\"°C\",\"device_class\":\"temperature\",\"state_class\":\"measurement\",\"unique_id\":\"kitchen_freezer_temperature\",\"device\":{\"identifiers\":[\"kitchen_fridge_freezer\"],\"name\":\"Fridge/Freezer Sensors\",\"manufacturer\":\"Ian\",\"model\":\"ESP32 DS18B20\"}}",
    true
  );

  // External
  client.publish(
    "homeassistant/sensor/kitchen_external_temperature/config",
    "{\"name\":\"External Temperature\",\"state_topic\":\"home/kitchen/external/temperature\",\"unit_of_measurement\":\"°C\",\"device_class\":\"temperature\",\"state_class\":\"measurement\",\"unique_id\":\"kitchen_external_temperature\",\"device\":{\"identifiers\":[\"kitchen_fridge_freezer\"],\"name\":\"Fridge/Freezer Sensors\",\"manufacturer\":\"Ian\",\"model\":\"ESP32 DS18B20\"}}",
    true
  );

  // Availability as binary_sensor
  client.publish(
    "homeassistant/binary_sensor/kitchen_sensors_availability/config",
    "{\"name\":\"Fridge Sensors Availability\",\"state_topic\":\"home/kitchen/sensors/availability\",\"device_class\":\"connectivity\",\"payload_on\":\"online\",\"payload_off\":\"offline\",\"unique_id\":\"kitchen_sensors_availability\",\"device\":{\"identifiers\":[\"kitchen_fridge_freezer\"],\"name\":\"Fridge/Freezer Sensors\",\"manufacturer\":\"Ian\",\"model\":\"ESP32 DS18B20\"}}",
    true
  );
}

void reconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32-fridge", mqtt_user, mqtt_pass, topic_availability, 0, true, "offline")) {
      Serial.println("connected!");
      client.publish(topic_availability, "online", true);
      publish_mqtt_discovery();
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
    }
  }
}

void checkWiFiConnection() {
  unsigned long now = millis();

  if (now - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = now;

    if (WiFi.status() != WL_CONNECTED) {
      // Track when WiFi disconnected and log the event
      if (wifiWasConnected && wifiConnectedTime > 0) {
        wifiLastDisconnectTime = millis();
        unsigned long connectedDuration = (wifiLastDisconnectTime - wifiConnectedTime) / 1000;  // seconds
        
        // Log the dropout event
        unsigned long nowEpoch = currentEpoch();
        wifiLogs[wifiLogIndex].timestamp = nowEpoch;
        wifiLogs[wifiLogIndex].statusCode = WiFi.status();
        wifiLogs[wifiLogIndex].rssi = WiFi.RSSI();
        wifiLogs[wifiLogIndex].connectedDuration = connectedDuration;
        wifiLogs[wifiLogIndex].reconnected = false;  // Will be updated if reconnection succeeds
        wifiLogs[wifiLogIndex].reconnectTime = 0;
        
        wifiLogIndex = (wifiLogIndex + 1) % MAX_WIFI_LOGS;
        if (wifiLogCount < MAX_WIFI_LOGS) wifiLogCount++;
        
        Serial.println("=== WiFi DROPOUT LOGGED ===");
        Serial.print("Time: ");
        Serial.println(getFormattedTime());
        Serial.print("Status Code: ");
        Serial.println(WiFi.status());
        Serial.print("RSSI: ");
        Serial.println(WiFi.RSSI());
        Serial.print("Was connected for: ");
        Serial.print(connectedDuration);
        Serial.println(" seconds");
        Serial.println("===========================");
        
        wifiWasConnected = false;
      }

      Serial.print("WiFi disconnected! Status: ");
      Serial.println(WiFi.status());
      Serial.print("RSSI: ");
      Serial.println(WiFi.RSSI());

      // Check if WiFi reconnected (non-blocking check)
      if (WiFi.status() == WL_CONNECTED) {
        // WiFi reconnected successfully
        unsigned long reconnectDuration = (millis() - wifiLastDisconnectTime) / 1000;  // seconds
        
        // Update the most recent log entry with reconnection info
        int lastLogIdx = (wifiLogIndex - 1 + MAX_WIFI_LOGS) % MAX_WIFI_LOGS;
        if (wifiLogCount > 0) {
          wifiLogs[lastLogIdx].reconnected = true;
          wifiLogs[lastLogIdx].reconnectTime = reconnectDuration;
        }
        
        Serial.println("=== WiFi RECONNECTED ===");
        Serial.print("Time: ");
        Serial.println(getFormattedTime());
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("RSSI: ");
        Serial.println(WiFi.RSSI());
        Serial.print("Reconnection took: ");
        Serial.print(reconnectDuration);
        Serial.println(" seconds");
        Serial.println("========================");
        
        wifiReconnectAttempts = 0;
        wifiConnectedTime = millis();
        wifiWasConnected  = true;

        if (!client.connected()) {
          reconnect();
        }
      } else if (now - lastWiFiReconnectAttempt >= WIFI_RECONNECT_INTERVAL) {
        // Time to attempt reconnection
        lastWiFiReconnectAttempt = now;
        wifiReconnectAttempts++;

        Serial.print("Attempting WiFi reconnect (attempt ");
        Serial.print(wifiReconnectAttempts);
        Serial.println(")...");

        WiFi.disconnect();
        delay(100);
        WiFi.mode(WIFI_STA);
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
        WiFi.begin(ssid, password);

        // Non-blocking reconnection attempt - check status without blocking delays
        // The reconnection will be checked on the next WiFi check interval
        Serial.println("WiFi reconnection initiated (non-blocking)");
        
        if (wifiReconnectAttempts >= MAX_WIFI_RECONNECT_ATTEMPTS) {
          Serial.println("Max reconnect attempts reached. Will try again later.");
          wifiReconnectAttempts = 0;
        }
      }
    } else {
      if (wifiReconnectAttempts > 0) {
        wifiReconnectAttempts = 0;
      }

      if (!wifiWasConnected || wifiConnectedTime == 0) {
        wifiConnectedTime = millis();
        wifiWasConnected  = true;
      }

      static unsigned long lastWiFiStatusLog = 0;
      if (now - lastWiFiStatusLog >= 300000UL) {
        lastWiFiStatusLog = now;
        Serial.print("WiFi OK - IP: ");
        Serial.print(WiFi.localIP());
        Serial.print(", RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
      }
    }
  }
}

void setupWiFi() {
  Serial.println("Initializing WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected! IP: ");
    Serial.println(WiFi.localIP());

    wifiConnectedTime = millis();
    wifiWasConnected  = true;

    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());
  } else {
    Serial.println("WiFi connection failed!");
    Serial.print("Status code: ");
    Serial.println(WiFi.status());
  }
}

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(":");
  }
}

void setupTime() {
  if (WiFi.status() == WL_CONNECTED) {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time from NTP");
    } else {
      Serial.println("Time synchronised via NTP");
    }
  }
}

unsigned long currentEpoch() {
  time_t nowEpoch = time(nullptr);
  if (nowEpoch < 100000) {
    return millis() / 1000;
  }
  return (unsigned long)nowEpoch;
}

String getFormattedTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "Time not set";
  }

  char timeString[64];
  strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(timeString);
}

String getWiFiUptime() {
  if (WiFi.status() == WL_CONNECTED && wifiConnectedTime > 0) {
    unsigned long uptimeMs = millis() - wifiConnectedTime;
    unsigned long seconds  = uptimeMs / 1000;
    unsigned long minutes  = seconds / 60;
    unsigned long hours    = minutes / 60;
    unsigned long days     = hours / 24;

    seconds = seconds % 60;
    minutes = minutes % 60;
    hours   = hours % 24;

    String uptime = "";
    if (days > 0)       uptime += String(days) + "d ";
    if (hours > 0 || days > 0)   uptime += String(hours) + "h ";
    if (minutes > 0 || hours > 0 || days > 0) uptime += String(minutes) + "m ";
    uptime += String(seconds) + "s";
    return uptime;
  } else if (wifiLastDisconnectTime > 0 && wifiConnectedTime > 0) {
    unsigned long connectedDuration = wifiLastDisconnectTime - wifiConnectedTime;
    unsigned long seconds  = connectedDuration / 1000;
    unsigned long minutes  = seconds / 60;
    unsigned long hours    = minutes / 60;
    unsigned long days     = hours / 24;

    seconds = seconds % 60;
    minutes = minutes % 60;
    hours   = hours % 24;

    String duration = "Was connected for: ";
    if (days > 0)       duration += String(days) + "d ";
    if (hours > 0 || days > 0)   duration += String(hours) + "h ";
    if (minutes > 0 || hours > 0 || days > 0) duration += String(minutes) + "m ";
    duration += String(seconds) + "s";
    return duration;
  } else {
    return "Not connected";
  }
}

void checkAlerts() {
  // External (old fridge sensor, but using FRIDGE thresholds)
  if (externalOk) {
    if (tExternal > FRIDGE_MAX_TEMP || tExternal < FRIDGE_MIN_TEMP) {
      externalAlert = true;
    } else {
      externalAlert = false;
    }
  } else {
    externalAlert = false;
  }

  // Freezer
  if (freezerOk) {
    if (tFreezer > FREEZER_MAX_TEMP || tFreezer < FREEZER_MIN_TEMP) {
      freezerAlert = true;
    } else {
      freezerAlert = false;
    }
  } else {
    freezerAlert = false;
  }

  // Fridge (old external sensor, EXTERNAL thresholds)
  if (fridgeOk) {
    if (tFridge > EXTERNAL_MAX_TEMP || tFridge < EXTERNAL_MIN_TEMP) {
      fridgeAlert = true;
    } else {
      fridgeAlert = false;
    }
  } else {
    fridgeAlert = false;
  }

  sensorFailureAlert = !fridgeOk || !freezerOk || !externalOk;
}

void scanSensors() {
  Serial.println("\n=== SENSOR DIAGNOSTICS ===");
  Serial.print("Total sensors found: ");
  int deviceCount = sensors.getDeviceCount();
  Serial.println(deviceCount);

  if (deviceCount == 0) {
    Serial.println("ERROR: No sensors detected!");
    return;
  }

  Serial.println("\nScanning for all connected sensors:");
  DeviceAddress tempDeviceAddress;
  for (int i = 0; i < deviceCount; i++) {
    if (sensors.getAddress(tempDeviceAddress, i)) {
      Serial.print("  Sensor ");
      Serial.print(i + 1);
      Serial.print(": ");
      printAddress(tempDeviceAddress);

      bool isExternal = true;
      bool isFreezer  = true;
      bool isFridge   = true;
      for (int j = 0; j < 8; j++) {
        if (tempDeviceAddress[j] != externalSensor[j]) isExternal = false;
        if (tempDeviceAddress[j] != freezerSensor[j])  isFreezer  = false;
        if (tempDeviceAddress[j] != fridgeSensor[j])   isFridge   = false;
      }

      if (isExternal) Serial.print(" <-- EXTERNAL");
      else if (isFreezer) Serial.print(" <-- FREEZER");
      else if (isFridge)  Serial.print(" <-- FRIDGE");
      else Serial.print(" <-- UNKNOWN SENSOR");

      Serial.println();
    }
  }

  Serial.println("\nCurrent sensor readings:");
  Serial.print("  External:  ");
  if (externalOk) {
    Serial.print(tExternal, 2);
    Serial.println(" °C");
  } else {
    Serial.print("FAILED (value: ");
    Serial.print(tExternal, 2);
    Serial.println(")");
  }

  Serial.print("  Freezer:   ");
  if (freezerOk) {
    Serial.print(tFreezer, 2);
    Serial.println(" °C");
  } else {
    Serial.print("FAILED (value: ");
    Serial.print(tFreezer, 2);
    Serial.println(")");
  }

  Serial.print("  Fridge:    ");
  if (fridgeOk) {
    Serial.print(tFridge, 2);
    Serial.println(" °C");
  } else {
    Serial.print("FAILED (value: ");
    Serial.print(tFridge, 2);
    Serial.println(")");
  }
  Serial.println("========================");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nESP32 Fridge/Freezer Sensor System Starting...");

  delay(500);

  setupWiFi();
  setupTime();

  ArduinoOTA.setHostname("fridge-freezer");
  ArduinoOTA.onStart([]() { otaInProgress = true; });
  ArduinoOTA.onEnd([]()   { otaInProgress = false; });
  ArduinoOTA.begin();

  client.setServer(mqtt_server, 1883);

  if (WiFi.status() == WL_CONNECTED) {
    if (!MDNS.begin("fridge-freezer")) {
      // ignore
    }
  }

  // Initialize readings array
  for (int i = 0; i < MAX_READINGS; i++) {
    readings[i].timestamp = 0;
    readings[i].tExternal = -127;
    readings[i].tFreezer  = -127;
    readings[i].tFridge   = -127;
  }

  readingIndex  = 0;
  readingCount  = 0;
  lastGraphSave = millis();

  // /data – last 24h, client filters 3h/12h/24h
  server.on("/data", []() {
    unsigned long nowEpoch = currentEpoch();
    unsigned long cutoffTime = 0;
    if (nowEpoch > 100000UL) {
      cutoffTime = nowEpoch - 24UL * 3600UL;
    }

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "");

    String header = "{\"now\":";
    header += String(nowEpoch);
    header += ",\"data\":[";
    server.sendContent(header);

    int startIdx = (readingCount < MAX_READINGS) ? 0 : readingIndex;
    int count    = (readingCount < MAX_READINGS) ? readingCount : MAX_READINGS;
    bool first   = true;
    int dataPoints = 0;

    for (int i = 0; i < count; i++) {
      int idx = (startIdx + i) % MAX_READINGS;
      if (readings[idx].timestamp != 0 &&
          (cutoffTime == 0 || readings[idx].timestamp >= cutoffTime)) {
        if (!first) {
          server.sendContent(",");
        }
        first = false;
        dataPoints++;

        String point = "{\"t\":";
        point += String(readings[idx].timestamp);
        point += ",\"tExternal\":";
        point += String(readings[idx].tExternal, 1);
        point += ",\"tFreezer\":";
        point += String(readings[idx].tFreezer, 1);
        point += ",\"tFridge\":";
        point += String(readings[idx].tFridge, 1);
        point += "}";
        server.sendContent(point);
      }
    }

    // Always include current reading so graph shows immediately
    if (nowEpoch > 100000UL) {
      if (!first) {
        server.sendContent(",");
      }
      String currentPoint = "{\"t\":";
      currentPoint += String(nowEpoch);
      currentPoint += ",\"tExternal\":";
      currentPoint += String(externalOk ? tExternal : -127, 1);
      currentPoint += ",\"tFreezer\":";
      currentPoint += String(freezerOk ? tFreezer : -127, 1);
      currentPoint += ",\"tFridge\":";
      currentPoint += String(fridgeOk ? tFridge : -127, 1);
      currentPoint += "}";
      server.sendContent(currentPoint);
      dataPoints++;
    }

    server.sendContent("]}");
    server.sendContent("");

    if (SERIAL_VERBOSE) {
      Serial.print("Sent ");
      Serial.print(dataPoints);
      Serial.println(" data points to graph");
    }
  });

  // /wifi-log endpoint: WiFi dropout history
  server.on("/wifi-log", []() {
    String json = "{\"logs\":[";
    bool first = true;
    int count = wifiLogCount;
    int startIdx = (count < MAX_WIFI_LOGS) ? 0 : wifiLogIndex;
    
    for (int i = 0; i < count; i++) {
      int idx = (startIdx + i) % MAX_WIFI_LOGS;
      if (!first) json += ",";
      first = false;
      
      json += "{";
      json += "\"timestamp\":" + String(wifiLogs[idx].timestamp) + ",";
      json += "\"statusCode\":" + String(wifiLogs[idx].statusCode) + ",";
      json += "\"rssi\":" + String(wifiLogs[idx].rssi) + ",";
      json += "\"connectedDuration\":" + String(wifiLogs[idx].connectedDuration) + ",";
      json += "\"reconnected\":" + String(wifiLogs[idx].reconnected ? "true" : "false") + ",";
      json += "\"reconnectTime\":" + String(wifiLogs[idx].reconnectTime);
      json += "}";
    }
    
    json += "],\"count\":" + String(count) + "}";
    server.send(200, "application/json", json);
  });

  // /status – live values
  server.on("/status", []() {
    int rssi = WiFi.RSSI();
    bool wifiConnected = (WiFi.status() == WL_CONNECTED);
    String json = "{";
    json += "\"tExternal\":"  + String(tExternal, 1) + ",";
    json += "\"tFreezer\":"   + String(tFreezer, 1)  + ",";
    json += "\"tFridge\":"    + String(tFridge, 1)   + ",";
    json += "\"okExternal\":" + String(externalOk ? "true" : "false") + ",";
    json += "\"okFreezer\":"  + String(freezerOk  ? "true" : "false") + ",";
    json += "\"okFridge\":"   + String(fridgeOk   ? "true" : "false") + ",";
    json += "\"wifiConnected\":" + String(wifiConnected ? "true" : "false") + ",";
    json += "\"rssi\":"       + String(rssi) + ",";
    json += "\"wifiUptime\":\"" + getWiFiUptime() + "\",";
    json += "\"mqtt\":"       + String(client.connected() ? "true" : "false") + ",";
    json += "\"ota\":"        + String(otaInProgress ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  });

  // Main page
  server.on("/", []() {
    String page = "<!DOCTYPE html><html lang='en'><head>";
    page += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    page += "<title>Sensor Status Dashboard</title>";
    page += "<style>";
    page += "*{margin:0;padding:0;box-sizing:border-box;}";
    page += "body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh;padding:20px;}";
    page += ".container{max-width:800px;margin:0 auto;background:rgba(255,255,255,0.95);border-radius:20px;box-shadow:0 10px 40px rgba(0,0,0,0.2);padding:40px;}";
    page += "h1{color:#333;text-align:center;margin-bottom:10px;font-size:2.5em;font-weight:600;}";
    page += ".subtitle{text-align:center;color:#666;margin-bottom:30px;font-size:0.9em;}";
    page += ".sensor-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:20px;margin-bottom:30px;}";
    page += ".sensor-card{background:#f8f9fa;border-radius:15px;padding:25px;box-shadow:0 4px 6px rgba(0,0,0,0.1);transition:transform 0.3s ease;}";
    page += ".sensor-card:hover{transform:translateY(-5px);box-shadow:0 6px 12px rgba(0,0,0,0.15);}";
    page += ".sensor-title{font-size:1.2em;font-weight:600;color:#555;margin-bottom:15px;display:flex;align-items:center;gap:10px;}";
    page += ".status-indicator{width:12px;height:12px;border-radius:50%;display:inline-block;}";
    page += ".status-ok{background:#28a745;box-shadow:0 0 10px rgba(40,167,69,0.5);}";
    page += ".status-error{background:#dc3545;box-shadow:0 0 10px rgba(220,53,69,0.5);}";
    page += ".temperature{font-size:2.5em;font-weight:700;color:#667eea;margin:10px 0;}";
    page += ".unit{font-size:0.6em;color:#999;}";
    page += ".status-text{font-size:0.9em;}";
    page += ".status-text-ok{color:#28a745;}";
    page += ".status-text-error{color:#dc3545;}";
    page += ".info-section{background:#f8f9fa;border-radius:15px;padding:25px;margin-top:20px;}";
    page += ".fade-value{transition:opacity 0.3s ease;}";
    page += ".info-title{font-size:1.3em;font-weight:600;color:#333;margin-bottom:15px;border-bottom:2px solid #667eea;padding-bottom:10px;}";
    page += ".info-row{display:flex;justify-content:space-between;align-items:center;padding:12px 0;border-bottom:1px solid #e0e0e0;}";
    page += ".info-row:last-child{border-bottom:none;}";
    page += ".info-label{font-weight:600;color:#555;}";
    page += ".info-value{color:#333;font-family:'Courier New',monospace;}";
    page += ".badge{display:inline-block;padding:5px 15px;border-radius:20px;font-size:0.85em;font-weight:600;}";
    page += ".badge-connected{background:#d4edda;color:#155724;}";
    page += ".badge-disconnected{background:#f8d7da;color:#721c24;}";
    page += ".badge-ready{background:#d1ecf1;color:#0c5460;}";
    page += ".badge-updating{background:#fff3cd;color:#856404;}";
    page += ".badge-signal-strong{background:#d4edda;color:#155724;}";
    page += ".badge-signal-medium{background:#fff3cd;color:#856404;}";
    page += ".badge-signal-weak{background:#f8d7da;color:#721c24;}";
    page += ".chart-section{background:#f8f9fa;border-radius:15px;padding:25px;margin-top:20px;}";
    page += ".chart-title{font-size:1.3em;font-weight:600;color:#333;margin-bottom:15px;border-bottom:2px solid #667eea;padding-bottom:10px;}";
    page += ".chart-container{position:relative;height:300px;margin-top:20px;}";
    page += ".chart-controls{display:flex;flex-wrap:wrap;gap:10px;margin-top:15px;}";
    page += ".chart-controls button{border:none;border-radius:999px;padding:6px 14px;font-size:0.85em;cursor:pointer;background:#667eea;color:#fff;box-shadow:0 2px 6px rgba(0,0,0,0.15);}";
    page += ".chart-controls button.secondary{background:#e0e3ff;color:#333;}";
    page += ".chart-controls button.active{background:#4338ca;color:#fff;}";
    page += "@media(max-width:600px){.container{padding:20px;}h1{font-size:1.8em;}.temperature{font-size:2em;}.chart-container{height:250px;}}";
    page += "</style>";
    page += "<script src='https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js'></script>";
    page += "<script src='https://cdn.jsdelivr.net/npm/chartjs-adapter-date-fns'></script>";
    page += "<script src='https://cdn.jsdelivr.net/npm/chartjs-plugin-zoom@2.0.1/dist/chartjs-plugin-zoom.umd.min.js'></script>";
    page += "</head><body>";
    page += "<div class='container'>";
    page += "<h1>Fridge Sensor Status Dashboard</h1>";
    page += "<p class='subtitle'>Real-time Temperature Monitoring</p>";
    page += "<div class='sensor-grid'>";

    // Fridge card (shows fridge sensor data)
    page += "<div class='sensor-card'>";
    page += "<div class='sensor-title'><span id='statusFridgeIndicator' class='status-indicator ";
    page += fridgeOk ? "status-ok" : "status-error";
    page += "'></span>Fridge</div>";
    if (fridgeOk) {
      page += "<div class='temperature'><span id='tempFridge' class='fade-value'>";
      page += String(tFridge, 1);
      page += "</span><span class='unit'>°C</span></div>";
      page += "<div id='statusFridgeText' class='status-text status-text-ok'>Sensor Active</div>";
    } else {
      page += "<div class='temperature'><span id='tempFridge' class='fade-value'>—</span><span class='unit'>°C</span></div>";
      page += "<div id='statusFridgeText' class='status-text status-text-error'>Sensor Not Detected</div>";
    }
    page += "</div>";

    // Freezer
    page += "<div class='sensor-card'>";
    page += "<div class='sensor-title'><span id='statusFreezerIndicator' class='status-indicator ";
    page += freezerOk ? "status-ok" : "status-error";
    page += "'></span>Freezer</div>";
    if (freezerOk) {
      page += "<div class='temperature'><span id='tempFreezer' class='fade-value'>";
      page += String(tFreezer, 1);
      page += "</span><span class='unit'>°C</span></div>";
      page += "<div id='statusFreezerText' class='status-text status-text-ok'>Sensor Active</div>";
    } else {
      page += "<div class='temperature'><span id='tempFreezer' class='fade-value'>—</span><span class='unit'>°C</span></div>";
      page += "<div id='statusFreezerText' class='status-text status-text-error'>Sensor Not Detected</div>";
    }
    page += "</div>";

    // External card (shows external sensor data)
    page += "<div class='sensor-card'>";
    page += "<div class='sensor-title'><span id='statusExternalIndicator' class='status-indicator ";
    page += externalOk ? "status-ok" : "status-error";
    page += "'></span>External</div>";
    if (externalOk) {
      page += "<div class='temperature'><span id='tempExternal' class='fade-value'>";
      page += String(tExternal, 1);
      page += "</span><span class='unit'>°C</span></div>";
      page += "<div id='statusExternalText' class='status-text status-text-ok'>Sensor Active</div>";
    } else {
      page += "<div class='temperature'><span id='tempExternal' class='fade-value'>—</span><span class='unit'>°C</span></div>";
      page += "<div id='statusExternalText' class='status-text status-text-error'>Sensor Not Detected</div>";
    }
    page += "</div>";

    page += "</div>"; // sensor-grid

    // System info
    page += "<div class='info-section'>";
    page += "<div class='info-title'>System Information</div>";
    page += "<div class='info-row'><span class='info-label'>Wi-Fi IP Address</span><span class='info-value'>";
    page += WiFi.localIP().toString();
    page += "</span></div>";

    String wifiStatusText = (WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected";
    String wifiStatusClass = (WiFi.status() == WL_CONNECTED) ? "badge-connected" : "badge-disconnected";
    page += "<div class='info-row'><span class='info-label'>Wi-Fi Status</span><span class='info-value'><span id='wifiStatusBadge' class='badge ";
    page += wifiStatusClass;
    page += "'>";
    page += wifiStatusText;
    page += "</span></span></div>";

    page += "<div class='info-row'><span class='info-label'>Wi-Fi Uptime</span><span class='info-value' id='wifiUptimeValue'>";
    page += getWiFiUptime();
    page += "</span></div>";

    int rssi = WiFi.RSSI();
    String signalStrength;
    String signalClass;
    if (rssi > -50) {
      signalStrength = "Strong (" + String(rssi) + " dBm)";
      signalClass = "badge-signal-strong";
    } else if (rssi > -70) {
      signalStrength = "Medium (" + String(rssi) + " dBm)";
      signalClass = "badge-signal-medium";
    } else {
      signalStrength = "Weak (" + String(rssi) + " dBm)";
      signalClass = "badge-signal-weak";
    }
    page += "<div class='info-row'><span class='info-label'>Wi-Fi Signal Strength</span><span class='info-value'><span id='wifiSignalBadge' class='badge ";
    page += signalClass;
    page += "'>";
    page += signalStrength;
    page += "</span></span></div>";

    page += "<div class='info-row'><span class='info-label'>MQTT Status</span><span class='info-value'><span id='mqttStatusBadge' class='badge ";
    page += client.connected() ? "badge-connected" : "badge-disconnected";
    page += "'>";
    page += client.connected() ? "Connected" : "Disconnected";
    page += "</span></span></div>";

    page += "<div class='info-row'><span class='info-label'>OTA Status</span><span class='info-value'><span id='otaStatusBadge' class='badge ";
    page += otaInProgress ? "badge-updating" : "badge-ready";
    page += "'>";
    page += otaInProgress ? "Update in Progress" : "Ready";
    page += "</span></span></div>";

    page += "<div class='info-row'><span class='info-label'>Last Refresh</span><span class='info-value' id='lastRefreshTime'>—</span></div>";
    page += "<div class='info-row'><span class='info-label'>Sketch Name</span><span class='info-value'>";
    page += SKETCH_NAME;
    page += "</span></div>";
    page += "<div class='info-row'><span class='info-label'>Sketch Folder</span><span class='info-value'>";
    page += SKETCH_FOLDER;
    page += "</span></div>";
    page += "<div class='info-row'><span class='info-label'>Version</span><span class='info-value'>";
    page += SKETCH_VERSION;
    page += "</span></div>";
    page += "</div>"; // info-section

    // Chart
    page += "<div class='chart-section'>";
    page += "<div class='chart-title'>Temperature History (Last 3 Hours)</div>";
    page += "<div class='chart-container'><canvas id='tempChart'></canvas></div>";
    page += "<div class='chart-controls'>";
    page += "<button id='range3h' class='active' type='button'>Last 3h</button>";
    page += "<button id='range12h' class='secondary' type='button'>Last 12h</button>";
    page += "<button id='range24h' class='secondary' type='button'>Last 24h</button>";
    page += "<button id='resetZoomBtn' class='secondary' type='button'>Reset zoom</button>";
    page += "</div>";
    page += "</div>";

    page += "</div>"; // container

    // JS
    page += "<script>";
    page += "let chart=null;let currentRange='3h';";
    page += "function updateTextWithFade(id,text){const el=document.getElementById(id);if(!el)return;if(el.textContent===text)return;el.style.opacity=0;setTimeout(()=>{el.textContent=text;el.style.opacity=1;},300);}";
    page += "function updateLastRefreshTime(){const now=new Date();const timeStr=now.toLocaleTimeString([],{hour:'2-digit',minute:'2-digit',second:'2-digit'});const el=document.getElementById('lastRefreshTime');if(el)el.textContent=timeStr;}";
    page += "function setRange(range){currentRange=range;const b3=document.getElementById('range3h');const b12=document.getElementById('range12h');const b24=document.getElementById('range24h');[b3,b12,b24].forEach(b=>{if(b)b.classList.remove('active');});if(range==='3h'&&b3)b3.classList.add('active');if(range==='12h'&&b12)b12.classList.add('active');if(range==='24h'&&b24)b24.classList.add('active');loadData();}";
    page += "function resetZoom(){if(chart&&chart.resetZoom){chart.resetZoom();}}";
    page += "function initChart(){const canvas=document.getElementById('tempChart');if(!canvas){console.error('Canvas element not found');return;}if(typeof Chart==='undefined'){console.error('Chart.js library not loaded');return;}const ctx=canvas.getContext('2d');chart=new Chart(ctx,{type:'line',data:{labels:[],datasets:[{label:'External',data:[],borderColor:'#4facfe',backgroundColor:'rgba(79,172,254,0.1)',tension:0.4,fill:true,pointRadius:0,pointHoverRadius:0},{label:'Freezer',data:[],borderColor:'#f093fb',backgroundColor:'rgba(240,147,251,0.1)',tension:0.4,fill:true,pointRadius:0,pointHoverRadius:0},{label:'Fridge',data:[],borderColor:'#667eea',backgroundColor:'rgba(102,126,234,0.1)',tension:0.4,fill:true,pointRadius:0,pointHoverRadius:0}]},options:{responsive:true,maintainAspectRatio:false,spanGaps:true,plugins:{legend:{position:'top',labels:{font:{size:12}}},zoom:{pan:{enabled:true,mode:'x',modifierKey:null},zoom:{wheel:{enabled:true},pinch:{enabled:true},mode:'x'}}},scales:{x:{type:'time',time:{unit:'hour',tooltipFormat:'MMM d HH:mm'},title:{display:true,text:'Time'}},y:{title:{display:true,text:'Temperature (°C)'}}}}});loadData();}";
    page += "function loadData(){if(!chart){console.log('Chart not initialized');return;}fetch('/data').then(r=>r.text()).then(text=>{if(!text){console.log('Empty /data response');return;}let result;try{result=JSON.parse(text);}catch(e){console.error('JSON parse error for /data:',e,text);return;}if(!result||!result.data){console.log('No data array in response');return;}const labels=[];const externalData=[];const freezerData=[];const fridgeData=[];const nowSec=result.now||Math.floor(Date.now()/1000);let rangeSeconds=3*3600;if(currentRange==='12h')rangeSeconds=12*3600;else if(currentRange==='24h')rangeSeconds=24*3600;let cutoff=nowSec-rangeSeconds;if(nowSec<100000)cutoff=0;if(result.data&&result.data.length>0){result.data.forEach(item=>{if(cutoff&&item.t<cutoff)return;const tsMs=item.t*1000;const externalValid=item.tExternal>-100&&item.tExternal<100;const freezerValid=item.tFreezer>-100&&item.tFreezer<100;const fridgeValid=item.tFridge>-100&&item.tFridge<100;if(externalValid||freezerValid||fridgeValid){labels.push(tsMs);externalData.push(externalValid?parseFloat(item.tExternal):NaN);freezerData.push(freezerValid?parseFloat(item.tFreezer):NaN);fridgeData.push(fridgeValid?parseFloat(item.tFridge):NaN);}});}chart.data.labels=labels;chart.data.datasets[0].data=externalData;chart.data.datasets[1].data=freezerData;chart.data.datasets[2].data=fridgeData;chart.update();}).catch(err=>{console.error('Error loading graph data:',err);});}";
    page += "function loadStatus(){fetch('/status').then(r=>r.json()).then(data=>{updateTextWithFade('tempExternal',data.tExternal.toFixed(1));updateTextWithFade('tempFreezer',data.tFreezer.toFixed(1));updateTextWithFade('tempFridge',data.tFridge.toFixed(1));const externalOk=data.okExternal;const freezerOk=data.okFreezer;const fridgeOk=data.okFridge;const indX=document.getElementById('statusExternalIndicator');const txtX=document.getElementById('statusExternalText');if(indX&&txtX){indX.classList.remove('status-ok','status-error');indX.classList.add(externalOk?'status-ok':'status-error');txtX.classList.remove('status-text-ok','status-text-error');txtX.classList.add(externalOk?'status-text-ok':'status-text-error');txtX.textContent=externalOk?'Sensor Active':'Sensor Not Detected';}const indZ=document.getElementById('statusFreezerIndicator');const txtZ=document.getElementById('statusFreezerText');if(indZ&&txtZ){indZ.classList.remove('status-ok','status-error');indZ.classList.add(freezerOk?'status-ok':'status-error');txtZ.classList.remove('status-text-ok','status-text-error');txtZ.classList.add(freezerOk?'status-text-ok':'status-text-error');txtZ.textContent=freezerOk?'Sensor Active':'Sensor Not Detected';}const indF=document.getElementById('statusFridgeIndicator');const txtF=document.getElementById('statusFridgeText');if(indF&&txtF){indF.classList.remove('status-ok','status-error');indF.classList.add(fridgeOk?'status-ok':'status-error');txtF.classList.remove('status-text-ok','status-text-error');txtF.classList.add(fridgeOk?'status-text-ok':'status-text-error');txtF.textContent=fridgeOk?'Sensor Active':'Sensor Not Detected';}const wifiBadge=document.getElementById('wifiSignalBadge');if(wifiBadge){let signalClass='badge-signal-weak';let signalText='Weak ('+data.rssi+' dBm)';if(data.rssi>-50){signalClass='badge-signal-strong';signalText='Strong ('+data.rssi+' dBm)';}else if(data.rssi>-70){signalClass='badge-signal-medium';signalText='Medium ('+data.rssi+' dBm)';}wifiBadge.classList.remove('badge-signal-strong','badge-signal-medium','badge-signal-weak');wifiBadge.classList.add(signalClass);wifiBadge.textContent=signalText;}const wifiStatusBadge=document.getElementById('wifiStatusBadge');if(wifiStatusBadge){wifiStatusBadge.classList.remove('badge-connected','badge-disconnected');wifiStatusBadge.classList.add(data.wifiConnected?'badge-connected':'badge-disconnected');wifiStatusBadge.textContent=data.wifiConnected?'Connected':'Disconnected';}const wifiUptimeValue=document.getElementById('wifiUptimeValue');if(wifiUptimeValue&&data.wifiUptime){wifiUptimeValue.textContent=data.wifiUptime;}const mqttBadge=document.getElementById('mqttStatusBadge');if(mqttBadge){mqttBadge.classList.remove('badge-connected','badge-disconnected');mqttBadge.classList.add(data.mqtt?'badge-connected':'badge-disconnected');mqttBadge.textContent=data.mqtt?'Connected':'Disconnected';}const otaBadge=document.getElementById('otaStatusBadge');if(otaBadge){otaBadge.classList.remove('badge-ready','badge-updating');otaBadge.classList.add(data.ota?'badge-updating':'badge-ready');otaBadge.textContent=data.ota?'Update in Progress':'Ready';}updateLastRefreshTime();}).catch(err=>console.error('Error loading status:',err));}";
    page += "window.addEventListener('load',function(){initChart();loadStatus();loadData();const b3=document.getElementById('range3h');const b12=document.getElementById('range12h');const b24=document.getElementById('range24h');const rz=document.getElementById('resetZoomBtn');if(b3)b3.addEventListener('click',()=>setRange('3h'));if(b12)b12.addEventListener('click',()=>setRange('12h'));if(b24)b24.addEventListener('click',()=>setRange('24h'));if(rz)rz.addEventListener('click',resetZoom);setInterval(loadData,180000);setInterval(loadStatus,10000);});";
    page += "</script>";
    page += "</body></html>";

    int pageLen = page.length();
    if (SERIAL_VERBOSE) {
      Serial.print("HTML page length: ");
      Serial.println(pageLen);
      Serial.print("Free heap before send: ");
      Serial.println(ESP.getFreeHeap());
    }
    if (!page.endsWith("</html>")) {
      Serial.println("ERROR: Page String is incomplete!");
      Serial.print("Page ends with: ");
      Serial.println(page.substring(max(0, pageLen - 50)));
      return;
    }
    server.send(200, "text/html", page);
  });

  // Start web server early so webpage is accessible even if sensors fail
  if (WiFi.status() == WL_CONNECTED) {
    server.begin();
    Serial.println("Web server started! Access at http://" + WiFi.localIP().toString());
  }

  // Now initialize sensors (after web server is running)
  sensors.begin();
  
  // Try to read sensors, but don't block if they're not connected
  delay(500);
  sensors.requestTemperatures();
  tExternal = sensors.getTempC(externalSensor);  // External sensor (28:7D:B0:B0:0F:00:00:4D)
  tFreezer  = sensors.getTempC(freezerSensor);   // Freezer sensor (28:89:89:87:00:3C:05:5A)
  tFridge   = sensors.getTempC(fridgeSensor);    // Fridge sensor (28:4F:2E:AF:0F:00:00:79)

  externalOk = (tExternal != DEVICE_DISCONNECTED_C && tExternal > -100);
  freezerOk  = (tFreezer  != DEVICE_DISCONNECTED_C && tFreezer  > -100);
  fridgeOk   = (tFridge   != DEVICE_DISCONNECTED_C && tFridge   > -100);

  scanSensors();

  // Store initial reading
  unsigned long nowEpoch = currentEpoch();
  readings[readingIndex].timestamp = nowEpoch;
  readings[readingIndex].tExternal = externalOk ? tExternal : -127;
  readings[readingIndex].tFreezer  = freezerOk  ? tFreezer  : -127;
  readings[readingIndex].tFridge   = fridgeOk   ? tFridge   : -127;

  readingIndex  = (readingIndex + 1) % MAX_READINGS;
  readingCount  = 1;

  if (client.connected()) {
    client.publish(topic_availability, "online", true);
  }
}

void loop() {
  checkWiFiConnection();
  ArduinoOTA.handle();

  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }

  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();
  if (now - lastRead >= interval) {
    lastRead = now;

    sensors.requestTemperatures();
    tExternal = sensors.getTempC(externalSensor);  // External sensor (28:7D:B0:B0:0F:00:00:4D)
    tFreezer  = sensors.getTempC(freezerSensor);   // Freezer sensor (28:89:89:87:00:3C:05:5A)
    tFridge   = sensors.getTempC(fridgeSensor);    // Fridge sensor (28:4F:2E:AF:0F:00:00:79)

    externalOk = (tExternal != DEVICE_DISCONNECTED_C && tExternal > -100);
    freezerOk  = (tFreezer  != DEVICE_DISCONNECTED_C && tFreezer  > -100);
    fridgeOk   = (tFridge   != DEVICE_DISCONNECTED_C && tFridge   > -100);

    checkAlerts();  // Update alert states
    scanSensors();

    // Publish mapped temps
    if (externalOk) client.publish(topic_external, String(tExternal).c_str(), true);
    if (freezerOk)  client.publish(topic_freezer, String(tFreezer).c_str(), true);
    if (fridgeOk)   client.publish(topic_fridge, String(tFridge).c_str(), true);

    if (now - lastGraphSave >= GRAPH_INTERVAL) {
      lastGraphSave = now;
      unsigned long nowEpoch = currentEpoch();
      readings[readingIndex].timestamp = nowEpoch;
      readings[readingIndex].tExternal = externalOk ? tExternal : -127;
      readings[readingIndex].tFreezer  = freezerOk  ? tFreezer  : -127;
      readings[readingIndex].tFridge   = fridgeOk   ? tFridge   : -127;

      readingIndex = (readingIndex + 1) % MAX_READINGS;
      if (readingCount < MAX_READINGS) readingCount++;
    }
  }
}