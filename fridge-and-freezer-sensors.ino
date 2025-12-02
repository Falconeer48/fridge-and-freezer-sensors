#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <time.h>   // For real-time clock via NTP

// Sketch version information
const char* SKETCH_NAME = "fridge-and-freezer-sensors";
const char* SKETCH_FOLDER = "/Users/ian/Documents/Arduino/fridge-and-freezer-sensors";
const char* SKETCH_VERSION = "1.0.0";

const char* ssid     = "Falconeer48_EXT";
const char* password = "0795418296";

const char* mqtt_server = "192.168.50.231";
const char* mqtt_user   = "ian";
const char* mqtt_pass   = "Falcon1959";

// NTP / time (South Africa = UTC+2, no DST)
const char* ntpServer         = "pool.ntp.org";
const long  gmtOffset_sec     = 2 * 3600;
const int   daylightOffset_sec = 0;

WiFiClient espClient;
PubSubClient client(espClient);
WebServer server(80);

#define ONE_WIRE_BUS 18
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Buzzer pin (using GPIO 4 - separate from OneWire bus on GPIO 18)
// GPIO 4 is a good choice: no pull-up resistors, not used by sensors
#define BUZZER_PIN 4

// Temperature alert thresholds
const float EXTERNAL_MAX_TEMP = 8.0;    // External too warm (°C)
const float EXTERNAL_MIN_TEMP = 0.0;    // External too cold (°C)
const float FREEZER_MAX_TEMP = -12.0;   // Freezer too warm (°C)
const float FREEZER_MIN_TEMP = -25.0;   // Freezer too cold (°C)
const float FRIDGE_MAX_TEMP = 8.0;      // Fridge too warm (°C)
const float FRIDGE_MIN_TEMP = 0.0;      // Fridge too cold (°C)

// Alert state tracking
bool externalAlert = false;
bool freezerAlert = false;
bool fridgeAlert = false;
bool sensorFailureAlert = false;
unsigned long lastBuzzerBeep = 0;
unsigned long buzzerBeepStart = 0;
bool buzzerActive = false;
const unsigned long BUZZER_INTERVAL = 2000;  // Beep every 2 seconds when alert active
const unsigned long BUZZER_BEEP_DURATION = 300;  // Beep duration in milliseconds
const int BUZZER_FREQUENCY = 2000;  // 2kHz frequency (works better than 4kHz)
const int BUZZER_DUTY = 230;  // 90% duty cycle (255 = 100%, 230 = 90%) - very loud
bool buzzerPWMAttached = false;

// Final sensor mapping after swap:
// External stays the same
// Freezer now uses old External address
// Fridge now uses old Freezer address
DeviceAddress externalSensor = { 0x28, 0x4F, 0x2E, 0xAF, 0x0F, 0x00, 0x00, 0x79 };
DeviceAddress freezerSensor  = { 0x28, 0x89, 0x89, 0x87, 0x00, 0x3C, 0x05, 0x5A };
DeviceAddress fridgeSensor   = { 0x28, 0x7D, 0xB0, 0xB0, 0x0F, 0x00, 0x00, 0x4D };

// MQTT topic paths
const char* topic_external = "home/kitchen/fridge/temperature";
const char* topic_freezer  = "home/kitchen/freezer/temperature";
const char* topic_fridge   = "home/kitchen/external/temperature";
const char* topic_availability = "home/kitchen/sensors/availability";

unsigned long lastRead = 0;
const unsigned long interval = 60000;  // 1 minute - MQTT publish interval
bool otaInProgress = false;

// Temperature variables
float tExternal = -127;
float tFreezer  = -127;
float tFridge   = -127;

bool externalOk = false;
bool freezerOk  = false;
bool fridgeOk   = false;

// Graph data storage - 3 hours at 3 minute intervals (60 readings)
const int MAX_READINGS = 60;  // 3 hours * 60 minutes / 3 = 60
const unsigned long GRAPH_INTERVAL = 180000;  // 3 minutes in milliseconds

struct TempReading {
  unsigned long timestamp;   // epoch seconds
  float tExternal;
  float tFreezer;
  float tFridge;
};

TempReading readings[MAX_READINGS];
int readingIndex = 0;
int readingCount = 0;
unsigned long lastGraphSave = 0;


void reconnect() {
  // Non-blocking reconnect - try once per call
  if (!client.connected() && WiFi.status() == WL_CONNECTED) {
    if (client.connect("ESP32-fridge", mqtt_user, mqtt_pass, topic_availability, 0, true, "offline")) {
      client.publish(topic_availability, "online", true);
    }
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
    // Time not set yet, fallback to millis-based relative seconds
    return millis() / 1000;
  }
  return (unsigned long)nowEpoch;
}

// Buzzer control functions - using PWM for maximum volume
void buzzerOn() {
  // Use PWM at 90% duty cycle for loudest sound (keeping PWM attached)
  if (!buzzerPWMAttached) {
    uint32_t channel = ledcAttach(BUZZER_PIN, BUZZER_FREQUENCY, 8);
    if (channel != 0) {
      buzzerPWMAttached = true;
    }
  }
  if (buzzerPWMAttached) {
    ledcWrite(BUZZER_PIN, BUZZER_DUTY);  // 90% duty cycle = very loud
  } else {
    // Fallback to digital if PWM fails
    digitalWrite(BUZZER_PIN, HIGH);
  }
  buzzerActive = true;
}

void buzzerOff() {
  if (buzzerPWMAttached) {
    ledcWrite(BUZZER_PIN, 0);  // Set duty to 0 (silent) but keep PWM attached
    // Don't detach PWM - keep it attached for next beep (this prevents clicking)
    // ledcDetach(BUZZER_PIN);
    // buzzerPWMAttached = false;
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
  buzzerActive = false;
}

void buzzerBeep(int duration = 300) {
  buzzerOn();
  buzzerBeepStart = millis();
}

// Non-blocking buzzer update (call this in loop)
void updateBuzzer() {
  if (buzzerActive) {
    // Check if beep duration has elapsed
    if (millis() - buzzerBeepStart >= BUZZER_BEEP_DURATION) {
      buzzerOff();
    }
  }
}

// Check for temperature alerts
void checkAlerts() {
  bool anyAlert = false;
  
  // Check external - only alert if sensor is OK but temperature is out of range
  if (externalOk) {
    if (tExternal > EXTERNAL_MAX_TEMP || tExternal < EXTERNAL_MIN_TEMP) {
      externalAlert = true;
      anyAlert = true;
    } else {
      externalAlert = false;
    }
  } else {
    externalAlert = false;  // Sensor failure - don't trigger buzzer
  }
  
  // Check freezer - only alert if sensor is OK but temperature is out of range
  if (freezerOk) {
    if (tFreezer > FREEZER_MAX_TEMP || tFreezer < FREEZER_MIN_TEMP) {
      freezerAlert = true;
      anyAlert = true;
    } else {
      freezerAlert = false;
    }
  } else {
    freezerAlert = false;  // Sensor failure - don't trigger buzzer
  }
  
  // Check fridge - only alert if sensor is OK but temperature is out of range
  if (fridgeOk) {
    if (tFridge > FRIDGE_MAX_TEMP || tFridge < FRIDGE_MIN_TEMP) {
      fridgeAlert = true;
      anyAlert = true;
    } else {
      fridgeAlert = false;
    }
  } else {
    fridgeAlert = false;  // Sensor failure - don't trigger buzzer
  }
  
  // Check for sensor failures (for display purposes only, not for buzzer)
  sensorFailureAlert = !fridgeOk || !freezerOk || !externalOk;
  
  // Control buzzer based on alerts (non-blocking)
  // Only beep for temperature out of range, NOT for sensor failures
  unsigned long now = millis();
  if (anyAlert) {
    // Start a new beep if interval has elapsed and buzzer is not currently active
    if (now - lastBuzzerBeep >= BUZZER_INTERVAL && !buzzerActive) {
      buzzerBeep(BUZZER_BEEP_DURATION);
      lastBuzzerBeep = now;
    }
  } else {
    // No alerts, ensure buzzer is off
    buzzerOff();
    lastBuzzerBeep = 0;
  }
}

void scanSensors() {
  Serial.println("\n=== SENSOR DIAGNOSTICS ===");
  Serial.print("Total sensors found: ");
  int deviceCount = sensors.getDeviceCount();
  Serial.println(deviceCount);
  
  if (deviceCount == 0) {
    Serial.println("ERROR: No sensors detected!");
    Serial.println("Check wiring and pull-up resistor.");
    return;
  }
  
  Serial.println("\nPHYSICAL ORDER ON BREADBOARD (left to right):");
  Serial.println("Position 1 = Leftmost sensor");
  Serial.println("Position 2 = Middle sensor");
  Serial.println("Position 3 = Rightmost sensor");
  Serial.println();
  
  Serial.println("Scanning for all connected sensors:");
  DeviceAddress tempDeviceAddress;
  for (int i = 0; i < deviceCount; i++) {
    if (sensors.getAddress(tempDeviceAddress, i)) {
      Serial.print("  Position ");
      Serial.print(i + 1);
      Serial.print(" (Sensor ");
      Serial.print(i + 1);
      Serial.print("): ");
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
      else if (isFridge) Serial.print(" <-- FRIDGE");
      else Serial.print(" <-- UNKNOWN SENSOR");
      
      Serial.println();
    }
  }
  
  Serial.println("\nExpected sensor addresses:");
  Serial.print("  External:  ");
  printAddress(externalSensor);
  Serial.println();
  Serial.print("  Freezer:   ");
  printAddress(freezerSensor);
  Serial.println();
  Serial.print("  Fridge:    ");
  printAddress(fridgeSensor);
  Serial.println();
  
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
  // Serial output enabled for diagnostics
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nESP32 Fridge/Freezer Sensor System Starting...");
  
  delay(500); // Small delay for stable boot
  
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi failed to connect.");
  } else {
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  }

  // Set up time via NTP
  setupTime();

  ArduinoOTA.setHostname("fridge-freezer");
  ArduinoOTA.onStart([]() { otaInProgress = true; });
  ArduinoOTA.onEnd([]()   { otaInProgress = false; });
  ArduinoOTA.begin();

  client.setServer(mqtt_server, 1883);
  sensors.begin();
  
  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Test buzzer on startup (3 short beeps at maximum volume)
  Serial.println("Testing buzzer...");
  for (int i = 0; i < 3; i++) {
    buzzerOn();
    delay(150);
    buzzerOff();
    delay(100);
  }
  Serial.println("Buzzer test complete.");
  
  // Only start mDNS and web server if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
    if (!MDNS.begin("fridge-freezer")) {
      // mDNS failed to start - could add error handling
    }
  }
  
  delay(1000);
  sensors.requestTemperatures();
  tExternal = sensors.getTempC(externalSensor);
  tFreezer  = sensors.getTempC(freezerSensor);
  tFridge   = sensors.getTempC(fridgeSensor);
  
  externalOk = (tExternal != DEVICE_DISCONNECTED_C && tExternal > -100);
  freezerOk  = (tFreezer  != DEVICE_DISCONNECTED_C && tFreezer  > -100);
  fridgeOk   = (tFridge   != DEVICE_DISCONNECTED_C && tFridge   > -100);
  
  // Run sensor diagnostics
  scanSensors();
  
  for (int i = 0; i < MAX_READINGS; i++) {
    readings[i].timestamp = 0;
    readings[i].tExternal = -127;
    readings[i].tFreezer  = -127;
    readings[i].tFridge   = -127;
  }
  
  unsigned long nowEpoch = currentEpoch();
  readings[readingIndex].timestamp = nowEpoch;
  readings[readingIndex].tExternal = externalOk ? tExternal : -127;
  readings[readingIndex].tFreezer  = freezerOk  ? tFreezer  : -127;
  readings[readingIndex].tFridge   = fridgeOk   ? tFridge   : -127;
  
  readingIndex = (readingIndex + 1) % MAX_READINGS;
  readingCount = 1;
  lastGraphSave = millis();
  
  // /data endpoint: graph history JSON (last 3 hours)
  server.on("/data", []() {
    unsigned long nowEpoch = currentEpoch();
    unsigned long cutoffTime = nowEpoch - 3UL * 3600UL;  // last 3 hours
    
    String json = "{\"now\":" + String(nowEpoch) + ",\"data\":[";
    
    int startIdx = (readingCount < MAX_READINGS) ? 0 : readingIndex;
    int count = (readingCount < MAX_READINGS) ? readingCount : MAX_READINGS;
    bool first = true;
    
    for (int i = 0; i < count; i++) {
      int idx = (startIdx + i) % MAX_READINGS;
      if (readings[idx].timestamp >= cutoffTime && readings[idx].timestamp != 0) {
        if (!first) json += ",";
        first = false;
        json += "{";
        json += "\"t\":"         + String(readings[idx].timestamp) + ",";
        json += "\"tExternal\":" + String(readings[idx].tExternal, 1) + ",";
        json += "\"tFreezer\":"  + String(readings[idx].tFreezer, 1) + ",";
        json += "\"tFridge\":"   + String(readings[idx].tFridge, 1);
        json += "}";
      }
    }
    json += "]}";
    server.send(200, "application/json", json);
  });

  // /status endpoint: live values + status for AJAX
  server.on("/status", []() {
    int rssi = WiFi.RSSI();
    String json = "{";
    json += "\"tExternal\":"  + String(tExternal, 1) + ",";
    json += "\"tFreezer\":"   + String(tFreezer, 1)  + ",";
    json += "\"tFridge\":"    + String(tFridge, 1)   + ",";
    json += "\"okExternal\":" + String(externalOk ? "true" : "false") + ",";
    json += "\"okFreezer\":"  + String(freezerOk  ? "true" : "false") + ",";
    json += "\"okFridge\":"   + String(fridgeOk   ? "true" : "false") + ",";
    json += "\"rssi\":"       + String(rssi) + ",";
    json += "\"mqtt\":"       + String(client.connected() ? "true" : "false") + ",";
    json += "\"ota\":"        + String(otaInProgress ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  });
  
  server.on("/", []() {
    String page = "<!DOCTYPE html><html lang='en'><head>";
    page += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    page += "<title>Sensor Status Dashboard</title>";
    page += "<style>";
    page += "* { margin: 0; padding: 0; box-sizing: border-box; }";
    page += "body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; padding: 20px; }";
    page += ".container { max-width: 800px; margin: 0 auto; background: rgba(255, 255, 255, 0.95); border-radius: 20px; box-shadow: 0 10px 40px rgba(0, 0, 0, 0.2); padding: 40px; }";
    page += "h1 { color: #333; text-align: center; margin-bottom: 10px; font-size: 2.5em; font-weight: 600; }";
    page += ".subtitle { text-align: center; color: #666; margin-bottom: 30px; font-size: 0.9em; }";
    page += ".sensor-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 20px; margin-bottom: 30px; }";
    page += ".sensor-card { background: #f8f9fa; border-radius: 15px; padding: 25px; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); transition: transform 0.3s ease; }";
    page += ".sensor-card:hover { transform: translateY(-5px); box-shadow: 0 6px 12px rgba(0, 0, 0, 0.15); }";
    page += ".sensor-title { font-size: 1.2em; font-weight: 600; color: #555; margin-bottom: 15px; display: flex; align-items: center; gap: 10px; }";
    page += ".status-indicator { width: 12px; height: 12px; border-radius: 50%; display: inline-block; }";
    page += ".status-ok { background: #28a745; box-shadow: 0 0 10px rgba(40, 167, 69, 0.5); }";
    page += ".status-error { background: #dc3545; box-shadow: 0 0 10px rgba(220, 53, 69, 0.5); }";
    page += ".temperature { font-size: 2.5em; font-weight: 700; color: #667eea; margin: 10px 0; }";
    page += ".unit { font-size: 0.6em; color: #999; }";
    page += ".status-text { font-size: 0.9em; }";
    page += ".status-text-ok { color: #28a745; }";
    page += ".status-text-error { color: #dc3545; }";
    page += ".info-section { background: #f8f9fa; border-radius: 15px; padding: 25px; margin-top: 20px; }";
    page += ".fade-value { transition: opacity 0.3s ease; }";
    page += ".info-title { font-size: 1.3em; font-weight: 600; color: #333; margin-bottom: 15px; border-bottom: 2px solid #667eea; padding-bottom: 10px; }";
    page += ".info-row { display: flex; justify-content: space-between; align-items: center; padding: 12px 0; border-bottom: 1px solid #e0e0e0; }";
    page += ".info-row:last-child { border-bottom: none; }";
    page += ".info-label { font-weight: 600; color: #555; }";
    page += ".info-value { color: #333; font-family: 'Courier New', monospace; }";
    page += ".badge { display: inline-block; padding: 5px 15px; border-radius: 20px; font-size: 0.85em; font-weight: 600; }";
    page += ".badge-connected { background: #d4edda; color: #155724; }";
    page += ".badge-disconnected { background: #f8d7da; color: #721c24; }";
    page += ".badge-ready { background: #d1ecf1; color: #0c5460; }";
    page += ".badge-updating { background: #fff3cd; color: #856404; }";
    page += ".badge-signal-strong { background: #d4edda; color: #155724; }";
    page += ".badge-signal-medium { background: #fff3cd; color: #856404; }";
    page += ".badge-signal-weak { background: #f8d7da; color: #721c24; }";
    page += ".chart-section { background: #f8f9fa; border-radius: 15px; padding: 25px; margin-top: 20px; }";
    page += ".chart-title { font-size: 1.3em; font-weight: 600; color: #333; margin-bottom: 15px; border-bottom: 2px solid #667eea; padding-bottom: 10px; }";
    page += ".chart-container { position: relative; height: 300px; margin-top: 20px; }";
    page += "@media (max-width: 600px) { .container { padding: 20px; } h1 { font-size: 1.8em; } .temperature { font-size: 2em; } .chart-container { height: 250px; } }";
    page += "</style>";
    page += "<script src='https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js'></script>";
    page += "</head><body>";
    page += "<div class='container'>";
    page += "<h1>Sensor Status Dashboard</h1>";
    page += "<p class='subtitle'>Real-time Temperature Monitoring</p>";
    page += "<div class='sensor-grid'>";
    
    // External Card
    page += "<div class='sensor-card'>";
    page += "<div class='sensor-title'>";
    page += "<span id='statusExternalIndicator' class='status-indicator " + String(externalOk ? "status-ok" : "status-error") + "'></span>";
    page += "External";
    page += "</div>";
    if (externalOk) {
      page += "<div class='temperature'><span id='tempExternal' class='fade-value'>" + String(tExternal, 1) + "</span><span class='unit'>°C</span></div>";
      page += "<div id='statusExternalText' class='status-text status-text-ok'>Sensor Active</div>";
    } else {
      page += "<div class='temperature'><span id='tempExternal' class='fade-value'>—</span><span class='unit'>°C</span></div>";
      page += "<div id='statusExternalText' class='status-text status-text-error'>Sensor Not Detected</div>";
    }
    page += "</div>";
    
    // Freezer Card
    page += "<div class='sensor-card'>";
    page += "<div class='sensor-title'>";
    page += "<span id='statusFreezerIndicator' class='status-indicator " + String(freezerOk ? "status-ok" : "status-error") + "'></span>";
    page += "Freezer";
    page += "</div>";
    if (freezerOk) {
      page += "<div class='temperature'><span id='tempFreezer' class='fade-value'>" + String(tFreezer, 1) + "</span><span class='unit'>°C</span></div>";
      page += "<div id='statusFreezerText' class='status-text status-text-ok'>Sensor Active</div>";
    } else {
      page += "<div class='temperature'><span id='tempFreezer' class='fade-value'>—</span><span class='unit'>°C</span></div>";
      page += "<div id='statusFreezerText' class='status-text status-text-error'>Sensor Not Detected</div>";
    }
    page += "</div>";
    
    // Fridge Card
    page += "<div class='sensor-card'>";
    page += "<div class='sensor-title'>";
    page += "<span id='statusFridgeIndicator' class='status-indicator " + String(fridgeOk ? "status-ok" : "status-error") + "'></span>";
    page += "Fridge";
    page += "</div>";
    if (fridgeOk) {
      page += "<div class='temperature'><span id='tempFridge' class='fade-value'>" + String(tFridge, 1) + "</span><span class='unit'>°C</span></div>";
      page += "<div id='statusFridgeText' class='status-text status-text-ok'>Sensor Active</div>";
    } else {
      page += "<div class='temperature'><span id='tempFridge' class='fade-value'>—</span><span class='unit'>°C</span></div>";
      page += "<div id='statusFridgeText' class='status-text status-text-error'>Sensor Not Detected</div>";
    }
    page += "</div>";
    
    page += "</div>"; // Close sensor-grid
    
    // System Information Section
    page += "<div class='info-section'>";
    page += "<div class='info-title'>System Information</div>";
    page += "<div class='info-row'><span class='info-label'>Wi-Fi IP Address</span><span class='info-value'>" + WiFi.localIP().toString() + "</span></div>";
    
    // WiFi Signal Strength with color coding
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
    page += "<div class='info-row'><span class='info-label'>Wi-Fi Signal Strength</span><span class='info-value'><span id='wifiSignalBadge' class='badge " + signalClass + "'>" + signalStrength + "</span></span></div>";
    
    page += "<div class='info-row'><span class='info-label'>MQTT Status</span><span class='info-value'><span id='mqttStatusBadge' class='badge " + String(client.connected() ? "badge-connected" : "badge-disconnected") + "'>" + String(client.connected() ? "Connected" : "Disconnected") + "</span></span></div>";
    page += "<div class='info-row'><span class='info-label'>OTA Status</span><span class='info-value'><span id='otaStatusBadge' class='badge " + String(otaInProgress ? "badge-updating" : "badge-ready") + "'>" + String(otaInProgress ? "Update in Progress" : "Ready") + "</span></span></div>";
    page += "<div class='info-row'><span class='info-label'>Last Refresh</span><span class='info-value' id='lastRefreshTime'>—</span></div>";
    page += "<div class='info-row'><span class='info-label'>Sketch Name</span><span class='info-value'>" + String(SKETCH_NAME) + "</span></div>";
    page += "<div class='info-row'><span class='info-label'>Sketch Folder</span><span class='info-value'>" + String(SKETCH_FOLDER) + "</span></div>";
    page += "<div class='info-row'><span class='info-label'>Version</span><span class='info-value'>" + String(SKETCH_VERSION) + "</span></div>";
    page += "</div>"; // info-section
    
    // Chart Section
    page += "<div class='chart-section'>";
    page += "<div class='chart-title'>Temperature History (Last 3 Hours)</div>";
    page += "<div class='chart-container'><canvas id='tempChart'></canvas></div>";
    page += "</div>";
    
    page += "</div>"; // container
    
    // JavaScript
    page += "<script>";
    page += "let chart = null;";
    
    page += "function updateTextWithFade(id, text) {";
    page += "  const el = document.getElementById(id);";
    page += "  if (!el) return;";
    page += "  if (el.textContent === text) return;";
    page += "  el.style.opacity = 0;";
    page += "  setTimeout(() => { el.textContent = text; el.style.opacity = 1; }, 300);";
    page += "}";
    
    page += "function updateLastRefreshTime() {";
    page += "  const now = new Date();";
    page += "  const timeStr = now.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });";
    page += "  const el = document.getElementById('lastRefreshTime');";
    page += "  if (el) el.textContent = timeStr;";
    page += "}";
    
    page += "function initChart() {";
    page += "  const canvas = document.getElementById('tempChart');";
    page += "  if (!canvas || typeof Chart === 'undefined') return;";
    page += "  const ctx = canvas.getContext('2d');";
    page += "  chart = new Chart(ctx, {";
    page += "    type: 'line',";
    page += "    data: { labels: [], datasets: [";
    page += "      { label: 'External', data: [], borderColor: '#667eea', backgroundColor: 'rgba(102, 126, 234, 0.1)', tension: 0.4, fill: true, pointRadius: 0, pointHoverRadius: 0 },";
    page += "      { label: 'Freezer', data: [], borderColor: '#f093fb', backgroundColor: 'rgba(240, 147, 251, 0.1)', tension: 0.4, fill: true, pointRadius: 0, pointHoverRadius: 0 },";
    page += "      { label: 'Fridge', data: [], borderColor: '#4facfe', backgroundColor: 'rgba(79, 172, 254, 0.1)', tension: 0.4, fill: true, pointRadius: 0, pointHoverRadius: 0 }";
    page += "    ] },";
    page += "    options: {";
    page += "      responsive: true,";
    page += "      maintainAspectRatio: false,";
    page += "      spanGaps: true,";
    page += "      elements: { point: { radius: 0 }, line: { borderWidth: 2 } },";
    page += "      plugins: { legend: { position: 'top', labels: { font: { size: 12 } } } },";
    page += "      scales: {";
    page += "        x: {";
    page += "          title: { display: true, text: 'Time' }";
    page += "        },";
    page += "        y: {";
    page += "          title: { display: true, text: 'Temperature (°C)' }";
    page += "        }";
    page += "      }";
    page += "    }";
    page += "  });";
    page += "  loadData();";
    page += "}";
    
    page += "function loadData() {";
    page += "  if (!chart) return;";
    page += "  fetch('/data').then(r => r.json()).then(result => {";
    page += "    const labels = []; const externalData = []; const freezerData = []; const fridgeData = [];";
    page += "    if (result.data && result.data.length > 0) {";
    page += "      result.data.forEach(item => {";
    page += "        const date = new Date(item.t * 1000);";  // epoch seconds → ms
    page += "        const label = date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });";
    page += "        const externalValid = item.tExternal > -100 && item.tExternal < 100;";
    page += "        const freezerValid = item.tFreezer > -100 && item.tFreezer < 100;";
    page += "        const fridgeValid = item.tFridge > -100 && item.tFridge < 100;";
    page += "        if (externalValid || freezerValid || fridgeValid) {";
    page += "          labels.push(label);";
    page += "          externalData.push(externalValid ? parseFloat(item.tExternal) : NaN);";
    page += "          freezerData.push(freezerValid ? parseFloat(item.tFreezer) : NaN);";
    page += "          fridgeData.push(fridgeValid ? parseFloat(item.tFridge) : NaN);";
    page += "        }";
    page += "      });";
    page += "    }";
    page += "    chart.data.labels = labels;";
    page += "    chart.data.datasets[0].data = externalData;";
    page += "    chart.data.datasets[1].data = freezerData;";
    page += "    chart.data.datasets[2].data = fridgeData;";
    page += "    chart.update();";
    page += "  }).catch(err => console.error('Error loading data:', err));";
    page += "}";
    
    page += "function loadStatus() {";
    page += "  fetch('/status').then(r => r.json()).then(data => {";
    page += "    updateTextWithFade('tempExternal', data.tExternal.toFixed(1));";
    page += "    updateTextWithFade('tempFreezer', data.tFreezer.toFixed(1));";
    page += "    updateTextWithFade('tempFridge', data.tFridge.toFixed(1));";
    
    page += "    const externalOk = data.okExternal;";
    page += "    const freezerOk = data.okFreezer;";
    page += "    const fridgeOk = data.okFridge;";
    
    page += "    const indX = document.getElementById('statusExternalIndicator');";
    page += "    const txtX = document.getElementById('statusExternalText');";
    page += "    if (indX && txtX) {";
    page += "      indX.classList.remove('status-ok','status-error');";
    page += "      indX.classList.add(externalOk ? 'status-ok' : 'status-error');";
    page += "      txtX.classList.remove('status-text-ok','status-text-error');";
    page += "      txtX.classList.add(externalOk ? 'status-text-ok' : 'status-text-error');";
    page += "      txtX.textContent = externalOk ? 'Sensor Active' : 'Sensor Not Detected';";
    page += "    }";
    
    page += "    const indZ = document.getElementById('statusFreezerIndicator');";
    page += "    const txtZ = document.getElementById('statusFreezerText');";
    page += "    if (indZ && txtZ) {";
    page += "      indZ.classList.remove('status-ok','status-error');";
    page += "      indZ.classList.add(freezerOk ? 'status-ok' : 'status-error');";
    page += "      txtZ.classList.remove('status-text-ok','status-text-error');";
    page += "      txtZ.classList.add(freezerOk ? 'status-text-ok' : 'status-text-error');";
    page += "      txtZ.textContent = freezerOk ? 'Sensor Active' : 'Sensor Not Detected';";
    page += "    }";
    
    page += "    const indF = document.getElementById('statusFridgeIndicator');";
    page += "    const txtF = document.getElementById('statusFridgeText');";
    page += "    if (indF && txtF) {";
    page += "      indF.classList.remove('status-ok','status-error');";
    page += "      indF.classList.add(fridgeOk ? 'status-ok' : 'status-error');";
    page += "      txtF.classList.remove('status-text-ok','status-text-error');";
    page += "      txtF.classList.add(fridgeOk ? 'status-text-ok' : 'status-text-error');";
    page += "      txtF.textContent = fridgeOk ? 'Sensor Active' : 'Sensor Not Detected';";
    page += "    }";
    
    page += "    const wifiBadge = document.getElementById('wifiSignalBadge');";
    page += "    if (wifiBadge) {";
    page += "      let signalClass = 'badge-signal-weak';";
    page += "      let signalText = 'Weak (' + data.rssi + ' dBm)';";
    page += "      if (data.rssi > -50) { signalClass = 'badge-signal-strong'; signalText = 'Strong (' + data.rssi + ' dBm)'; }";
    page += "      else if (data.rssi > -70) { signalClass = 'badge-signal-medium'; signalText = 'Medium (' + data.rssi + ' dBm)'; }";
    page += "      wifiBadge.classList.remove('badge-signal-strong','badge-signal-medium','badge-signal-weak');";
    page += "      wifiBadge.classList.add(signalClass);";
    page += "      wifiBadge.textContent = signalText;";
    page += "    }";
    
    page += "    const mqttBadge = document.getElementById('mqttStatusBadge');";
    page += "    if (mqttBadge) {";
    page += "      mqttBadge.classList.remove('badge-connected','badge-disconnected');";
    page += "      mqttBadge.classList.add(data.mqtt ? 'badge-connected' : 'badge-disconnected');";
    page += "      mqttBadge.textContent = data.mqtt ? 'Connected' : 'Disconnected';";
    page += "    }";
    
    page += "    const otaBadge = document.getElementById('otaStatusBadge');";
    page += "    if (otaBadge) {";
    page += "      otaBadge.classList.remove('badge-ready','badge-updating');";
    page += "      otaBadge.classList.add(data.ota ? 'badge-updating' : 'badge-ready');";
    page += "      otaBadge.textContent = data.ota ? 'Update in Progress' : 'Ready';";
    page += "    }";
    
    page += "    updateLastRefreshTime();";
    page += "  }).catch(err => console.error('Error loading status:', err));";
    page += "}";
    
    page += "window.addEventListener('load', function() {";
    page += "  initChart();";
    page += "  loadStatus();";
    page += "  setInterval(loadData, 300000);";  // graph every 5 minutes
    page += "  setInterval(loadStatus, 10000);"; // status every 10 seconds
    page += "});";
    
    page += "</script>";
    
    page += "</body></html>";
    server.send(200, "text/html", page);
  });
  
  // Only start web server if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
    server.begin();
  }
  
  // Only publish MQTT if connected
  if (client.connected()) {
    client.publish(topic_availability, "online", true);
  }
}

void loop() {
  ArduinoOTA.handle();
  
  // Only handle web server if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }
  
  // Non-blocking MQTT reconnect
  if (!client.connected()) reconnect();
  client.loop();
  
  // Update buzzer (non-blocking)
  updateBuzzer();

  unsigned long now = millis();
  if (now - lastRead >= interval) {
    lastRead = now;

    sensors.requestTemperatures();
    tExternal = sensors.getTempC(externalSensor);
    tFreezer  = sensors.getTempC(freezerSensor);
    tFridge   = sensors.getTempC(fridgeSensor);

    externalOk = (tExternal != DEVICE_DISCONNECTED_C && tExternal > -100);
    freezerOk  = (tFreezer  != DEVICE_DISCONNECTED_C && tFreezer  > -100);
    fridgeOk   = (tFridge   != DEVICE_DISCONNECTED_C && tFridge   > -100);
    
    // Check for alerts and control buzzer
    checkAlerts();
    
    scanSensors();

    if (externalOk) client.publish(topic_external, String(tExternal).c_str(), true);
    if (freezerOk)  client.publish(topic_freezer,  String(tFreezer).c_str(), true);
    if (fridgeOk)   client.publish(topic_fridge,   String(tFridge).c_str(), true);
    
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
