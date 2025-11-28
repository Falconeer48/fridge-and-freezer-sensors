#include <OneWire.h>
#include <DallasTemperature.h>

// OneWire bus pin (same as main sketch)
#define ONE_WIRE_BUS 18

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(":");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== OneWire Sensor Scanner ===");
  Serial.println("Scanning for all devices on GPIO 18...\n");
  
  sensors.begin();
  delay(500);
  
  int deviceCount = sensors.getDeviceCount();
  
  Serial.print("Total sensors found: ");
  Serial.println(deviceCount);
  Serial.println();
  
  if (deviceCount == 0) {
    Serial.println("ERROR: No sensors detected!");
    Serial.println("Check wiring and pull-up resistor (4.7kΩ between DATA and VCC).");
    Serial.println("\nTroubleshooting:");
    Serial.println("1. Verify power is connected (VCC and GND)");
    Serial.println("2. Check DATA line is connected to GPIO 18");
    Serial.println("3. Verify pull-up resistor (4.7kΩ) is connected");
    Serial.println("4. Check all connections are secure");
    return;
  }
  
  Serial.println("Physical order on breadboard (left to right):");
  Serial.println("Position 1 = Leftmost sensor");
  Serial.println("Position 2 = Middle sensor");
  Serial.println("Position 3 = Rightmost sensor");
  Serial.println();
  
  Serial.println("=== All Detected Sensors ===");
  DeviceAddress tempDeviceAddress;
  
  for (int i = 0; i < deviceCount; i++) {
    if (sensors.getAddress(tempDeviceAddress, i)) {
      Serial.print("Position ");
      Serial.print(i + 1);
      Serial.print(" (Index ");
      Serial.print(i);
      Serial.print("): ");
      printAddress(tempDeviceAddress);
      Serial.println();
    }
  }
  
  Serial.println();
  Serial.println("=== Reading Temperatures ===");
  sensors.requestTemperatures();
  delay(750); // Wait for conversion
  
  for (int i = 0; i < deviceCount; i++) {
    if (sensors.getAddress(tempDeviceAddress, i)) {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(" (");
      printAddress(tempDeviceAddress);
      Serial.print("): ");
      
      float tempC = sensors.getTempC(tempDeviceAddress);
      if (tempC != DEVICE_DISCONNECTED_C) {
        Serial.print(tempC, 2);
        Serial.println(" °C");
      } else {
        Serial.println("ERROR - Device disconnected");
      }
    }
  }
  
  Serial.println();
  Serial.println("=== Copy-Paste Format ===");
  Serial.println("Use these addresses in your main sketch:");
  Serial.println();
  
  for (int i = 0; i < deviceCount; i++) {
    if (sensors.getAddress(tempDeviceAddress, i)) {
      Serial.print("DeviceAddress sensor");
      Serial.print(i + 1);
      Serial.print(" = { ");
      for (uint8_t j = 0; j < 8; j++) {
        Serial.print("0x");
        if (tempDeviceAddress[j] < 16) Serial.print("0");
        Serial.print(tempDeviceAddress[j], HEX);
        if (j < 7) Serial.print(", ");
      }
      Serial.println(" };");
    }
  }
  
  Serial.println();
  Serial.println("=== Scan Complete ===");
}

void loop() {
  // Nothing to do in loop - just scan once on startup
  delay(10000);
}

