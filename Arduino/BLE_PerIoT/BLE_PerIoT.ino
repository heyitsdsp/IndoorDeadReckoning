#include <ArduinoBLE.h>

void setup() {
  Serial.begin(9600);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("Nano33IoT");
  BLE.advertise();

  Serial.println("Peripheral advertising...");
}

void loop() {
  // Do nothing here, just advertise
  BLE.poll();
}

