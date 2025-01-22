#include <ArduinoBLE.h>

// Function prototypes
void printPeripheralData(BLECharacteristic characteristic);
void printData(const unsigned char data[], int length);

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1)
      ;
  }

  Serial.println("Bluetooth® Low Energy Central - Peripheral Explorer");

  // start scanning for peripherals
  BLE.scan();
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    // check peripheral
    if (peripheral.localName() == "Peripheral Arduino") {
      // stop scanning
      BLE.stopScan();

      Serial.println("Connecting ...");

      if (peripheral.connect()) {
        Serial.println("Connected");
      } else {
        Serial.println("Failed to connect!");
        return;
      }

      // discover peripheral attributes
      Serial.println("Discovering attributes ...");
      if (peripheral.discoverAttributes()) {
        Serial.println("Attributes discovered");
      } else {
        Serial.println("Attribute discovery failed!");
        peripheral.disconnect();
        return;
      }

      Serial.println();
      Serial.print("Device name: ");
      Serial.println(peripheral.deviceName());
      Serial.print("Appearance: 0x");
      Serial.println(peripheral.appearance(), HEX);
      Serial.println();

      while (peripheral.localName() == "Peripheral Arduino") {
        bool serviceFound = false;

        // Iterate through all services
        for (int i = 0; i < peripheral.serviceCount(); i++) {
          BLEService service = peripheral.service(i);

          // Iterate through all characteristics in this service
          for (int j = 0; j < service.characteristicCount(); j++) {
            BLECharacteristic characteristic = service.characteristic(j);

            // Check if the characteristic matches the target UUID
            if (characteristic.uuid() == "e93df100-b754-4fda-adf3-5ca2ea89bde3") {
              serviceFound = true;

              // Print the characteristic data
              printPeripheralData(characteristic);
            }
          }
        }
        // If no service or characteristic matches, print a message
        if (!serviceFound) {
          Serial.println("No service with the target UUID found");
        }
        Serial.println(peripheral.rssi());
      }
    }
  }
  delay(50);
}

void printPeripheralData(BLECharacteristic characteristic) {
  if (characteristic.canRead()) {
    // read the characteristic value
    characteristic.read();

    if (characteristic.valueLength() > 0) {
      // print out the value of the characteristic
      printData(characteristic.value(), characteristic.valueLength());
    }
  }
  Serial.println();
}

void printData(const unsigned char data[], int length) {
  for (int i = 0; i < length; i++) {
    unsigned char b = data[i];

    if (b < 16) {
      Serial.print("0");
    }

    Serial.print(b, HEX);
  }
}
