#include <ArduinoBLE.h>

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

      //explorerPeripheral(peripheral);

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

      /*BLEService service = peripheral.service(0);

      Serial.print("Service ");
      Serial.println(service.uuid());

      Serial.print("\tCharacteristic ");
      Serial.print(characteristic.uuid());
      Serial.print(", properties 0x");
      Serial.print(characteristic.properties(), HEX);
      Serial.println();*/

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
      //Serial.println(String(*characteristic.value()));
    }
  }
  Serial.println();
}

/*void explorerPeripheral(BLEDevice peripheral) {
  // connect to the peripheral
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

  // read and print device name of peripheral
  Serial.println();
  Serial.print("Device name: ");
  Serial.println(peripheral.deviceName());
  Serial.print("Appearance: 0x");
  Serial.println(peripheral.appearance(), HEX);
  Serial.println();

  // loop the services of the peripheral and explore each
  for (int i = 0; i < peripheral.serviceCount(); i++) {
    BLEService service = peripheral.service(i);

    exploreService(service);
  }

  Serial.println();

  // we are done exploring, disconnect
  Serial.println("Disconnecting ...");
  peripheral.disconnect();
  Serial.println("Disconnected");
}

void exploreService(BLEService service) {
  // print the UUID of the service
  Serial.print("Service ");
  Serial.println(service.uuid());

  // loop the characteristics of the service and explore each
  for (int i = 0; i < service.characteristicCount(); i++) {
    BLECharacteristic characteristic = service.characteristic(i);

    exploreCharacteristic(characteristic);
  }
}

void exploreCharacteristic(BLECharacteristic characteristic) {
  // print the UUID and properties of the characteristic
  Serial.print("\tCharacteristic ");
  Serial.print(characteristic.uuid());
  Serial.print(", properties 0x");
  Serial.print(characteristic.properties(), HEX);

  // check if the characteristic is readable
  if (characteristic.canRead()) {
    // read the characteristic value
    characteristic.read();

    if (characteristic.valueLength() > 0) {
      // print out the value of the characteristic
      Serial.print(", value 0x");
      printData(characteristic.value(), characteristic.valueLength());
    }
  }
  Serial.println();

  // loop the descriptors of the characteristic and explore each
  for (int i = 0; i < characteristic.descriptorCount(); i++) {
    BLEDescriptor descriptor = characteristic.descriptor(i);

    exploreDescriptor(descriptor);
  }
}

void exploreDescriptor(BLEDescriptor descriptor) {
  // print the UUID of the descriptor
  Serial.print("\t\tDescriptor ");
  Serial.print(descriptor.uuid());

  // read the descriptor value
  descriptor.read();

  // print out the value of the descriptor
  Serial.print(", value 0x");
  printData(descriptor.value(), descriptor.valueLength());

  Serial.println();
}*/

void printData(const unsigned char data[], int length) {
  for (int i = 0; i < length; i++) {
    unsigned char b = data[i];

    if (b < 16) {
      Serial.print("0");
    }

    //Serial.print(b, HEX);
    Serial.print(b, HEX);
  }
}
