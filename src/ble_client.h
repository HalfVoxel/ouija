/*
    Based on Neil Kolban example for IDF:
   https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp Ported to Arduino
   ESP32 by Evandro Copercini updates by chegewara
*/

#pragma once
#include "ble_server.h"
#include "time.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

const int scanTime = 5; // In seconds

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {};

struct OuijaBoardClient : public BLEAdvertisedDeviceCallbacks {
  BLEScan *scan;
  BLEClient *client;
  BLEAdvertisedDevice mAdvertisedDevice;
  bool foundDevice = false;
  bool touchActive;

  template <typename T> bool readData(BLEUUID characteristicUUID, T &value) const {
    if (!client->isConnected()) {
      Serial.println("Client is not connected");
      return false;
    }

    auto service = client->getService(SERVICE_UUID);
    if (service == nullptr) {
      Serial.println("Could not find service");
      return false;
    }
    auto characteristic = service->getCharacteristic(characteristicUUID);
    if (characteristic == nullptr) {
      Serial.println("Could not find characteristic");
      return false;
    }
    auto valueStr = characteristic->readValue();
    if (valueStr.size() != sizeof(T)) {
      Serial.print("Invalid value size. Client is probably disconnected. Size=");
      Serial.println(valueStr.size());
      delay(1000);
      return false;
    }
    memcpy(&value, valueStr.data(), sizeof(T));
    Serial.print("Value: ");
    Serial.println(value);

    return true;
  }

  void update() {
    uint32_t batteryPercent;
    uint32_t touchRaw;
    bool failed = !readData<uint32_t>(CHARACTERISTIC_UUID_BATTERY_PERCENTAGE, batteryPercent) ||
                  !readData<uint32_t>(CHARACTERISTIC_UUID_TOUCH_RAW, touchRaw);
    if (!failed) {
      Serial.print("Battery: ");
      Serial.print(batteryPercent);
      Serial.print(" Touch: ");
      Serial.println(touchRaw);
    }

    touchActive = !failed && touchRaw <= 400;

    if (failed) {
      Serial.println("Disconnecting...");
      client->disconnect();

      Serial.println("Scanning...");
      BLEScanResults foundDevices = scan->start(scanTime, false);

      // Add a small delay, because I think the library has a race condition that can cause it to try to
      // access the scan results a short duration after the scan has stopped, due to a gatt event.
      delay(10);

      scan->clearResults(); // delete results fromBLEScan buffer to release memory
      Serial.println("Finished scan");

      if (foundDevice) {
        foundDevice = false;
        Serial.println(mAdvertisedDevice.toString().c_str());
        Serial.println("Connecting...");
        if (client->connect(&mAdvertisedDevice)) {
          Serial.println("Connected");

          auto service = client->getService(SERVICE_UUID);
          if (service != nullptr) {
            auto characteristic = service->getCharacteristic(CHARACTERISTIC_UUID_TOUCH);
            if (characteristic != nullptr) {
              characteristic->registerForNotify(
                  [](BLERemoteCharacteristic *c, uint8_t *data, size_t len, bool isNotify) {
                    Serial.print("Notify: ");
                    Serial.println(c->getUUID().toString().c_str());
                    Serial.print("Data: ");
                    if (len == 4) {
                      uint32_t val;
                      memcpy(&val, data, 4);
                      Serial.println(val);
                    } else {
                      Serial.println("Invalid length");
                    }
                  });
            }
          }
        } else {
          Serial.println("Failed to connect");
        }
      }
    }
  }

  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.isAdvertisingService(SERVICE_UUID)) {
      this->mAdvertisedDevice = advertisedDevice;
      this->foundDevice = true;
      scan->stop(); // Stop scanning before connecting to the device. We cannot scan and connect at the same time.
    }
  }

  ~OuijaBoardClient() {
    scan->stop();
    delete scan;
  }
};

OuijaBoardClient *setupBluetoothClient() {
  BLEDevice::init("Ouija Board");
  auto pBLEScan = BLEDevice::getScan(); // create new scan
  auto client = new OuijaBoardClient();
  client->scan = pBLEScan;
  client->client = BLEDevice::createClient();
  pBLEScan->setAdvertisedDeviceCallbacks(client);
  pBLEScan->setActiveScan(true); // active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99); // less or equal setInterval value

  return client;
}
