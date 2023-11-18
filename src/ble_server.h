/*
    Based on Neil Kolban example for IDF:
   https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp Ported to Arduino
   ESP32 by Evandro Copercini updates by chegewara
*/

#pragma once
#include "battery.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

const BLEUUID SERVICE_UUID = BLEUUID("16fbc490-4e81-4b38-ace3-f78a1da5937b");
const BLEUUID CHARACTERISTIC_UUID_TIME = BLEUUID("75152a81-2aa4-4a0f-bc8e-9c756866b368");
const BLEUUID CHARACTERISTIC_UUID_TOUCH = BLEUUID(0x2AE2U); // "75152a81-2aa4-4a0f-bc8e-9c756866b368");
const BLEUUID CHARACTERISTIC_UUID_TOUCH_RAW = BLEUUID("75152a81-2aa4-4a0f-bc8e-9c756866b368");
const BLEUUID CHARACTERISTIC_UUID_BATTERY_PERCENTAGE = BLEUUID(0x2A19U); // "312ada26-4704-445a-995a-cc54dc878fd8");
const BLEUUID CHARACTERISTIC_UUID_BATTERY_VOLTAGE = BLEUUID(0x2B18U);

class PlanchetServer : public BLEServerCallbacks {
  BLEServer *mServer;
  BLEService *mService;
  BLECharacteristic *mCharacteristicTime;
  BLECharacteristic *mCharacteristicTouch;
  BLECharacteristic *mCharacteristicTouchRaw;
  BLECharacteristic *mCharacteristicBatteryPercentage;
  BLECharacteristic *mCharacteristicBatteryVoltage;

public:
  PlanchetServer(BLEServer *server, BLEService *service) : mServer(server), mService(service) {
    mCharacteristicTime = service->createCharacteristic(CHARACTERISTIC_UUID_TIME, BLECharacteristic::PROPERTY_READ);
    mCharacteristicTouch = service->createCharacteristic(
        CHARACTERISTIC_UUID_TOUCH, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    mCharacteristicTouchRaw =
        service->createCharacteristic(CHARACTERISTIC_UUID_TOUCH_RAW, BLECharacteristic::PROPERTY_READ);
    mCharacteristicBatteryPercentage =
        service->createCharacteristic(CHARACTERISTIC_UUID_BATTERY_PERCENTAGE, BLECharacteristic::PROPERTY_READ);
    mCharacteristicBatteryVoltage =
        service->createCharacteristic(CHARACTERISTIC_UUID_BATTERY_VOLTAGE, BLECharacteristic::PROPERTY_READ);
    // setTouch(0);
    // setBattery(0);
  }

  void setTime(uint32_t v) {
    // uint32_t vi = v ? 1 : 0;
    mCharacteristicTime->setValue(v);
  }

  void setTouch(bool v, uint32_t raw) {
    uint32_t vi = v ? 1 : 0;
    mCharacteristicTouch->setValue(vi);
    mCharacteristicTouchRaw->setValue(raw);
  }

  void setBattery(const Battery &battery) {
    auto volt = battery.getVoltage();
    auto perc = static_cast<uint32_t>(battery.getPercentage());
    mCharacteristicBatteryVoltage->setValue(volt);
    mCharacteristicBatteryPercentage->setValue(perc);
  }

  bool anyConnections() { return mServer->getPeerDevices(false).size() > 0; }

  void onConnect(BLEServer *pServer) override {
    Serial.println("Connected");
    // pServer->startAdvertising(); // restart advertising
  }

  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override {}

  void onDisconnect(BLEServer *pServer) override {
    Serial.println("Disconnected. Restarting advertisment.");
    pServer->startAdvertising(); // restart advertising
  }

  void onDisconnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override {}
};

PlanchetServer *setupBluetoothServer() {
  Serial.println("Creating bluetooth server...");
  BLEDevice::init("Planchet");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  auto server = new PlanchetServer(pServer, pService);
  pServer->setCallbacks(server); // set the callback function

  Serial.println("Starting bluetooth service...");
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);

  Serial.println("Starting bluetooth advertisement...");
  BLEDevice::startAdvertising();
  Serial.println("BLE Configured...");
  return server;
}
