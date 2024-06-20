#include "battery.h"
#include "ble_client.h"
#include "ble_server.h"
#include "communication.h"
#include "debug.h"
#include "esp_log.h"
#include "math.h"
#include "motion.h"
#include "ota.h"
#include "sensors.h"
#include "time.h"
#include "tmc2209_utils.h"
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMC2209x.h>

HardwareSerial &serial_stream = Serial2;

#define STEP_PIN1 23
#define DIR_PIN1 22
#define STEP_PIN2 19
#define DIR_PIN2 21
#define STEP_PIN3 5
#define DIR_PIN3 18
#define POT1 27
#define POT2 26
#define POT3 25
#define DIAG 14

const uint8_t step_pins[nSteppers] = {STEP_PIN1, STEP_PIN2};
const uint8_t dir_pins[nSteppers] = {DIR_PIN1, DIR_PIN2};

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 1;
const uint8_t RUN_CURRENT_PERCENT = 50;

int32_t lastCurrent = -1;

float lastT = 0;
AnalogSensor p1(POT1, 10);
AnalogSensor p2(POT2, 10);
AnalogSensor p3(POT3, 10);
TouchSensor touch(TOUCH_PAD_NUM3, 400, 800);

int32_t manualSpeed = -1;
int32_t realSpeed = 0;
uint32_t motorsEnabled = ~0U;

Battery battery(AnalogSensor(26, 10));

enum BoardType {
  BOARD_TYPE_PLANCHET,
  BOARD_TYPE_BOARD,
};

const BoardType boardType = BoardType::BOARD_TYPE_BOARD;
RTC_DATA_ATTR uint32_t wakeupCount = 0;

void setupHardware() {
  pinMode(LED_PIN, OUTPUT);
  flash();

  wakeupCount++;
  Serial.begin(SERIAL_BAUD_RATE);
  // adjustMotorPositions();

  if (boardType == BoardType::BOARD_TYPE_BOARD) {
    p1.init();
    p2.init();
    p3.init();

    engine.init();
    for (int i = 0; i < nSteppers; i++) {
      steppers[i] = setupStepper(drivers[i], step_pins[i], dir_pins[i],
                                 static_cast<TMC2209::SerialAddress>(TMC2209::SerialAddress::SERIAL_ADDRESS_0 + i),
                                 serial_stream, RUN_CURRENT_PERCENT);

      if (steppers[i] == NULL) {
        Serial.println("Failed to initialize stepper");
        flash(10, 200);
        ESP.restart();
      }
    }
    setMicrosteps(microsteps);

    for (int i = 0; i < 2; i++) {
      limitSwitches[i].init();
    }
  }
  Serial.println("Setup done");

  // Isolate GPIO12 pin from external circuits. This is needed for modules
  // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
  // to minimize current consumption.
  // TODO: Necessary?
  // rtc_gpio_isolate(GPIO_NUM_12);
}

void monitorBluetooth(void *param) {
  auto client = static_cast<OuijaBoardClient *>(param);
  while (true) {
    client->update();
    delay(50);
  }
}

void setupPlanchette() {
  battery.init();
  touch.init();
  setCpuFrequencyMhz(80);

  auto server = setupBluetoothServer();
  uint32_t i = 0;
  uint32_t lastTouch = 0;
  uint32_t wakeTime = millis();
  uint32_t connectionTime = 0;
  while (true) {
    auto t = millis();
    if ((i % 10) == 0) {
      battery.update();
      server->setBattery(battery);
      if (server->anyConnections()) {
        if (connectionTime == 0) {
          connectionTime = t;
        }
      }
    }
    touch.update();
    if (touch.isTouched()) {
      lastTouch = t;
    }
    // Serial.print("Touch: ");
    // Serial.println(touch.value());
    server->setTouch(touch.isTouched(), touch.value());
    server->setTime(i);

    bool shouldSleep = (lastTouch == 0 || t - lastTouch > MILLIS_PER_SECOND * 60) &&
                       (t - wakeTime > MILLIS_PER_SECOND * 30 ||
                        (connectionTime != 0 && t - connectionTime > MILLIS_PER_SECOND * 100));

    if (shouldSleep) {
      Serial.println("Going to sleep");
      BLEDevice::deinit(false);
      touch.setWakeupFromTouch();
      ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(60 * 60 * static_cast<uint64_t>(MICROS_PER_SECOND)));

      digitalWrite(LED_PIN, LOW);
      delay(50);
      digitalWrite(LED_PIN, HIGH);

      esp_deep_sleep_start();
    }

    delay(100);
    i++;
  }
}

void setupOuijaBoard() {
  home();

  auto client = setupBluetoothClient();
  xTaskCreate(monitorBluetooth, "BLE", 4096, client, 10, nullptr);

  int steps = 100;
  lastT = micros();
  for (int i = 0; i < 20 * steps; i++) {
    while (!client->touchActive) {
      for (int j = 0; j < nSteppers; j++) {
        steppers[j]->stopMove();
      }
      delay(20);
    }

    auto time = micros();
    auto dt = (time - lastT) * 0.000001f;
    lastT = time;

    float t = i / static_cast<float>(steps);
    float c[2];
    memcpy(c, boardCenter, sizeof(boardCenter));
    c[0] += 60 * cos(t * (float)TWO_PI);
    c[1] += 28 * sin(t * (float)TWO_PI);
    int32_t target[3];
    positionToMotor(c, target);
    moveToPosition(target, 40);
    auto t0 = millis();
    if (!blockUntilAtTarget(30 * static_cast<int32_t>(microsteps), target, [&] { return client->touchActive; })) {
      i--;
      continue;
    }
    auto t1 = millis();
    Serial.print("Move took ");
    Serial.println(t1 - t0);
  }
  // while (true) {
  //   Serial.println("Updating");
  //   client->update();
  //   delay(100);

  //   client->touchActive
  // }
}

void setup() {
  setupHardware();

  // Start OTA on first wakeup only
  if (wakeupCount == 1) {
    setupOTA();
  }

  if (boardType == BoardType::BOARD_TYPE_PLANCHET) {
    setupPlanchette();
  } else {
    setupOuijaBoard();
  }
  return;

  for (int i = 0; i < 100; i++) {
    home();

    Serial.println("Randomizing position");
    float f0 = (rand() % 10) * 0.1f;
    float f1 = (rand() % 10) * 0.1f;
    float f2 = 1.0f - f1 - f2;
    float target[2] = {
        motorCoords[0][0] * f0 + motorCoords[1][0] * f1 + motorCoords[2][0] * f2,
        motorCoords[0][1] * f0 + motorCoords[1][1] * f1 + motorCoords[2][1] * f2,
    };
    int32_t target2[3];
    positionToMotor(target, target2);
    moveToPosition(target2, 80);
    delay(100);
    blockUntilNotMoving();

    Serial.println("Extending steppers");
    // Extend steppers randomly
    for (int j = 0; j < nSteppers; j++) {
      steppers[j]->move(distanceToMicrosteps((rand() % 20) * 0.1f));
    }
    blockUntilNotMoving();
  }
  Serial.println("Finished");
  delay(10000000);

  // calibrate();
  lastT = micros();
}

bool motorCommands(String cmd) {
  int32_t intValue;
  if (cmd == "stop") {
    manualSpeed = 0;
    return true;
  } else if (cmd == "auto") {
    manualSpeed = -1;
    return true;
  } else if (cmd == "status") {
    for (int i = 0; i < nSteppers; i++) {
      Serial.print("\n\nStatus for driver: ");
      Serial.println(i);
      printDriverStatus(drivers[i]);
    }
    Serial.println();
    return true;
  } else if (cmd == "speed") {
    Serial.print("Real speed ");
    Serial.println(realSpeed);
    return true;
  } else if (sscanf(cmd.c_str(), "speed %d", &intValue) == 1) {
    manualSpeed = intValue;
    return true;
  } else if (sscanf(cmd.c_str(), "enabled %d", &intValue) == 1) {
    motorsEnabled = static_cast<uint32_t>(intValue);
    return true;
  } else if (sscanf(cmd.c_str(), "microsteps %d", &intValue) == 1) {
    setMicrosteps(static_cast<uint32_t>(intValue));
    return true;
  } else {
    return false;
  }
}

bool executeCommand(String cmd) {
  String suffix;
  if (split(cmd, "motors ", suffix)) {
    return motorCommands(suffix);
  }
  return false;
}
void readCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (!executeCommand(cmd)) {
      Serial.print("Unknown command '");
      Serial.print(cmd);
      Serial.println("'");
    }
  }
}

void loop() {
  delay(100);
  return;

  readCommands();
  auto t = micros();
  auto dt = (t - lastT) * 0.000001f;

  p1.update();
  p2.update();
  p3.update();

  realSpeed = manualSpeed != -1 ? manualSpeed : static_cast<int32_t>(p1.value() * 500);
  auto current = 50; //(int32_t)(p2 * 100);

  if (abs(current - lastCurrent) > 3) {
    lastCurrent = current;
    for (int i = 0; i < nSteppers; i++) {
      drivers[i].setRunCurrent(current);
    }
  }

  auto targetPos = sin(t * 0.001f) * 200 + sin(t * 0.01f) * 20 + sin(t * 0.01242f) * 30;

  if (realSpeed <= 0) {
    for (int i = 0; i < nSteppers; i++)
      steppers[i]->stopMove();
  } else {
    for (int32_t i = 0; i < nSteppers; i++) {
      if (motorsEnabled & (1U << i)) {
        steppers[i]->setSpeedInHz(static_cast<uint32_t>(realSpeed) * microsteps);
        // stepper1->moveTo(targetPos * microsteps, false);
        steppers[i]->runForward();
      } else {
        steppers[i]->stopMove();
      }
    }
  }

  for (int i = 0; i < nSteppers; i++) {
    // int32_t currentSpeed = steppers[i]->getCurrentSpeedInMilliHz() / (1000 *
    // microsteps); float stall = (float)driver.getStallGuardResult() /
    // (float)getStallBase(currentSpeed); if (currentSpeed > (speed/2) && stall
    // < 0.8) {
    //   stepper1->stopMove();
    //   delay(1000);
    // }
  }

  // printDriverStatus(driver);
  if (edge(lastT, t, 50)) {
    // Serial.print("Speed:");
    // Serial.print(speed);
    // Serial.print(",Stall:");
    // Serial.print((float)driver.getStallGuardResult() /
    // (float)getStallBase(speed)); Serial.print(",target:");
    // Serial.print(targetPos);
    Serial.print(t - lastT);
    Serial.print(",pos:");
    Serial.print((steppers[0]->getCurrentPosition() / static_cast<int32_t>(microsteps)));
    Serial.print(",dt:");
    Serial.println(dt * 1000.0f);
  }

  lastT = t;
  delay(DELAY);
}
