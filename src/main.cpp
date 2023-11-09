#include "communication.h"
#include "math.h"
#include "motion.h"
#include "sensors.h"
#include "tmc2209_utils.h"
#include <FastAccelStepper.h>
#include <TMC2209x.h>

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

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

uint32_t manualSpeed = -1;
uint32_t realSpeed = 0;
uint32_t motorsEnabled = ~0;

void hardwareSetup() {
  p1.init();
  p2.init();
  p3.init();
  Serial.begin(SERIAL_BAUD_RATE);
  // adjustMotorPositions();

  engine.init();
  for (int i = 0; i < nSteppers; i++) {
    steppers[i] = setupStepper(drivers[i], step_pins[i], dir_pins[i],
                               (TMC2209::SerialAddress)(TMC2209::SerialAddress::SERIAL_ADDRESS_0 + i), serial_stream,
                               RUN_CURRENT_PERCENT);
  }
  setMicrosteps(microsteps);
  Serial.println("Setup done");
}

void setup() {
  hardwareSetup();

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
    for (int i = 0; i < nSteppers; i++) {
      steppers[i]->move(distanceToMicrosteps((rand() % 20) * 0.1f));
    }
    blockUntilNotMoving();
  }
  Serial.println("Finished");
  delay(10000000);

  // calibrate();
  lastT = micros();
}

bool motorCommands(String cmd) {
  int intValue;
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
    motorsEnabled = intValue;
    return true;
  } else if (sscanf(cmd.c_str(), "microsteps %d", &intValue) == 1) {
    setMicrosteps(intValue);
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

bool edge(unsigned long lastT, unsigned long t, unsigned long interval) {
  interval *= 1000;
  return (lastT / interval) != (t / interval);
}

void loop() {
  readCommands();
  auto t = micros();
  auto dt = (t - lastT) * 0.000001f;

  p1.update();
  p2.update();
  p3.update();

  realSpeed = manualSpeed != -1 ? manualSpeed : (int32_t)(p1.value * 500);
  auto current = 50; //(int32_t)(p2 * 100);

  if (abs(current - lastCurrent) > 3) {
    lastCurrent = current;
    for (int i = 0; i < nSteppers; i++) {
      drivers[i].setRunCurrent(current);
    }
  }

  auto targetPos = sin(t * 0.001f) * 200 + sin(t * 0.01f) * 20 + sin(t * 0.01242f) * 30;

  if (realSpeed == 0) {
    for (int i = 0; i < nSteppers; i++)
      steppers[i]->stopMove();
  } else {
    for (int i = 0; i < nSteppers; i++) {
      if (motorsEnabled & (1 << i)) {
        steppers[i]->setSpeedInHz(realSpeed * microsteps);
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
    Serial.print((steppers[0]->getCurrentPosition() / (int32_t)microsteps));
    Serial.print(",dt:");
    Serial.println(dt * 1000.0f);
  }

  lastT = t;
  delay(DELAY);
}
