#pragma once
#include "tmc2209_utils.h"
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMC2209x.h>

// Instantiate TMC2209
TMC2209 drivers[3];
const int nSteppers = 2;
FastAccelStepper *steppers[nSteppers];
FastAccelStepperEngine engine = FastAccelStepperEngine();

int32_t homingPositions[3];

const uint32_t stepsPerRevolution = 200;
const float shaftDiameter = 14.2f;
const float tensioningFactor = 1.0f;
const float tensioningOffset = 0.0f;

float motorCoords[nSteppers][2] = {
    {0, 0},
    {0, 0},
};

float motorProjection[nSteppers][2] = {
    {1, 0},
    {0, 1},
};

float boardCenter[2] = {70, 28};

int32_t distanceToMicrosteps(float d) {
  return (int32_t)(d * microsteps * stepsPerRevolution / ((float)PI * shaftDiameter));
}

void setMicrosteps(uint32_t v) {
  microsteps = v;
  for (int i = 0; i < nSteppers; i++) {
    steppers[i]->setAcceleration(500000 * microsteps);
    drivers[i].setMicrostepsPerStep(microsteps);
  }
}

float stallFraction(int motor) {
  return (float)drivers[motor].getStallGuardResult() /
         (float)getStallBase(steppers[motor]->getSpeedInMilliHz() / (1000 * microsteps));
}

bool isStalling(int motor) {
  Serial.print("Stall");
  Serial.print(motor);
  Serial.print(":");
  auto x = stallFraction(motor);
  Serial.print(x);
  Serial.println();
  return x < 0.8f;
}

void blockUntilNotMoving() {
  for (int i = 0; i < nSteppers; i++) {
    while (steppers[i]->isRunning())
      delay(1);
  }
}

void tension() {
  int speed = 50;
  for (int i = 0; i < nSteppers; i++) {
    drivers[i].setStandstillMode(TMC2209::StandstillMode::NORMAL);
    drivers[i].setHoldCurrent(50);
    drivers[i].setRunCurrent(10);
    steppers[i]->setSpeedInHz(microsteps * speed);
  }
  for (int i = 0; i < nSteppers; i++) {
    steppers[i]->runBackward();
  }
  delay(50);
  int stalled = 0;
  for (int it = 0; stalled != 0b111; it++) {
    // if ((it % 50) == 0) {
    //   for (int i = 0; i < nSteppers; i++) {
    //     Serial.print("S");
    //     Serial.print(i);
    //     Serial.print(":");
    //     Serial.println(stallFraction(i));
    //   }
    // }
    // float stallBuffer[nSteppers] = { 0.0f, 0.0f, 0.0f };
    // for (int t = 0; t < 4; t++) {
    //   for (int i = 0; i < nSteppers; i++) {
    //     stallBuffer[i] += stallFraction(i);
    //   }
    // }
    // for (int i = 0; i < nSteppers; i++) stallBuffer[i] *= 0.25f;

    Serial.print("Stall");
    for (int i = 0; i < nSteppers; i++) {
      auto b = stallFraction(i);
      Serial.print(":");
      Serial.print(b);
      if ((stalled & (1 << i)) == 0 && b < 0.8f) {
        stalled |= 1 << i;
        Serial.println("Stalling");
        steppers[i]->stopMove();
      }
    }
    Serial.println();
    delay(1);
  }
}

void positionToMotor(const float coords[2], int32_t motorPositions[nSteppers]) {
  Serial.print("G: ");
  for (int i = 0; i < nSteppers; i++) {
    float dist = 0;
    for (int j = 0; j < 2; j++) {
      auto d = (coords[j] - motorCoords[i][j]) * motorProjection[i][j];
      dist += d * d;
    }
    dist = sqrt(dist);
    Serial.print(dist);
    Serial.print(", ");
    motorPositions[i] = homingPositions[i] + distanceToMicrosteps(dist * tensioningFactor - tensioningOffset);
  }
  Serial.println();
}

void moveToPosition(const int32_t motorPositions[nSteppers], int32_t speed, int8_t motorMask = ~0) {
  if (motorMask == 0) {
    Serial.println("Skipping move because mask is empty");
    return;
  }
  int32_t maxDelta = 0;
  for (int i = 0; i < nSteppers; i++) {
    if ((motorMask & (1 << i)) == 0)
      continue;
    auto delta = abs(motorPositions[i] - steppers[i]->getCurrentPosition());
    maxDelta = max(maxDelta, delta);
  }
  Serial.print("Moving to: ");
  for (int i = 0; i < nSteppers; i++) {
    if ((motorMask & (1 << i)) == 0)
      continue;

    auto delta = abs(motorPositions[i] - steppers[i]->getCurrentPosition());
    auto motorSpeed = max((uint32_t)microsteps, (speed * delta * microsteps) / maxDelta);
    steppers[i]->setSpeedInHz(motorSpeed);
    // steppers[i]->stopMove();
    auto res = steppers[i]->moveTo(motorPositions[i]);
    if (res != MOVE_OK) {
      Serial.print("Move failed: ");
      Serial.println(res);
      delay(1000);
    }
    Serial.print(motorPositions[i]);
    Serial.print(" speed=");
    Serial.print(motorSpeed);
    Serial.print(", ");
  }
  Serial.println();
}

void blockUntilAtTarget(int32_t maxMicrosteps, int32_t target[nSteppers]) {
  bool any = true;
  while (any) {
    any = false;
    for (int i = 0; i < nSteppers; i++) {
      while (abs(steppers[i]->getCurrentPosition() - target[i]) > maxMicrosteps) {
        any = true;
        // Serial.print("E");
        // Serial.print(i);
        // Serial.print(":");
        // Serial.println(abs(steppers[i]->getCurrentPosition() - target[i]));
        delay(10);
      }
    }
  }
}

void blockUntilAlmostNotMoving(int32_t maxMicrosteps) {
  // Fast accel stepper doesn't update the target pos immediately it seems like
  delay(1);

  bool any = true;
  while (any) {
    any = false;
    for (int i = 0; i < nSteppers; i++) {
      while (abs(steppers[i]->getCurrentPosition() - steppers[i]->targetPos()) > maxMicrosteps) {
        any = true;
        delay(1);
      }
    }
  }
}

void homeStep(int speed, float stallThresholds[nSteppers]) {
  for (int i = 0; i < nSteppers; i++) {
    drivers[i].setStandstillMode(TMC2209::StandstillMode::NORMAL);
    drivers[i].setHoldCurrent(30);
    drivers[i].setRunCurrent(30);
    steppers[i]->setSpeedInHz(microsteps * speed);
    steppers[i]->runBackward();
  }
  delay(200);
  {
    int stalled = 0;
    for (int it = 0; stalled != 0b11 && it < 20000 / (3 * 4 * 2); it++) {
      float b[nSteppers] = {0, 0};
      for (int t = 0; t < 3 * 4; t++) {
        for (int i = 0; i < nSteppers; i++) {
          b[i] += stallFraction(i);
        }
        delay(2);
      }
      for (int i = 0; i < nSteppers; i++) {
        b[i] /= 3 * 4;
      }
      Serial.print("S0:");
      Serial.print(b[0]);
      Serial.print(" S1:");
      Serial.println(b[1]);
      for (int i = 0; i < nSteppers; i++) {
        if (b[i] < stallThresholds[i]) {
          stalled |= 1 << i;
          steppers[i]->forceStopAndNewPosition(0);
          homingPositions[i] = 0;
        }
      }
    }
  }
}

void homeMotors() {
  float t1[nSteppers] = {1.0f, 1.1f};
  homeStep(80, t1);
  for (int i = 0; i < nSteppers; i++) {
    steppers[i]->move(distanceToMicrosteps(10), true);
  }

  Serial.println("Step 2");
  float t2[nSteppers] = {1.0f, 1.1f};
  homeStep(40, t2);
  for (int i = 0; i < nSteppers; i++) {
    steppers[i]->move(distanceToMicrosteps(5), true);
  }

  // for (int k = 0; k < 100; k++) {
  //   for (int i = 0; i < nSteppers; i++) {
  //     auto d = ((k % 2) == 0 ? 1 : -1) * distanceToMicrosteps(20);
  //     Serial.println(d);
  //     steppers[i]->move(d, false);
  //   }
  //   delay(50);
  //   blockUntilNotMoving();
  // }

  for (int i = 0; i < nSteppers; i++) {
    drivers[i].setStandstillMode(TMC2209::StandstillMode::NORMAL);
    drivers[i].setHoldCurrent(50);
    drivers[i].setRunCurrent(50);
  }
  int32_t target[3];
  positionToMotor(boardCenter, target);
  moveToPosition(target, 160);
  blockUntilNotMoving();

  int steps = 100;
  auto lastT = micros();
  for (int i = 0; i < 20 * steps; i++) {
    auto time = micros();
    auto dt = (time - lastT) * 0.000001f;
    lastT = time;

    float t = i / (float)steps;
    float c[2];
    memcpy(c, boardCenter, sizeof(boardCenter));
    c[0] += 60 * cos(t * (float)TWO_PI);
    c[1] += 28 * sin(t * (float)TWO_PI);
    positionToMotor(c, target);
    moveToPosition(target, 40);
    auto t0 = millis();
    blockUntilAtTarget(30 * microsteps, target);
    auto t1 = millis();
    Serial.print("Move took ");
    Serial.println(t1 - t0);
  }
}

void home() {
  Serial.println("Homing...");
  homeMotors();
  // while(true) {
  //   Serial.println("Tensioning...");
  //   tension();
  //   steppers[0]->setSpeedInHz(microsteps * 400);
  //   steppers[0]->move(200);
  // }
  // delay(2000000);
}

void adjustMotorPositions() {
  for (int i = 0; i < nSteppers; i++) {
    float dir[2] = {boardCenter[0] - motorCoords[i][0], boardCenter[1] - motorCoords[i][1]};
    float len = sqrt(dir[0] * dir[0] + dir[1] * dir[1]);
    dir[0] /= len;
    dir[1] /= len;
    for (int j = 0; j < 2; j++) {
      motorCoords[i][j] += dir[j] * 13.5f;
    }

    Serial.print("Motor coordinate ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(motorCoords[i][0]);
    Serial.print(", ");
    Serial.print(motorCoords[i][1]);
    Serial.println();
  }
}

FastAccelStepper *setupStepper(TMC2209 &driver, uint32_t step_pin, uint32_t dir_pin, TMC2209::SerialAddress address,
                               HardwareSerial &serial_stream, uint8_t run_current_percent) {
  Serial.print("Setting up stepper at address ");
  Serial.println(address);
  driver.setup(serial_stream, 500000, address);
  driver.setReplyDelay(4);

  // stepper_driver.setStallGuardThreshold(STALL_GUARD_THRESHOLD);
  driver.enableStealthChop();
  // driver.enableAutomaticGradientAdaptation();
  driver.enableAutomaticCurrentScaling();
  driver.disableAnalogCurrentScaling();
  // driver.disableCoolStep();
  driver.moveUsingStepDirInterface();
  driver.disableAutomaticGradientAdaptation();
  // driver.disableAutomaticCurrentScaling();
  // driver.setPwmGradient(16);
  // driver.setPwmOffset(30);

  driver.setRunCurrent(run_current_percent);
  driver.setMicrostepsPerStep(microsteps);
  printDriverStatus(driver);

  driver.enable();

  auto stepper = engine.stepperConnectToPin(step_pin, DRIVER_MCPWM_PCNT);
  stepper->setDirectionPin(dir_pin);

  if (stepper == NULL) {
    Serial.println("Failed to setup stepper");
    return NULL;
  }

  return stepper;
}
