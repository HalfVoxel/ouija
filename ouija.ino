#include "TMC2209x.h"
#include "FastAccelStepper.h"
#include "tmc2209_utils.h"
// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

HardwareSerial & serial_stream = Serial2;

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

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 1;
// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 50;
const int32_t VELOCITY = 10000;
const uint8_t STALL_GUARD_THRESHOLD = 50;


// Instantiate TMC2209
TMC2209 drivers[3];

FastAccelStepperEngine engine = FastAccelStepperEngine();
const int nSteppers = 3;
FastAccelStepper* steppers[nSteppers];


int32_t lastCurrent = -1;

float p1 = 0;
float p2 = 0;
float p3 = 0;
float lastT = 0;

uint32_t manualSpeed = -1;
uint32_t realSpeed = 0;
uint32_t microsteps = 16;
uint32_t motorsEnabled = ~0;

void setMicrosteps(uint32_t v) {
  microsteps = v;
  for (int i = 0; i < nSteppers; i++) {
    steppers[i]->setAcceleration(500000*microsteps);
    drivers[i].setMicrostepsPerStep(microsteps);
  }
}

FastAccelStepper* setupStepper(TMC2209& driver, uint32_t step_pin, uint32_t dir_pin, TMC2209::SerialAddress address) {
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

  driver.setRunCurrent(RUN_CURRENT_PERCENT);
  printDriverStatus(driver);

  driver.enable();

  auto stepper = engine.stepperConnectToPin(step_pin, DRIVER_MCPWM_PCNT);
  stepper->setDirectionPin(dir_pin);

  if (stepper == NULL) {
    Serial.println("Failed to setup stepper");
    return NULL;
  }

  driver.setMicrostepsPerStep(microsteps);
  return stepper;
}

float stallFraction(int motor) {
  return (float)drivers[motor].getStallGuardResult() / (float)getStallBase(steppers[motor]->getSpeedInMilliHz()/(1000*microsteps));
}
bool isStalling(int motor) {
  return stallFraction(motor) < 0.8f;
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
  for(int it = 0;stalled != 0b111; it++) {
    if ((it % 50) == 0) {
      for (int i = 0; i < nSteppers; i++) {
        Serial.print("S");
        Serial.print(i);
        Serial.print(":");
        Serial.println(stallFraction(i));
      }
    }
    for (int i = 0; i < nSteppers; i++) {
      if ((stalled & (1<<i)) == 0 && isStalling(i)) {
        stalled |= 1 << i;
        Serial.println("Stalling");
        steppers[i]->stopMove();
      }
    }
    delay(1);
  }
}

int32_t homingPositions[3];

const uint32_t stepsPerRevolution = 200;
const float shaftDiameter = 6.0f;
const float triangleSide = 130.0f;
const float toCenterDist = 70.0f;
const float tensioningFactor = 1.0f;
const float tensioningOffset = 45.0f;

const int32_t motorCoords[nSteppers][2] = {
  { 0, 0 },
  { 185, 0},
  { 185/2, 160 }
};

const float triangleCenter[2] = {
  (0 + 185 + 185/2) * 0.333333f,
  (0 + 0 + 160) * 0.333333f,
};

void homeMotors() {
  int speed = 80;

  for (int motorToHome = 0; motorToHome < nSteppers; motorToHome++) {
    for (int i = 0; i < nSteppers; i++) {
      drivers[i].setStandstillMode(i <= motorToHome ? TMC2209::StandstillMode::NORMAL : TMC2209::StandstillMode::FREEWHEELING);
      drivers[i].setHoldCurrent(i <= motorToHome ? 50 : 0);
      drivers[i].setRunCurrent(10);
      steppers[i]->setSpeedInHz(microsteps * speed);
    }
    for (int i = 0; i < nSteppers; i++) {
      steppers[i]->stopMove();
    }
    for (int i = 0; i < motorToHome; i++) {
      if (i == motorToHome - 1) {
        const uint32_t stepsPerRevolution = 200;
        const float shaftDiameter = 6.0f;
        const float triangleSide = 130.0f;
        steppers[i]->move((int32_t)(triangleSide / (shaftDiameter * (float)PI) * stepsPerRevolution * microsteps));
      }
    }
    steppers[motorToHome]->runBackward();

    delay(50);
    int stalled = 0;
    for(int it = 0;; it++) {
      // if ((it % 50) == 0) {
      //   for (int i = 0; i < nSteppers; i++) {
      //     if (i == motorToHome) {
      //       Serial.print("S");
      //       Serial.print(i);
      //       Serial.print(":");
      //       Serial.println(stallFraction(i));
      //     }
      //   }
      // }
      if (isStalling(motorToHome)) {
        Serial.println("Stalling");
        homingPositions[motorToHome] = steppers[motorToHome]->getCurrentPosition();
        Serial.print("Homed at: ");
        Serial.println(homingPositions[motorToHome]);
        steppers[motorToHome]->forceStop();
        break;
      }
      delay(1);
    }
  }

  for (int i = 0; i < nSteppers; i++) {
    drivers[i].setStandstillMode(TMC2209::StandstillMode::NORMAL);
    drivers[i].setHoldCurrent(50);
    drivers[i].setRunCurrent(50);
  }
  int32_t target[3];
  positionToMotor(triangleCenter, target);
  moveToPosition(target, 80);
  blockUntilNotMoving();

  int steps = 100;
  for (int i = 0; i < 6*steps; i++) {
    float t = i / (float)steps;
    float c[2];
    memcpy(c, triangleCenter, sizeof(triangleCenter));
    c[0] += 40 * cos(t * (float)TWO_PI);
    c[1] += 40 * sin(t * (float)TWO_PI);
    positionToMotor(c, target);
    moveToPosition(target, 80);
    auto t0 = millis();
    blockUntilAtTarget(30 * microsteps, target);
    auto t1 = millis();
    Serial.print("Move took ");
    Serial.println(t1 - t0);
  }
}

void blockUntilNotMoving() {
  for (int i = 0; i < nSteppers;i++) {
    while (steppers[i]->isRunning()) delay(1);
  }
}

void blockUntilAtTarget(int32_t maxMicrosteps, int32_t target[nSteppers]) {
  bool any = true;
  while(any) {
    any = false;
    for (int i = 0; i < nSteppers;i++) {
      while (abs(steppers[i]->getCurrentPosition() - target[i]) > maxMicrosteps) {
        any = true;
        Serial.print("E");
        Serial.print(i);
        Serial.print(":");
        Serial.println(abs(steppers[i]->getCurrentPosition() - target[i]));
        delay(10);
      }
    }
  }
}

void blockUntilAlmostNotMoving(int32_t maxMicrosteps) {
  // Fast accel stepper doesn't update the target pos immediately it seems like
  delay(1);

  bool any = true;
  while(any) {
    any = false;
    for (int i = 0; i < nSteppers;i++) {
      while (abs(steppers[i]->getCurrentPosition() - steppers[i]->targetPos()) > maxMicrosteps) {
        any = true;
        delay(1);
      }
    }
  }
}

void moveToPosition(const int32_t motorPositions[nSteppers], int32_t speed) {
  int32_t maxDelta = 0;
  for (int i = 0; i < nSteppers;i++) {
    auto delta = abs(motorPositions[i] - steppers[i]->getCurrentPosition());
    maxDelta = max(maxDelta, delta);
  }
  Serial.print("Moving to: ");
  for (int i = 0; i < nSteppers;i++) {
    auto delta = abs(motorPositions[i] - steppers[i]->getCurrentPosition());
    auto motorSpeed = max((uint32_t)1, (speed * delta * microsteps) / maxDelta);
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

void positionToMotor(const float coords[2], int32_t motorPositions[nSteppers]) {
  Serial.print("G: ");
  for (int i = 0; i < nSteppers; i++) {
    float dist = 0;
    for (int j = 0; j < 2; j++) {
      auto d = coords[j] - motorCoords[i][j];
      dist += d*d;
    }
    dist = sqrt(dist);
    Serial.print(dist);
    Serial.print(", ");
    motorPositions[i] = homingPositions[i] + (int32_t)((dist - tensioningOffset) / (shaftDiameter * (float)PI) * stepsPerRevolution * microsteps * tensioningFactor);
  }
  Serial.println();
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
  delay(2000000);
}

void setup()
{
  pinMode(POT1, INPUT);
  pinMode(POT2, INPUT);
  pinMode(POT3, INPUT);
  Serial.begin(SERIAL_BAUD_RATE);

  engine.init();
  steppers[0] = setupStepper(drivers[0], STEP_PIN1, DIR_PIN1, TMC2209::SerialAddress::SERIAL_ADDRESS_0);
  steppers[1] = setupStepper(drivers[1], STEP_PIN2, DIR_PIN2, TMC2209::SerialAddress::SERIAL_ADDRESS_1);
  steppers[2] = setupStepper(drivers[2], STEP_PIN3, DIR_PIN3, TMC2209::SerialAddress::SERIAL_ADDRESS_2);
  Serial.println("Setup done");

  setMicrosteps(microsteps);

  home();

  // calibrate();
  lastT = micros();
}

const uint32_t STALL_BASE_STRIDE_HZ = 10;
int32_t stallBase[] = {
  10,
  38,
  80,
  133,
  187,
  239,
  297,
  341,
  381,
  384,
};

float lerp(float a, float b, float t) {
  return a + (b-a)*min(1.0f, max(0.0f, t));
}

int32_t getStallBase(int32_t speedHz) {
  int maxIndex = sizeof(stallBase)/sizeof(*stallBase);
  float t = sqrt(speedHz/(float)STALL_BASE_STRIDE_HZ) - 1;
  int index1 = (int)t;
  int index2 = index1 + 1;
  float interp = t - index1;
  index1 = max(min(index1, maxIndex - 1), 0);
  index2 = max(min(index2, maxIndex - 1), 0);
  return lerp(stallBase[index1], stallBase[index2], interp);
}

float readAndSmooth(uint8_t pin, float& value, float alpha) {
  auto newValue = analogRead(pin) / 4096.0f;
  value = lerp(value, newValue, alpha);
  return value;
}

void calibrate(FastAccelStepper* stepper, TMC2209& driver) {
  for (int i = 1; i <= 10; i++) {
    int32_t speed = i * i * STALL_BASE_STRIDE_HZ;
    stepper->setSpeedInHz(speed * microsteps);
    stepper->runForward();
    delay(10);
    int32_t stall = 0;
    const size_t SAMPLES = 100;
    for (int j = 0; j < SAMPLES; j++) {
      delay(10);
      stall += driver.getStallGuardResult();
    }
    stall /= SAMPLES;
    Serial.print("Stall base for ");
    Serial.print(speed);
    Serial.print(" ");
    Serial.println(stall);
    stallBase[i-1] = stall;
  }
  for (int i = 1; i <= 10; i++) {
    int32_t speed = i * i * STALL_BASE_STRIDE_HZ;
    Serial.print("Stall check: ");
    Serial.print(stallBase[i-1]);
    Serial.print(" = ");
    Serial.println(getStallBase(speed));
  }
  delay(2000);
}

bool split(const String& cmd, const String& prefix, String& suffix) {
  if (cmd.startsWith(prefix)) {
    suffix = cmd.substring(prefix.length());
    return true;
  }
  return false;
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
  return (lastT/interval) != (t/interval);
}

void loop()
{
  readCommands();
  auto t = micros();
  auto dt = (t - lastT) * 0.000001f;

  readAndSmooth(POT1, p1, 10*dt);
  readAndSmooth(POT2, p2, 10*dt);
  readAndSmooth(POT3, p3, 10*dt);
  realSpeed = manualSpeed != -1 ? manualSpeed : (int32_t)(p1 * 500);
  auto current = 50; //(int32_t)(p2 * 100);

  if (abs(current - lastCurrent) > 3) {
    lastCurrent = current;
    for (int i = 0; i < nSteppers; i++) {
      drivers[i].setRunCurrent(current);
    }
  }

  auto targetPos = sin(t*0.001f) * 200 + sin(t*0.01f) * 20 + sin(t*0.01242f) * 30;
  
  if (realSpeed == 0) {
    for (int i = 0; i < nSteppers; i++) steppers[i]->stopMove();
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
    // int32_t currentSpeed = steppers[i]->getCurrentSpeedInMilliHz() / (1000 * microsteps);
    // float stall = (float)driver.getStallGuardResult() / (float)getStallBase(currentSpeed);
    // if (currentSpeed > (speed/2) && stall < 0.8) {
    //   stepper1->stopMove();
    //   delay(1000);
    // }
  }

  // printDriverStatus(driver);
  if (edge(lastT, t, 50)) {
    // Serial.print("Speed:");
    // Serial.print(speed);
    // Serial.print(",Stall:");
    // Serial.print((float)driver.getStallGuardResult() / (float)getStallBase(speed));
    // Serial.print(",target:");
    // Serial.print(targetPos);
    Serial.print(t - lastT);
    Serial.print(",pos:");
    Serial.print((steppers[0]->getCurrentPosition()/(int32_t)microsteps));
    Serial.print(",dt:");
    Serial.println(dt*1000.0f);
  }

  lastT = t;
  delay(DELAY);
}
