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
TMC2209 driver1;
TMC2209 driver2;
TMC2209 driver3;

FastAccelStepperEngine engine = FastAccelStepperEngine();
const int nSteppers = 3;
FastAccelStepper* steppers[nSteppers];


int32_t lastCurrent = -1;

float p1 = 0;
float p2 = 0;
float p3 = 0;
float lastT = 0;

uint32_t manualSpeed = -1;
uint32_t microsteps = 16;

void setMicrosteps(uint32_t v) {
  microsteps = v;
  for (int i = 0; i < nSteppers; i++) steppers[i]->setAcceleration(2000*microsteps);
  driver1.setMicrostepsPerStep(microsteps);
  driver2.setMicrostepsPerStep(microsteps);
  driver3.setMicrostepsPerStep(microsteps);
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

  engine.init();
  auto stepper = engine.stepperConnectToPin(step_pin, DRIVER_RMT);
  stepper->setDirectionPin(dir_pin);

  if (stepper == NULL) {
    Serial.println("Failed to setup stepper");
    return NULL;
  }

  driver.setMicrostepsPerStep(microsteps);
  return stepper;
}

void setup()
{
  pinMode(POT1, INPUT);
  pinMode(POT2, INPUT);
  pinMode(POT3, INPUT);
  Serial.begin(SERIAL_BAUD_RATE);

  steppers[0] = setupStepper(driver1, STEP_PIN1, DIR_PIN1, TMC2209::SerialAddress::SERIAL_ADDRESS_0);
  steppers[1] = setupStepper(driver2, STEP_PIN2, DIR_PIN2, TMC2209::SerialAddress::SERIAL_ADDRESS_1);
  steppers[2] = setupStepper(driver3, STEP_PIN3, DIR_PIN3, TMC2209::SerialAddress::SERIAL_ADDRESS_2);

  setMicrosteps(microsteps);

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
    printDriverStatus(driver1);
    printDriverStatus(driver2);
    printDriverStatus(driver3);
    return true;
  } else if (sscanf(cmd.c_str(), "speed %d", &intValue) == 1) {
    manualSpeed = intValue;
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
  auto speed = manualSpeed != -1 ? manualSpeed : (int32_t)(p1 * 500);
  auto current = 50; //(int32_t)(p2 * 100);

  if (abs(current - lastCurrent) > 3) {
    lastCurrent = current;
    driver1.setRunCurrent(current);
    driver2.setRunCurrent(current);
    driver3.setRunCurrent(current);
  }

  auto targetPos = sin(t*0.001f) * 200 + sin(t*0.01f) * 20 + sin(t*0.01242f) * 30;
  
  if (speed == 0) {
    for (int i = 0; i < nSteppers; i++) steppers[i]->stopMove();
  } else {
    for (int i = 0; i < nSteppers; i++) {
      steppers[i]->setSpeedInHz(speed * microsteps);
      // stepper1->moveTo(targetPos * microsteps, false);
      steppers[i]->runForward();
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
