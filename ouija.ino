#define TMC2130DEBUG
#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <SPI.h>
//#include <AccelStepper.h>
#include "FastAccelStepper.h"
#include "AVRStepperPins.h"
#include <TMC2209.h>

#define R_SENSE 0.11f  // Match to your driver \
                       // SilentStepStick series use 0.11 \
                       // UltiMachine Einsy and Archim2 boards use 0.2 \
                       // Panucatt BSD2660 uses 0.1 \
                       // Watterott TMC5160 uses 0.075

#define CS_PIN1 32
#define CS_PIN2 22
#define EN_PIN 27
#define STEP_PIN1 4
#define STEP_PIN2 15
#define DIR_PIN1 33
#define DIR_PIN2 21
#define LED 2
// #define MOSI_PIN 23
// #define MISO_PIN 19
// #define SCK_PIN 18

#define STEP_PIN3 13
#define DIR_PIN3 12
#define UART_TX3 17
#define UART_RX3 16

TMC2130Stepper driver1(CS_PIN1, R_SENSE);  //, MOSI_PIN, MISO_PIN, SCK_PIN);
TMC2130Stepper driver2(CS_PIN2, R_SENSE);

constexpr uint32_t steps_per_mm = 80;

// AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

// SPIClass SPI1(VSPI);
#define SUBSTEPS 64
HardwareSerial &serial_stream = Serial2;
TMC2209 driver3;


void tension(uint32_t current, uint32_t speed, uint32_t steps) {
  driver1.rms_current(current);  // Set motor RMS current
  driver2.rms_current(current);
  stepper1->setSpeedInHz(speed * SUBSTEPS);
  stepper2->setSpeedInHz(speed * SUBSTEPS);
  stepper1->move(steps * SUBSTEPS, false);
  stepper2->move(-steps * SUBSTEPS, true);
}

void printDriverStatus(TMC2130Stepper &driver) {
  // const uart_port_t uart_num = UART_NUM_2;
  // uart_config_t uart_config = {
  //     .baud_rate = 115200,
  //     .data_bits = UART_DATA_8_BITS,
  //     .parity = UART_PARITY_DISABLE,
  //     .stop_bits = UART_STOP_BITS_1,
  //     .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
  //     .rx_flow_ctrl_thresh = 122,
  // };
  // // Configure UART parameters
  // ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  // ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_TX3, UART_RX3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  auto status = driver.DRV_STATUS();

  bool fullStepActive = (status & (1 << 15)) != 0;
  int32_t current = (status >> 16) & ((1 << 5) - 1);
  bool overtemperature = (status & (1 << 25)) != 0;
  bool overtemperaturePre = (status & (1 << 26)) != 0;
  bool shortToGroundA = (status & (1 << 27)) != 0;
  bool shortToGroundB = (status & (1 << 28)) != 0;
  bool openLoadA = (status & (1 << 29)) != 0;
  bool openLoadB = (status & (1 << 30)) != 0;
  bool standstill = (status & (1 << 31)) != 0;


  if (driver.drv_err()) {
    Serial.println("Driver reports an error");
  }
  if (fullStepActive) {
    Serial.print("Full step active: ");
    Serial.println(fullStepActive);
  }
  Serial.print("Current: ");
  Serial.println(current);
  if (overtemperature) {
    Serial.print("Overtemperature Error: ");
    Serial.println(overtemperature);
  }
  if (overtemperaturePre) {
    Serial.print("Overtemperature pre-warning: ");
    Serial.println(overtemperaturePre);
  }
  if (shortToGroundA) {
    Serial.print("Short to ground A: ");
    Serial.println(shortToGroundA);
  }
  if (shortToGroundB) {
    Serial.print("Short to ground B: ");
    Serial.println(shortToGroundB);
  }
  if (openLoadA) Serial.println("Open load A");
  if (openLoadB) Serial.println("Open load B");
  if (standstill) {
    Serial.print("Standstill: ");
    Serial.println(standstill);
  }
}

void initDriver3() {
  // serial_stream.begin(115200, SERIAL_8N1, UART_RX3, 0);
  // driver3.setup(serial_stream,);
  driver3.setup(serial_stream, 115200, TMC2209::SERIAL_ADDRESS_0, UART_RX3, 0);
  // driver3.enable();
  delay(5);
  driver3.moveAtVelocity(1000);
  for (int i = 0; i < 10; i++) {
    Serial.println(driver3.getVersion());
  }
  while (true) {
    if (serial_stream.available()) {
      Serial.println(serial_stream.read());
    }
  }
  Serial.println(driver3.getVersion());
  return;

  if (driver3.isCommunicatingButNotSetup()) {
    Serial.println("Driver working");
  } else {
    Serial.println("Driver not working");
  }

  digitalWrite(DIR_PIN3, HIGH);
  while (true) {
    digitalWrite(STEP_PIN3, LOW);
    delay(3);
    digitalWrite(STEP_PIN3, HIGH);
    delay(3);
  }
}
void setup() {
  Serial.begin(115200);
  Serial.println("INIT");
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(CS_PIN1, OUTPUT);
  pinMode(CS_PIN2, OUTPUT);
  pinMode(LED, OUTPUT);
  // pinMode(17, OUTPUT);
  // pinMode(16, INPUT);

  pinMode(DIR_PIN3, OUTPUT);
  pinMode(STEP_PIN3, OUTPUT);
  digitalWrite(EN_PIN, LOW);  // Enable driver in hardware

  for (int i = 0; i < 5; i++) {
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(50);
  }

  initDriver3();
  return;

  // Enable one according to your setup
  SPI.begin();
  // SPI1.begin();                    // SPI drivers
  //SERIAL_PORT.begin(115200);      // HW UART drivers
  //driver.beginSerial(115200);     // SW UART drivers

  driver1.begin();  //  SPI: Init CS pins and possible SW SPI pins
                    // UART: Init SW UART (if selected) with default 115200 baudrate

  Serial.println("Driver");
  driver1.toff(5);               // Enables driver in software
  driver1.rms_current(100);      // Set motor RMS current
  driver1.microsteps(SUBSTEPS);  // Set microsteps to 1/16th

  driver1.en_pwm_mode(true);  // Toggle stealthChop on TMC2130/2160/5130/5160
  // driver.en_spreadCycle(true);   // Toggle spreadCycle on TMC2208/2209/2224

  driver1.pwm_autoscale(true);  // Needed for stealthChop


  driver2.begin();               //  SPI: Init CS pins and possible SW SPI pins
  driver2.toff(5);               // Enables driver in software
  driver2.rms_current(100);      // Set motor RMS current
  driver2.microsteps(SUBSTEPS);  // Set microsteps to 1/16th
  driver2.en_pwm_mode(true);     // Toggle stealthChop on TMC2130/2160/5130/5160
  driver2.pwm_autoscale(true);   // Needed for stealthChop

  Serial.println("Driver status 1:");
  printDriverStatus(driver1);
  Serial.println("Driver status 2:");
  printDriverStatus(driver2);
  Serial.println();

  // stepper.setMaxSpeed(100*steps_per_mm);
  // stepper.setAcceleration(1000*steps_per_mm);
  // stepper.setEnablePin(EN_PIN);
  // stepper.setPinsInverted(false, false, true);
  // stepper.enableOutputs();
  // stepper.setMinPulseWidth(20);

  engine.init();
  stepper1 = engine.stepperConnectToPin(STEP_PIN1, DRIVER_RMT);
  if (stepper1 == NULL) {
    Serial.println("!!!");
    return;
  }
  stepper1->setDirectionPin(DIR_PIN1);
  stepper1->setEnablePin(EN_PIN);
  stepper1->setAutoEnable(false);
  stepper1->setSpeedInHz(40000);      // 500 steps/s
  stepper1->setAcceleration(350000);  // 100 steps/s²

  stepper2 = engine.stepperConnectToPin(STEP_PIN2, DRIVER_RMT);
  if (stepper2 == NULL) {
    Serial.println("!!!");
    return;
  }
  stepper2->setDirectionPin(DIR_PIN2);
  stepper2->setEnablePin(EN_PIN);
  stepper2->setAutoEnable(false);
  stepper2->setSpeedInHz(40000);      // 500 steps/s
  stepper2->setAcceleration(350000);  // 100 steps/s²

  Serial.println("DONE");
  // Serial.println(driver.step());
  Serial.println(driver1.maxspeed());
  Serial.println("/");

  tension(10, 500, 1000);
  delay(500);

  tension(10, 150, 400);
  tension(10, 50, 100);
  digitalWrite(LED, HIGH);
  tension(10, 15, 30);
  tension(100, 15, 30);
  tension(600, 15, 30);
  delay(2000);

  Serial.println("Homing finished");
  stepper1->setSpeedInHz(100000);
  stepper2->setSpeedInHz(100000);

  Serial.println("\nDriver status 1:");
  printDriverStatus(driver1);
  Serial.println("\nDriver status 2:");
  printDriverStatus(driver2);
  Serial.println();

  for (int i = 0; i < 50; i++) {
    while (stepper1->isRunning()) delay(1);
    while (stepper2->isRunning()) delay(1);
    Serial.println(i);
    delay(20);
    auto dir = (i % 2) ? 1 : -1;
    auto speed = (i % 5) + 1;
    stepper1->setSpeedInHz(30000 * speed);
    stepper2->setSpeedInHz(30000 * speed);
    stepper1->move(20000 * dir, false);
    stepper2->move(20000 * dir, false);
  }
}

bool shaft = false;

void loop() {
  return;
  if (driver3.isSetupAndCommunicating()) {
    Serial.println("Stepper driver is setup and communicating!");
    Serial.println("Try turning driver power off to see what happens.");
  } else if (driver3.isCommunicatingButNotSetup()) {
    Serial.println("Stepper driver is communicating but not setup!");
    Serial.println("Running setup again...");
    driver3.setup(serial_stream);
  } else {
    Serial.println("Stepper driver is not communicating!");
    Serial.println("Try turning driver power on to see what happens.");
  }
  Serial.println();
  delay(3000);

  // driver.microsteps(128);
  // for (int s = 3; s < 8; s++) {
  //   auto steps = s;
  //   Serial.println(steps);
  //   stepper.setMaxSpeed(steps*10*steps_per_mm);
  //   // driver.microsteps(steps);
  //   stepper.move(20*steps_per_mm*steps); // Move 100mm
  //   // if (stepper.distanceToGo() == 0) {
  //   //   Serial.println("Running");
  //   //   stepper.disableOutputs();
  //   //   delay(1000);
  //   //   stepper.move(200*steps_per_mm); // Move 100mm
  //   //   stepper.enableOutputs();
  //   // }
  //   while(stepper.distanceToGo() != 0) {
  //     stepper.run();
  //   }
  // }
  return;
  for (int s = 1; s < 8; s++) {
    driver1.rms_current(10);  // Set motor RMS current
    driver2.rms_current(10);
    auto hz = 30000;  // 10000*s;
    stepper1->setSpeedInHz(hz);
    stepper2->setSpeedInHz(hz);
    Serial.println(s);
    Serial.println(stepper1->getSpeedInTicks());
    Serial.println(stepper1->getSpeedInUs());
    Serial.println();
    if (stepper1->move(20000, false) != MOVE_OK) {
      Serial.println("Move failed");
    }
    if (stepper2->move(-20000, false) != MOVE_OK) {
      Serial.println("Move failed");
    }
    while (stepper1->isRunning()) {
      // Serial.println(driver.cs2rms(driver.cs_actual()));
      delay(10);
    }
    // stepper->move(-20000, true);
    delay(1000);
  }


  // Run 5000 steps and switch direction in software
  // int d = 64 / (SUBSTEPS/16);
  // for (uint32_t i = 256*6000; i>0; i--) {
  //   digitalWrite(STEP_PIN, HIGH);
  //   delayMicroseconds(d);
  //   digitalWrite(STEP_PIN, LOW);
  //   delayMicroseconds(d);
  // }
  // shaft = !shaft;
  // driver.shaft(shaft);
}
