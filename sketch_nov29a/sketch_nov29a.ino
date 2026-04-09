#include <Wire.h>
#include <RF24.h>
#include <SoftPWM.h>
#include <MPU6050.h>
#include <VL53L1X.h>
#include <VL53L0X.h>

#define B(...) \
  (const uint8_t[]) { \
    __VA_ARGS__ \
  }
#define SLAVE_ADDR 8
#define CE_PIN 10
#define CSN_PIN 9
#define XSHUT_LEFT 5
#define XSHUT_RIGHT 6
#define THRESHOLD 18000
#define DT 0.01f
#define GYRO_PERIOD 20  // 20ms
#define RF_PERIOD 300   // 300ms
#define DISP_PERIOD 20  // 20ms
#define TOF_PERIOD 100  // 20ms

const byte address[6] = "1Node";  // Same address on BOTH boards
unsigned long rfTimer = 0;
unsigned long gyroTimer = 0;
unsigned long dispTimer = 0;
unsigned long tofTimer = 0;
int16_t roll = 0;
int16_t pitch = 0;
int16_t yaw = 0;

const uint8_t FRONT_LEFT[2] = { A1, A0 };
const uint8_t FRONT_RIGHT[2] = { 8, 7 };
const uint8_t REAR_LEFT[2] = { A3, A2 };
const uint8_t REAR_RIGHT[2] = { 4, 2 };
const uint8_t segMap[6] = {
  0b1000000, 0b0000010, 0b0000100,
  0b0001000, 0b0010000, 0b0100000
};

struct SendPayload {
  uint8_t digitalIR[4];
  uint8_t analogIR[4];
  uint16_t distanceSensor[5];
  int16_t gyro[3];
  uint8_t batt;
};
SendPayload sendPayload;
struct ReceivePayload {
  int8_t joystick1[2];
  int8_t joystick2[2];
  uint8_t digitalButton[6];
  uint8_t analogButton;
  //uint8_t batt;
};
ReceivePayload receivePayload;

RF24 radio(CE_PIN, CSN_PIN);
VL53L1X sensorFront;
VL53L0X sensorRight;
VL53L0X sensorLeft;
MPU6050 mpu;

uint8_t servo1Value = 90;
uint8_t servo2Value = 90;
uint8_t irValues[8];
uint16_t sensors[5];
bool isStopped = false;

// int return
uint16_t masterCallInt(uint8_t func, const uint8_t* args = nullptr, uint8_t len = 0) {
  Wire.beginTransmission((uint8_t)SLAVE_ADDR);
  Wire.write(func);
  if (args && len > 0) Wire.write(args, len);
  if (Wire.endTransmission(true) != 0) return 30054;

  unsigned long start = millis();
  uint8_t received = Wire.requestFrom((uint8_t)SLAVE_ADDR, (uint8_t)2);

  while (Wire.available() < 2) {
    if (millis() - start > 20) return 30054;  // 15ms timeout
  }

  uint16_t value = 0;
  Wire.readBytes((uint8_t*)&value, 2);
  return value;
}

// 8-byte array return
uint8_t masterCallIntArray(uint8_t func, const uint8_t* args, uint8_t* results) {
  Wire.beginTransmission((uint8_t)SLAVE_ADDR);
  Wire.write(func);
  if (args) Wire.write(args, 8);
  if (Wire.endTransmission(true) != 0) return 254;

  unsigned long start = millis();
  if (Wire.requestFrom((uint8_t)SLAVE_ADDR, (uint8_t)8) != 8) {
    while (Wire.available() < 8) {
      if (millis() - start > 20) return 254;  // 15ms timeout
    }
  }

  uint8_t buffer[8];
  Wire.readBytes(buffer, 8);
  memcpy(results, buffer, 8);
  return 0;
}

void setMotor(uint8_t pins[2], int8_t speed) {
  speed = map(speed, -126, 126, -100, 100);
  speed = constrain(speed, -100, 100);

  if (speed > 10) {
    SoftPWMSetPercent(pins[1], 0);
    SoftPWMSetPercent(pins[0], speed);
    isStopped = false;
  } else if (speed < -10) {
    SoftPWMSetPercent(pins[0], 0);
    SoftPWMSetPercent(pins[1], abs(speed));
    isStopped = false;
  } else {
    SoftPWMSetPercent(pins[0], 0);
    SoftPWMSetPercent(pins[1], 0);
    isStopped = true;
  }
}

void setServo(int8_t position, uint8_t* servoValue, uint8_t servo) {
  if (abs(position) < 10) return;
  uint8_t buf[2];

  position = map(position, -126, 126, -3, 3);
  position = constrain(position, -3, 3);

  *servoValue = constrain(*servoValue += position, 5, 175);

  buf[0] = servo;
  buf[1] = *servoValue;
  masterCallInt(8, buf, 2);
}

void moveFordward() {
  setMotor(FRONT_LEFT, receivePayload.joystick2[1]);
  setMotor(FRONT_RIGHT, receivePayload.joystick2[1]);
  setMotor(REAR_LEFT, receivePayload.joystick2[1]);
  setMotor(REAR_RIGHT, receivePayload.joystick2[1]);
}

void moveHead() {
  setServo(receivePayload.joystick1[1], &servo1Value, 10);
  setServo(receivePayload.joystick1[0], &servo2Value, 9);
}

void updateDisplay() {
  uint8_t buf[2];

  // === 1. Gauges for first 5 analog irValues (digits 1 to 5) ===

  int8_t gauges[5] = {
    abs(receivePayload.joystick1[0]),
    abs(receivePayload.joystick1[1]),
    abs(receivePayload.joystick2[0]),
    abs(receivePayload.joystick2[1]),
    receivePayload.analogButton
  };

  for (uint8_t i = 0; i < 5; i++) {
    // Scale to 0-7 (height of bar)
    uint8_t level = map(constrain(gauges[i], 0, 130), 0, 130, 0, 6);

    uint8_t pattern = 0;

    for (uint8_t j = 1; j <= level; j++)
      pattern |= segMap[j];

    buf[0] = i + 1;  // digit position 0..4
    buf[1] = pattern;
    masterCallInt(9, buf, 2);
  }

  // === 2. Vertical lines for digital buttons (digits 6,7,8) ===

  uint8_t btns[6] = {
    receivePayload.digitalButton[0],
    receivePayload.digitalButton[1],
    receivePayload.digitalButton[2],
    receivePayload.digitalButton[3],
    receivePayload.digitalButton[4],
    receivePayload.digitalButton[5]
  };

  for (uint8_t i = 0; i < 3; i++) {
    uint8_t pattern = 0x00;

    if (btns[i] == 0)
      pattern |= 0b0110000;
    if (btns[i + 3] == 0)
      pattern |= 0b0000110;

    buf[0] = i + 6;  // digits 5,6,7
    buf[1] = pattern;
    masterCallInt(9, buf, 2);
  }
}

void setup() {
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);

  Wire.begin();
  Wire.setClock(200000);  // use 200 kHz I2C
  Wire.setWireTimeout(30000, false);
  SoftPWMBegin();
  radio.begin();

  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  // 250 kbps = best range & reliability
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(115);

  radio.openWritingPipe(address);     // TX address
  radio.openReadingPipe(1, address);  // RX address (different pipe)
  radio.startListening();             // start in RX mode

  sensorFront.init();
  sensorFront.setAddress(0x30);
  sensorFront.setTimeout(100);
  sensorFront.setDistanceMode(VL53L1X::Long);
  sensorFront.setMeasurementTimingBudget(33000);
  sensorFront.startContinuous(100);

  digitalWrite(XSHUT_LEFT, HIGH);

  sensorLeft.init();
  sensorLeft.setAddress(0x31);
  sensorLeft.setTimeout(100);
  sensorLeft.setMeasurementTimingBudget(33000);
  sensorLeft.startContinuous(100);

  digitalWrite(XSHUT_RIGHT, HIGH);

  sensorRight.init();
  sensorRight.setAddress(0x32);
  sensorRight.setTimeout(100);
  sensorRight.setMeasurementTimingBudget(33000);
  sensorRight.startContinuous(100);

  SoftPWMSet(2, 0);
  SoftPWMSet(4, 0);
  SoftPWMSet(7, 0);
  SoftPWMSet(8, 0);
  SoftPWMSet(A0, 0);
  SoftPWMSet(A1, 0);
  SoftPWMSet(A2, 0);
  SoftPWMSet(A3, 0);

  masterCallInt(2, B(0, 1, 3), 3);            // set INPUT US
  masterCallInt(2, B(1, 0, 2), 3);            // set OUTPUT US
  masterCallInt(2, B(1, 4, 21, 20), 4);       // set OUTPUT 7seg
  masterCallInt(2, B(1, 10, 9), 3);           // set OUTPUT Servo
  masterCallInt(2, B(0, 14, 15, 16, 17), 4);  // set INPUT IR
  masterCallInt(2, B(0, 7, 8, 12, 13), 4);    // set INPUT IR

  masterCallInt(9, 1, 1);     // Init 7seg
  masterCallInt(8, B(0), 1);  // Init Servo

  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  // Calibrate gyroscope. The calibration must be at rest.
  mpu.calibrateGyro();
  mpu.setThreshold(3);
}

void loop() {
  // 1. RECEIVE PART
  if (radio.available()) {
    radio.read(&receivePayload, sizeof(receivePayload));

    moveFordward();
    moveHead();
  }

  unsigned long now = millis();
  if (now - dispTimer >= DISP_PERIOD) {
    updateDisplay();
  }

  if (analogRead(A7) > 750) {

    if (now - tofTimer >= TOF_PERIOD) {
      sensors[0] = sensorFront.read(false);
      sensors[1] = sensorLeft.readRangeContinuousMillimeters();
      sensors[2] = sensorRight.readRangeContinuousMillimeters();
      sensors[3] = masterCallInt(7, B(0, 1), 2);
      sensors[4] = masterCallInt(7, B(2, 3), 2);

      masterCallIntArray(5, B(7, 8, 12, 13, 14, 15, 16, 17), irValues);
    }

    if (now - gyroTimer >= GYRO_PERIOD) {
      gyroTimer = GYRO_PERIOD;

      Vector norm = mpu.readNormalizeGyro();

      roll += (int16_t)(norm.XAxis * GYRO_PERIOD);  // period(ms) / 1000 * 1000(scale)
      pitch += (int16_t)(norm.YAxis * GYRO_PERIOD);
      yaw += (int16_t)(norm.ZAxis * GYRO_PERIOD);

      roll = (abs(roll) > THRESHOLD) ? roll * -1 : roll;
      pitch = (abs(pitch) > THRESHOLD) ? pitch * -1 : pitch;
      yaw = (abs(yaw) > THRESHOLD) ? yaw * -1 : yaw;

      sendPayload.gyro[0] = roll;
      sendPayload.gyro[1] = pitch;
      sendPayload.gyro[2] = yaw;
    }
  }

  // 2. TRANSMIT PART
  if (now - rfTimer >= RF_PERIOD) {
    rfTimer = now;
    radio.stopListening();

    sendPayload.digitalIR[0] = irValues[0];
    sendPayload.digitalIR[1] = irValues[1];
    sendPayload.digitalIR[2] = irValues[2];
    sendPayload.digitalIR[3] = irValues[3];

    sendPayload.analogIR[0] = irValues[4];
    sendPayload.analogIR[1] = irValues[5];
    sendPayload.analogIR[2] = irValues[6];
    sendPayload.analogIR[3] = irValues[7];

    sendPayload.distanceSensor[0] = sensors[0];
    sendPayload.distanceSensor[1] = sensors[1];
    sendPayload.distanceSensor[2] = sensors[2];

    sendPayload.batt = map(analogRead(A7), 0, 1023, 0, 255);

    radio.write(&sendPayload, sizeof(sendPayload));
    radio.startListening();
  }
}