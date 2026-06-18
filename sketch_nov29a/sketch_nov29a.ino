#include <avr/wdt.h>
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
#define XSHUT_LEFT 1
#define XSHUT_MIDDLE 6
#define XSHUT_RIGHT 0

#define THRESHOLD 18000
#define DT 0.01f
#define GYRO_PERIOD 10  // 10ms
#define RF_PERIOD 300   // 500ms
#define DISP_PERIOD 20  // 20ms
#define TOF_PERIOD 101  // 101ms
#define US_PERIOD 100   // 100ms
#define CONNECTION_TIMEOUT 500

const byte address[6] = "1Node";  // Same address on BOTH boards

uint8_t displayIndex = 0;
uint8_t displayBuffer[8] = { 0 };
bool isClear = false;

unsigned long lastReceiveTime = 0;
unsigned long rfTimer = 0;
unsigned long gyroTimer = 0;
unsigned long dispTimer = 0;
unsigned long tofTimer = 0;
unsigned long usTimer = 0;

const uint8_t MOTOR[4][2] = {
  { A0, A1 },  // FRONT_RIGHT
  { 8, 7 },    // FRONT_LEFT
  { A2, A3 },  // REAR_RIGHT
  { 4, 2 }     // REAR_LEFT
};

const uint8_t segMap[6] = {
  0b1000000, 0b0000010, 0b0000100,
  0b0001000, 0b0010000, 0b0100000
};

struct SendPayload {
  uint8_t digitalIR[4];
  uint8_t analogIR[4];
  uint16_t tofSensors[3];
  uint16_t usSensors[2];
  int16_t gyro[3];
  uint8_t batt;
};
SendPayload sendPayload;
struct ReceivePayload {
  int8_t joystick1[2];
  int8_t joystick2[2];
  uint8_t digitalButton[6];
  uint8_t analogButton;
};
ReceivePayload receivePayload;

RF24 radio(CE_PIN, CSN_PIN);
VL53L1X sensorFront;
VL53L0X sensorRight;
VL53L0X sensorLeft;
MPU6050 mpu;

int16_t gyro[3] = {0};
uint8_t servoValue[2] = {90};
uint8_t irValues[8];
uint16_t tofSensors[3];
uint16_t usSensors[2];
bool isStopped = false;

// int return
uint16_t masterCallInt(uint8_t func, const uint8_t* args = nullptr, uint8_t len = 0) {
  if (!args || len <= 0) return;
  Wire.beginTransmission((uint8_t)SLAVE_ADDR);
  Wire.write(func);
  Wire.write(args, len);
  if (Wire.endTransmission(true) != 0) return 30054;

  Wire.requestFrom((uint8_t)SLAVE_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 30054;

  uint16_t value = 0;
  Wire.readBytes((uint8_t*)&value, 2);
  return value;
}

// 8-byte array return
uint8_t masterCallIntArray(uint8_t func, const uint8_t* args, uint8_t* results, uint8_t len = 0) {
  if (!func || len != 8) return;
  Wire.beginTransmission((uint8_t)SLAVE_ADDR);
  Wire.write(func);
  Wire.write(args, 8);
  if (Wire.endTransmission(true) != 0) return 254;

  Wire.requestFrom((uint8_t)SLAVE_ADDR, (uint8_t)8);
  if (Wire.available() < 8) return 254;

  uint8_t buffer[8];
  Wire.readBytes(buffer, 8);
  memcpy(results, buffer, 8);
  return 0;
}

void setMotor(uint8_t pins[2], int8_t speed, uint8_t i) {
  speed = map(speed, -127, 127, -100, 100);
  speed = constrain(speed, -100, 100);

  if (speed < -10) {  // Forward
    SoftPWMSetPercent(pins[1], 0);
    SoftPWMSetPercent(pins[0], abs(speed));
    isStopped = false;
  } else if (speed > 10) {  // Backward
    SoftPWMSetPercent(pins[1], speed);
    SoftPWMSetPercent(pins[0], 0);
    isStopped = false;
  } else {
    if (isStopped) return;
    SoftPWMSetPercent(pins[0], 0);
    SoftPWMSetPercent(pins[1], 0);
    if (i == 3) isStopped = true;
  }
}

void setServo(int8_t position, uint8_t* servoValue, uint8_t servo) {
  if (abs(position) < 10) return;
  uint8_t tempBuffer[2];

  position = map(position, -127, 127, -4, 4);
  position = constrain(position, -4, 4);

  int16_t temp = *servoValue + position;
  if (servo == 9)
    *servoValue = (uint8_t)constrain(temp, 10, 170);
  else  *servoValue = (uint8_t)constrain(temp, 5, 175);

  tempBuffer[0] = servo;
  tempBuffer[1] = *servoValue;
  masterCallInt(8, tempBuffer, 2);
}

void moveRobot(bool canMove = true) {
  int16_t raw[4]{ 0 };

  if (canMove) {
    int8_t x = receivePayload.joystick2[0];  // left/right
    int8_t y = receivePayload.joystick2[1];  // forward/back
    int8_t rot = 0;

    if (receivePayload.analogButton > 110)
      rot = -50;
    else if (receivePayload.analogButton > 80)
      rot = 50;

    raw[0] = y - x + rot;  // FRONT_RIGHT
    raw[1] = y - x - rot;  // FRONT_LEFT
    raw[2] = y + x + rot;  // REAR_RIGHT
    raw[3] = y + x - rot;  // REAR_LEFT

    // Normalize to avoid overflow
    int16_t maxVal = 0;
    for (uint8_t i = 0; i < 4; i++) {
      if (abs(raw[i]) > maxVal) maxVal = abs(raw[i]);
    }

    if (maxVal > 127) {
      for (uint8_t i = 0; i < 4; i++) {
        raw[i] = raw[i] * 127 / maxVal;
      }
    }
  }

  for (uint8_t i = 0; i < 4; i++) {
    setMotor(MOTOR[i], (int8_t)raw[i], i);
  }
}

void moveHead() {
  setServo(-receivePayload.joystick1[1], &servoValue[0], 10);  // x-axis
  setServo(receivePayload.joystick1[0], &servoValue[1], 9);    // y-axis
}

void updateBuffer() {
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
    uint8_t level = map(gauges[i], 0, 130, 0, 6);
    level = constrain(level, 0, 6);

    uint8_t pattern = 0;

    for (uint8_t j = 1; j <= level; j++)
      pattern |= segMap[j];

    displayBuffer[i] = pattern;  // digit position 0..4
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

    if (!btns[i])
      pattern |= 0b0110000;
    if (!btns[i + 3])
      pattern |= 0b0000110;

    displayBuffer[i + 5] = pattern;  // digits 5,6,7
  }
}

void reconnectSpinner() {
  if (displayIndex == 3) {
    displayBuffer[6] = displayBuffer[6] ^= 0b1001110;
  }
  if (displayIndex == 7) {
    displayBuffer[1] = displayBuffer[1] ^= 0b1111000;
  }
}

void deviceInit() {
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_MIDDLE, LOW);

  delay(50);

  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  // Calibrate gyroscope. The calibration must be at rest.
  mpu.calibrateGyro();
  mpu.setThreshold(3);

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);

  sensorLeft.init();
  sensorLeft.setAddress(0x31);
  sensorLeft.setTimeout(60);
  sensorLeft.setMeasurementTimingBudget(33000);
  sensorLeft.startContinuous(100);

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);

  sensorRight.init();
  sensorRight.setAddress(0x32);
  sensorRight.setTimeout(60);
  sensorRight.setMeasurementTimingBudget(33000);
  sensorRight.startContinuous(100);

  digitalWrite(XSHUT_MIDDLE, HIGH);
  delay(50);

  sensorFront.init();
  sensorFront.setAddress(0x30);
  sensorFront.setTimeout(60);
  sensorFront.setDistanceMode(VL53L1X::Long);
  sensorFront.setMeasurementTimingBudget(33000);
  sensorFront.startContinuous(100);

  delay(50);

  masterCallInt(2, B(INPUT, 14, 15, 16, 17, 1), 6);  // set INPUT IR, US
  masterCallInt(2, B(INPUT, 7, 8, 12, 13, 3), 6);    // set INPUT IR, US

  masterCallInt(2, B(OUTPUT, 0, 2, 5, 6), 5);        // set OUTPUT US, PWM
  masterCallInt(2, B(OUTPUT, 4, 20, 21), 4);         // set OUTPUT 7seg

  masterCallInt(9, 1, 1);      // Init 7seg
  masterCallInt(8, B(0), 1);   // Init Servo
  masterCallInt(11, B(0), 1);  // Init Tone
}

void setup() {
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_MIDDLE, OUTPUT);

  wdt_enable(WDTO_1S);

  Wire.begin();
  Wire.setClock(200000);  // use 200 kHz I2C
  Wire.setWireTimeout(40000, false);
  radio.begin();

  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  // 250 kbps = best range & reliability
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(115);

  radio.openWritingPipe(address);     // TX address
  radio.openReadingPipe(1, address);  // RX address (different pipe)
  radio.startListening();             // start in RX mode

  SoftPWMBegin();

  SoftPWMSet(2, 0);
  SoftPWMSet(4, 0);
  SoftPWMSet(7, 0);
  SoftPWMSet(8, 0);
  SoftPWMSet(A0, 0);
  SoftPWMSet(A1, 0);
  SoftPWMSet(A2, 0);
  SoftPWMSet(A3, 0);

  deviceInit();
}

void loop() {
  wdt_reset();
  unsigned long now = millis();

  // 1. RECEIVE PART
  if (radio.available()) {
    radio.read(&receivePayload, sizeof(receivePayload));

    if (isClear) isClear = !isClear;
    lastReceiveTime = now;
    updateBuffer();
  }

  if (now - lastReceiveTime >= CONNECTION_TIMEOUT) {
    memset(&receivePayload, 0, sizeof(receivePayload));
    memset(receivePayload.digitalButton, 1, sizeof(receivePayload.digitalButton));
    moveRobot(false);

    if (isClear) {
      memset(displayBuffer, 0, sizeof(displayBuffer));
      isClear = !isClear;
    }
    reconnectSpinner();
  }

  if (analogRead(A7) >= 550) {
    if (now - gyroTimer >= GYRO_PERIOD) {
      gyroTimer = now;
      Vector norm = mpu.readNormalizeGyro();

      gyro[0] += (int16_t)(norm.XAxis * GYRO_PERIOD);  // period(ms) / 1000 * 1000(scale)
      gyro[1] += (int16_t)(norm.YAxis * GYRO_PERIOD);
      gyro[2] += (int16_t)(norm.ZAxis * GYRO_PERIOD);

      for (uint8_t i = 0; i < 3; i++) {
        int16_t temp = (abs(gyro[i]) > THRESHOLD) ? gyro[i] * -1 : gyro[i];
        gyro[i] = constrain(temp, -THRESHOLD, THRESHOLD);        
      }
    }
    
    moveHead();

    if (now - tofTimer >= TOF_PERIOD) {
      tofSensors[0] = 0;//sensorFront.readRangeContinuousMillimeters(false);
      tofSensors[1] = sensorLeft.readRangeContinuousMillimeters();
      tofSensors[2] = sensorRight.readRangeContinuousMillimeters();

      masterCallIntArray(5, B(7, 8, 12, 13, 14, 15, 16, 17), irValues, 8);
    }

    if (now - usTimer >= US_PERIOD) {
      usTimer = now;

      usSensors[0] = masterCallInt(7, B(0, 1), 2);
      usSensors[1] = masterCallInt(7, B(2, 3), 2);
    }
  }

  if (analogRead(A7) > 700) {
    moveRobot();
  }

  if (now - dispTimer >= DISP_PERIOD) {
    dispTimer = now;
    uint8_t tempBuffer[2] = { displayIndex + 1, displayBuffer[displayIndex] };
    masterCallInt(9, tempBuffer, 2);

    displayIndex++;
    if (displayIndex > 7) displayIndex = 0;
  }

  // 2. TRANSMIT PART
  if (now - rfTimer >= RF_PERIOD) {
    rfTimer = now;
    radio.stopListening();

    sendPayload.gyro[0] = gyro[0];
    sendPayload.gyro[1] = gyro[1];
    sendPayload.gyro[2] = gyro[2];

    sendPayload.digitalIR[0] = irValues[0];
    sendPayload.digitalIR[1] = irValues[1];
    sendPayload.digitalIR[2] = irValues[2];
    sendPayload.digitalIR[3] = irValues[3];

    sendPayload.analogIR[0] = irValues[4];
    sendPayload.analogIR[1] = irValues[5];
    sendPayload.analogIR[2] = irValues[6];
    sendPayload.analogIR[3] = irValues[7];

    sendPayload.tofSensors[0] = tofSensors[0];
    sendPayload.tofSensors[1] = tofSensors[1];
    sendPayload.tofSensors[2] = tofSensors[2];

    sendPayload.usSensors[0] = usSensors[0];
    sendPayload.usSensors[1] = usSensors[1];

    sendPayload.batt = map(analogRead(A7), 0, 1023, 0, 255);

    radio.write(&sendPayload, sizeof(sendPayload));
    radio.startListening();
  }
}