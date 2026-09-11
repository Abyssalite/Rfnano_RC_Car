#include <avr/wdt.h>
#include <Wire.h>
#include <RF24.h>
#include <SoftPWM.h>
#include <VL53L0X.h>
#include "I2CMaster.h"

#define B(...) \
  (const uint8_t[]) { \
    __VA_ARGS__ \
  }
#define CE_PIN 10
#define CSN_PIN 9
#define XSHUT_LEFT 7
#define XSHUT_MIDDLE 0
#define XSHUT_RIGHT 8

#define THRESHOLD 18000
#define RF_PERIOD 200   // 300ms
#define IR_PERIOD 110   // 30ms
#define GYRO_PERIOD 60  // 20ms
#define DISP_PERIOD 120  // 33ms
#define TOF_PERIOD 80   // 45ms
#define US_PERIOD 100    // 55ms
#define CONNECTION_TIMEOUT 500

const byte address[6] = "1Node";  // Same address on BOTH boards

uint8_t reconnectTicker = 0;
uint8_t displayBuffer[8] = { 0 };

unsigned long lastReceiveTime = 0;
unsigned long gyroTimer = 0;
unsigned long rfTimer = 0;
unsigned long dispTimer = 0;
unsigned long tofTimer = 0;
unsigned long usTimer = 0;
unsigned long irTimer = 0;

const uint8_t MOTOR[4][2] = {
  { A0, A1 },  // FRONT_LEFT
  { 6, 5 },    // FRONT_RIGHT
  { A2, A3 },  // REAR_LEFT
  { 4, 3 }     // REAR_RIGHT
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
  uint8_t alert[2];
  uint8_t analogButton;
};
ReceivePayload receivePayload;

RF24 radio(CE_PIN, CSN_PIN);
VL53L0X sensorFront;
VL53L0X sensorRight;
VL53L0X sensorLeft;

int16_t gyro[3] = { 0 };
uint16_t servoValue[2] = { 900 };
uint16_t irValues[8];
uint16_t tofSensors[3];
uint16_t usSensors[2];
byte isStopped = 0;

void setMotor(uint8_t pins[2], int8_t speed, uint8_t i) {
  speed = map(speed, -127, 127, -100, 100);
  speed = constrain(speed, -100, 100);

  if (speed < -10) {  // Forward
    SoftPWMSetPercent(pins[1], 0);
    SoftPWMSetPercent(pins[0], abs(speed));
    isStopped = 0;
  } else if (speed > 10) {  // Backward
    SoftPWMSetPercent(pins[1], speed);
    SoftPWMSetPercent(pins[0], 0);
    isStopped = 0;
  } else {
    if (isStopped) return;
    SoftPWMSetPercent(pins[0], 0);
    SoftPWMSetPercent(pins[1], 0);
    if (i == 3) isStopped = 1;
  }
}

void setServo(int8_t joystick, uint16_t* servoValue, uint8_t servo) {
  if (abs(joystick) < 10) return;

  joystick = map(joystick, -127, 127, -15, 15);
  joystick = constrain(joystick, -15, 15);

  int16_t temp = *servoValue + joystick;
  if (servo == 9)
    *servoValue = (uint16_t)constrain(temp, 100, 1700);
  else *servoValue = (uint16_t)constrain(temp, 50, 1750);
}

void moveRobot(byte canMove = 1) {
  int16_t raw[4]{ 0 };

  if (canMove) {
    int8_t x = receivePayload.joystick2[0];  // left/right
    int8_t y = receivePayload.joystick2[1];  // forward/back
    int8_t rot = 0;

    if (receivePayload.analogButton > 120)
      rot = 0;
    else if (receivePayload.analogButton > 110)
      rot = -50;
    else if (receivePayload.analogButton > 80)
      rot = 50;
    else rot = 0;

    raw[1] = rot - y + x;  // FRONT_RIGHT
    raw[0] = rot - y - x;  // FRONT_LEFT
    raw[3] = rot + y + x;  // REAR_RIGHT
    raw[2] = rot + y - x;  // REAR_LEFT

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
    setMotor(MOTOR[i], static_cast<int8_t>(raw[i]), i);
  }
}

void moveHead() {
  uint8_t tempBuffer[6];

  setServo(-receivePayload.joystick1[1], &servoValue[0], 10);  // x-axis
  setServo(receivePayload.joystick1[0], &servoValue[1], 9);    // y-axis

  tempBuffer[0] = 10;
  tempBuffer[1] = (uint8_t)(servoValue[0] / 10);
  tempBuffer[2] = (uint8_t)(servoValue[0] % 10);
  tempBuffer[3] = 9;
  tempBuffer[4] = (uint8_t)(servoValue[1] / 10);
  tempBuffer[5] = (uint8_t)(servoValue[1] % 10);

  masterCallUInt16(8, tempBuffer, 6);
}

void updateBuffer() {
  // === 1. Gauges for first 5 analog irValues (digits 1 to 5) ===
  memset(displayBuffer, 0, sizeof(displayBuffer));
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
  memset(displayBuffer, 0, sizeof(displayBuffer));
  if (reconnectTicker % 2 == 0) { 
    displayBuffer[6] = displayBuffer[6] ^= 0b0001100;
    displayBuffer[1] = displayBuffer[1] ^= 0b1100000;
  }
  else {
    displayBuffer[6] = displayBuffer[6] ^= 0b1000010;
    displayBuffer[1] = displayBuffer[1] ^= 0b0011000;
  }
}

void deviceInit() {
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_MIDDLE, LOW);

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(100);
  sensorLeft.init();
  sensorLeft.setAddress(0x31);
  sensorLeft.setTimeout(50);
  sensorLeft.setMeasurementTimingBudget(33000);
  sensorLeft.startContinuous(40);

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(100);
  sensorRight.init();
  sensorRight.setAddress(0x32);
  sensorRight.setTimeout(50);                    // short timeout
  sensorRight.setMeasurementTimingBudget(33000); // 33 ms (default)
  sensorRight.startContinuous(40);

  digitalWrite(XSHUT_MIDDLE, HIGH);
  delay(100);
  sensorFront.init();
  sensorFront.setAddress(0x33);
  sensorFront.setTimeout(50);                    // short timeout
  sensorFront.setMeasurementTimingBudget(33000); // 33 ms (default)
  sensorFront.startContinuous(40);

  delay(100);
  masterCallUInt16(2, B(INPUT, 0, 1, 3, 7, 8, 14, 16, 17, 20, 21), 11);  // set INPUT IR, US
  masterCallUInt16(2, B(OUTPUT, 2, 4, 5, 6, 11, 12, 13, 15), 9);  // set OUTPUT US, PWM, OUTPUT 7seg
}

void setup() {
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  //Serial.begin(9600);
  wdt_enable(WDTO_2S);

  Wire.begin();
  Wire.setClock(200000);  // use 200 kHz I2C
  radio.begin();

  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  // 250 kbps = best range & reliability
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(115);
  radio.setPayloadSize(32);
  radio.openWritingPipe(address);     // TX address
  radio.openReadingPipe(1, address);  // RX address (different pipe)
  radio.startListening();             // start in RX mode

  SoftPWMBegin();

  SoftPWMSet(3, 0);
  SoftPWMSet(4, 0);
  SoftPWMSet(5, 0);
  SoftPWMSet(6, 0);
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

    lastReceiveTime = now;
    updateBuffer();
  }

  if (now - lastReceiveTime >= CONNECTION_TIMEOUT) {
    memset(&receivePayload, 0, sizeof(receivePayload));
    memset(receivePayload.digitalButton, 1, sizeof(receivePayload.digitalButton));
    moveRobot(0);

    reconnectTicker++;
    if (reconnectTicker > 253) reconnectTicker = 0;
    reconnectSpinner();
  }

  if (now - tofTimer >= TOF_PERIOD) {
    tofTimer = now;
    tofSensors[0] = 0;  //sensorFront.readRangeContinuousMillimeters();
    tofSensors[1] = sensorLeft.readRangeContinuousMillimeters();
    tofSensors[2] = sensorRight.readRangeContinuousMillimeters();
  }
  if (now - usTimer >= IR_PERIOD) {
    irTimer = now;
    masterCallInt16Array(5, B(0, 1, 7, 8, 16, 17, 20, 21), 8, irValues, 8);
  }

  if (now - usTimer >= US_PERIOD) {
    usTimer = now;
    masterCallUInt16Array(7, B(2, 3, 15, 14), 4, usSensors, 2);
  }

  if (analogRead(A6) >= 550) {
    moveHead();
  }
  if (analogRead(A6) >= 700) {
    moveRobot();
  }

  if (now - dispTimer >= DISP_PERIOD) {
    dispTimer = now;
    masterCallUInt16(9, displayBuffer, 8);
  }

  if (now - gyroTimer >= GYRO_PERIOD) {
    gyroTimer = now;
    masterCallInt16Array(12, 1, 1, gyro, 3);
  }

  // 2. TRANSMIT PART
  if (now - rfTimer >= RF_PERIOD) {
    rfTimer = now;
    radio.stopListening();

    sendPayload.gyro[0] = gyro[0];
    sendPayload.gyro[1] = gyro[1];
    sendPayload.gyro[2] = gyro[2];

    sendPayload.digitalIR[0] = (uint8_t)irValues[0];
    sendPayload.digitalIR[1] = (uint8_t)irValues[1];
    sendPayload.digitalIR[2] = (uint8_t)irValues[2];
    sendPayload.digitalIR[3] = (uint8_t)irValues[3];

    sendPayload.analogIR[0] = (uint8_t)map(irValues[4], 0, 1023, 0, 255);
    sendPayload.analogIR[1] = (uint8_t)map(irValues[5], 0, 1023, 0, 255);
    sendPayload.analogIR[2] = (uint8_t)map(irValues[6], 0, 1023, 0, 255);
    sendPayload.analogIR[3] = (uint8_t)map(irValues[7], 0, 1023, 0, 255);

    sendPayload.tofSensors[0] = tofSensors[0];
    sendPayload.tofSensors[1] = tofSensors[1];
    sendPayload.tofSensors[2] = tofSensors[2];

    sendPayload.usSensors[0] = usSensors[0];
    sendPayload.usSensors[1] = usSensors[1];

    sendPayload.batt = map(analogRead(A6), 0, 1023, 0, 255);

    radio.write(&sendPayload, sizeof(sendPayload));
    radio.startListening();
  }
}