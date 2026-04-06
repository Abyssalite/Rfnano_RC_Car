#include <Wire.h>
#define SLAVE_ADDR 8
#define B(...)  (const uint8_t[]){ __VA_ARGS__ }
#include <RF24.h>
#include <SoftPWM.h>
#include <MPU6050.h>
#include <VL53L1X.h>
#include <VL53L0X.h>

#define CE_PIN  10
#define CSN_PIN  9
#define XSHUT_LEFT  6
#define XSHUT_RIGHT  5
RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "1Node";          // Same address on BOTH boards
const uint8_t RF_PERIOD = 300; 
unsigned long rfTimer = 0;

const uint8_t FRONT_LEFT[2] = {A1, A0}; 
const uint8_t FRONT_RIGHT[2] = {8, 7};  
const uint8_t REAR_LEFT[2] = {A3, A2};  
const uint8_t REAR_RIGHT[2] = {4, 2}; ; 

struct SendPayload {
    uint8_t digitalIR[4];
    uint8_t analogIR[4];
    uint16_t distanceSensor[3];
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

VL53L1X sensorFront;
VL53L0X sensorRight;
VL53L0X sensorLeft;

MPU6050 mpu;
unsigned long gyroTimer = 0;
const uint8_t GYRO_PERIOD = 10;   // 100 Hz

uint8_t servo1Value;
uint8_t servo2Value;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

// int return
uint16_t masterCallInt(uint8_t func, const uint8_t* args = nullptr, uint8_t len = 0) {
  Wire.beginTransmission((uint8_t)SLAVE_ADDR);
  Wire.write(func);
  if (args && len > 0) Wire.write(args, len);
  Wire.endTransmission();

  if (Wire.requestFrom((uint8_t)SLAVE_ADDR, (uint8_t)2) != 2) return 30054;

  uint16_t value;
  Wire.readBytes((uint8_t*)&value, 2);

  return value;
}

// 8-byte array return
uint8_t masterCallIntArray(uint8_t func, const uint8_t* args = nullptr, uint8_t len = 0, uint8_t* results = nullptr, uint8_t resultslen = 0) {
  Wire.beginTransmission((uint8_t)SLAVE_ADDR);
  Wire.write(func);
  if (args && len == 8) Wire.write(args, len);
  Wire.endTransmission();

  if (Wire.requestFrom((uint8_t)SLAVE_ADDR, (uint8_t)8) != 8 || resultslen != 8) return 254;

  uint8_t buffer[8];
  Wire.readBytes(buffer, 8);

  for (uint8_t i = 0; i < 8; i++) {
    results[i] = buffer[i] | (buffer[i+1] << 8);  // little-endian
  }
  return 0;
}

void motor(uint8_t pin1, uint8_t pin2, int8_t& speed) {
  speed = map(speed, -126, 126, -100, 100);
  speed = constrain(speed, -100, 100);
 
  if (speed > 10) {
    SoftPWMSetPercent(pin2, 0);
    SoftPWMSetPercent(pin1, speed);
  } 
  else if (speed < -10) {
    SoftPWMSetPercent(pin1, 0);
    SoftPWMSetPercent(pin2, abs(speed));
  }
  else {
    SoftPWMSetPercent(pin1, 0);
    SoftPWMSetPercent(pin2, 0);
  }
}

void moveFordward(int8_t speed){
  motor(FRONT_LEFT[0],FRONT_LEFT[1],speed);
  motor(FRONT_RIGHT[0],FRONT_RIGHT[1],speed);
  motor(REAR_LEFT[0],REAR_LEFT[1],speed);
  motor(REAR_RIGHT[0],REAR_RIGHT[1],speed);
}

void updateDisplay() {
  uint8_t buf[2];

  // === 1. Gauges for first 5 analog values (digits 1 to 5) ===

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
    if (level >= 1) pattern |= 0b1000000; 
    if (level >= 2) pattern |= 0b0000010;
    if (level >= 3) pattern |= 0b0000100; 
    if (level >= 4) pattern |= 0b0001000; 
    if (level >= 5) pattern |= 0b0010000; 
    if (level >= 6) pattern |= 0b0100000; 

    buf[0] = i + 1;           // digit position 0..4
    buf[1] = pattern;
    masterCallInt(10, buf, 2);
  }

  // === 2. Vertical lines for digital buttons (digits 6,7,8) ===
  // Show 6 buttons as simple on/off vertical line
  
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

    buf[0] = i + 6;       // digits 5,6,7
    buf[1] = pattern;
    masterCallInt(10, buf, 2);
  }
}

void setup() {  
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);

  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  SoftPWMBegin();  
  radio.begin();

  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);       // 250 kbps = best range & reliability
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(115);

  radio.openWritingPipe(address);       // TX address
  radio.openReadingPipe(1, address);    // RX address (different pipe)

  radio.startListening();               // start in RX mode

  Serial.println(mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G));
  // Calibrate gyroscope. The calibration must be at rest.
  mpu.calibrateGyro();
  mpu.setThreshold(3);

  sensorFront.init();
  sensorFront.setAddress(0x30);
  sensorFront.setTimeout(50);
  sensorFront.setDistanceMode(VL53L1X::Long);
  sensorFront.setMeasurementTimingBudget(33000);
  sensorFront.startContinuous(50);

  digitalWrite(XSHUT_LEFT, HIGH);
  
  sensorLeft.init();
  sensorLeft.setAddress(0x31);
  sensorLeft.setTimeout(50);
  sensorLeft.setMeasurementTimingBudget(33000);
  sensorLeft.startContinuous(50);

  digitalWrite(XSHUT_RIGHT, HIGH);
  
  sensorRight.init();
  sensorLeft.setAddress(0x32);
  sensorRight.setTimeout(50);
  sensorRight.setMeasurementTimingBudget(33000);
  sensorRight.startContinuous(50);

  SoftPWMSet(0, 0);
  SoftPWMSet(4, 0);
  SoftPWMSet(7, 0);
  SoftPWMSet(8, 0);
  SoftPWMSet(A0, 0);
  SoftPWMSet(A1, 0);
  SoftPWMSet(A2, 0);
  SoftPWMSet(A3, 0);

  masterCallInt(2, B(0, 1, 3), 3); // set INPUT US
  masterCallInt(2, B(1, 0, 2), 3); // set OUTPUT US
  masterCallInt(2, B(1, 4, 8, 7), 4); // set OUTPUT 7seg
  masterCallInt(10, 1, 1); // Init 7seg

  masterCallInt(8, B(5, 6), 2);
}

void loop() {
  // 1. RECEIVE PART (always active)
  if (radio.available()) {
    radio.read(&receivePayload, sizeof(receivePayload));
    
    updateDisplay(); 
  }

  const uint16_t sensors[3] = {sensorFront.read(), sensorLeft.readRangeContinuousMillimeters(), sensorRight.readRangeContinuousMillimeters()};
  uint8_t values[8];
  masterCallIntArray(5, B(10, 11, 12, 13, 14, 15, 16, 17), 8, values, sizeof(values));

  unsigned long now = millis();
  if (now - gyroTimer >= GYRO_PERIOD){
      gyroTimer = now;
      Vector norm = mpu.readNormalizeGyro();
      pitch += norm.YAxis * 0.01;
      roll  += norm.XAxis * 0.01;
      yaw   += norm.ZAxis * 0.01;

      pitch = (abs(pitch) > 18.0)? pitch * -1 : pitch;
      roll = (abs(roll) > 18.0)? roll * -1 : roll;
      yaw = (abs(yaw) > 18.0)? yaw * -1 : yaw;

      sendPayload.gyro[0] = (int16_t)(pitch * 100.0f);
      sendPayload.gyro[1] = (int16_t)(roll * 100.0f);
      sendPayload.gyro[2] = (int16_t)(yaw * 100.0f);
  }

  /*masterCallInt(9, B(5, 180), 2);
  masterCallInt(9, B(180, 180), 2);*/
  
  moveFordward(receivePayload.joystick2[1]);

  // 2. TRANSMIT PART (every 100 ms)
  if (now - rfTimer >= RF_PERIOD){
    rfTimer = now;
    radio.stopListening();              // switch to TX mode

    sendPayload.digitalIR[0] = values[0];
    sendPayload.digitalIR[1] = values[1];
    sendPayload.digitalIR[2] = values[2];
    sendPayload.digitalIR[3] = values[3];

    sendPayload.analogIR[0] = values[4];
    sendPayload.analogIR[1] = values[5];
    sendPayload.analogIR[2] = values[6];
    sendPayload.analogIR[3] = values[7];

    sendPayload.distanceSensor[0] = sensors[0];
    sendPayload.distanceSensor[1] = sensors[1];
    sendPayload.distanceSensor[2] = sensors[2];

    sendPayload.batt = map(analogRead(A6), 0, 1023, 0, 255);

    bool ok = radio.write(&sendPayload, sizeof(sendPayload));

    radio.startListening();             // back to RX mode
  }
}