#include <avr/wdt.h>
#include <Wire.h>
#include <Servo.h>
#include <LedControl.h> 
#include <MPU6050_light.h>

#define SLAVE_ADDR 8

// Pins for MAX7219
#define DIN 11
#define CS  12
#define CLK 13
#define GYRO_PERIOD 20  // 10ms

// Global objects
Servo servo9;
Servo servo10;
LedControl lc = LedControl(DIN, CLK, CS, 1);  // 1 device
MPU6050 mpu(Wire);

byte mpuDetected = 1;
unsigned long gyroTimer = 0;
float pitch = 0;
float roll = 0;
float yaw = 0;

// Communication buffers
volatile uint8_t args[12] = {0};
volatile uint8_t argCount = 0;

union {
  volatile uint16_t singleUInt;

  volatile uint16_t uintArray16[8];
  volatile int16_t  intArray16[8];

  volatile uint8_t  bytes[16];       // raw byte access
} result;

volatile uint8_t replyLength = 0;

void moveServo(uint8_t ch, uint8_t whole, uint8_t remainder) {
  uint16_t angle = whole * 10 + remainder;
  uint16_t us = map(angle, 0, 1800, 500, 2500);

  if (ch == 9) {
    if (!servo9.attached()) {
      servo9.attach(9, 500, 2500);
    }
    servo9.writeMicroseconds(us);
  }
  else if (ch == 10) {
    if (!servo10.attached()) {
      servo10.attach(10, 500, 2500);
    }
    servo10.writeMicroseconds(us);
  }
}

void setServo() {
  if (argCount == 3) {
    replyLength = 2;
    moveServo(args[0], args[1], args[2]);
    result.singleUInt = 0;
  }
  if (argCount == 6) {
    replyLength = 2;

    moveServo(args[0], args[1], args[2]);
    moveServo(args[3], args[4], args[5]);
    result.singleUInt = 0;
  }
}

void playTone(uint8_t pin, uint8_t value, uint8_t mul) {
  if (value == 0 || mul == 0) {
    noTone(pin);
    return;
  }
  uint16_t freq = constrain((uint16_t)value * (uint16_t)mul, 31, 16000);  // tone() min ≈ 31 Hz
  tone(pin, freq);

  replyLength = 2;
  result.singleUInt = 0;
}

void stopTone(uint8_t pin) {
  noTone(pin);
}

void read8Pin() {
  replyLength = 16;

  for (uint8_t i = 0; i < 8; i++) {
    if (args[i] < 14)
      result.intArray16[i] = digitalRead(args[i]);
    else if (args[i] < 22)                       // A0–A7 on Nano
      result.intArray16[i] = analogRead(args[i]);
  }
}

void read1Pin() {
  replyLength = 2;

  if (args[0] < 14)
    result.singleUInt = digitalRead(args[0]);
  else if (args[0] < 22)
    result.singleUInt = analogRead(args[0]);
}

void writePin() {
  bool value = args[0];
  for (uint8_t i = 1; i < argCount; i++)
    digitalWrite(args[i], value);

  replyLength = 2;
  result.singleUInt = 0;
}

void setPin() {
  bool value = args[0];
  for (uint8_t i = 1; i < argCount; i++)
    pinMode(args[i], value);

  replyLength = 2;
  result.singleUInt = 0;
}

void initDisp() {
  lc.shutdown(0, false);       // wake up
  lc.setIntensity(0, 8);       // 0–15
  lc.clearDisplay(0);

  replyLength = 2;
  result.singleUInt = 0;
}

void maxWrite() {
  // LedControl uses digit 0–7 and a value (0–15 for digits, or raw segment byte)
  for (uint8_t i = 0; i < argCount; i++)
      lc.setRow(0, i, args[i]);
  if (argCount < 8) {
    for (uint8_t i = argCount; i < 8; i++)
      lc.setRow(0, i, 0);
  }

  replyLength = 2;
  result.singleUInt = 0;
}

uint16_t calculateUltrasonicDistance(uint8_t trigger, uint8_t echo) {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  uint32_t duration = pulseIn(echo, HIGH, 30000UL);

  if (duration == 0) {
    return 0;
  }
  return static_cast<int16_t>((duration * 343UL) / 2000UL);
}

void getUltrasonicDistance() {
  if (argCount == 2) {
    replyLength = 2;
    result.singleUInt = calculateUltrasonicDistance(args[0], args[1]);
  }
  if (argCount == 4) {
    replyLength = 4;
    result.uintArray16[0] = calculateUltrasonicDistance(args[0], args[1]);
    result.uintArray16[1] = calculateUltrasonicDistance(args[2], args[3]);
  }
}

void getMPU() {
  replyLength = 6;
  result.intArray16[0] = static_cast<int16_t>(pitch * 100.0f);
  result.intArray16[1] = static_cast<int16_t>(roll * 100.0f);
  result.intArray16[2] = static_cast<int16_t>(yaw * 100.0f);

}

void switchFunction(uint8_t functionId) {
  switch (functionId) {

    case 2: // set pinMode
      if (argCount > 1) setPin();
      break;

    case 3: // digitalWrite
      if (argCount > 1) writePin();
      break;

    case 4: // read 1 pin
      if (argCount == 1) read1Pin();
      break;

    case 5: // read 8 pins
      if (argCount == 8) read8Pin();
      break;

    case 7: // ultrasonic
      if (argCount == 2 || argCount == 4) getUltrasonicDistance();
      break;

    case 8: // servo
      if (argCount == 3 ||  argCount == 6) setServo();
      break;

    case 9: // display
      if (argCount > 0 && argCount < 9)
        maxWrite();
      break;

    case 10: // PWM / analogWrite
      if (argCount == 2){
        analogWrite(args[0], constrain(args[1], 0, 255));
        replyLength = 2;
        result.singleUInt = 0;
      }
      break;

    case 11: // tone
      if (argCount == 3)
        playTone(args[0], args[1], args[2]);
      break;

    case 12: // mpu
      if (argCount == 1)
        getMPU();
      break;

    default:
      result.singleUInt = 32001;
      break;
  }
}

void receiveEvent(int howMany) {
  if (howMany < 1) return;

  uint8_t functionId = Wire.read();
  argCount = howMany - 1;

  for (uint8_t i = 0; i < argCount && Wire.available(); i++)
    args[i] = Wire.read();

  replyLength = 0;
  switchFunction(functionId);
}

void requestEvent() {
    Wire.write((uint8_t*)&result, replyLength);
}

void setup() {
  //Serial.begin(9600);
  wdt_enable(WDTO_4S);

  Wire.begin(SLAVE_ADDR);
  Wire.setClock(200000);  // use 200 kHz I2C

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  initDisp();
  mpuDetected = !mpu.begin();
  
  if (mpuDetected) { 
    delay(1000);
    mpu.calcOffsets(); // gyro and accelero
  }
}

void loop() {
  wdt_reset();
  unsigned long now = millis();

  if ((now - gyroTimer >= GYRO_PERIOD) && mpuDetected) {
    gyroTimer += GYRO_PERIOD;
    mpu.update();
    // Get the filtered angles (in degrees)
    roll  = mpu.getAngleX();       // X axis
    pitch = mpu.getAngleY();       // Y axis
    yaw   = mpu.getAngleZ();       // Z axis    
  }  
}