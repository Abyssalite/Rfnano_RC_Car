#include <Wire.h>
#include <Servo.h>
#include <LedControl.h> 
#include <MPU6050.h>

#define SLAVE_ADDR 8

// Pins for MAX7219
#define DIN 11
#define CS  12
#define CLK 13
#define GYRO_PERIOD 10  // 10ms
#define TIME_STEP 0.01f

// Global objects
Servo servo9;
Servo servo10;
LedControl lc = LedControl(DIN, CLK, CS, 1);  // 1 device
MPU6050 mpu;

unsigned long gyroTimer = 0;
float pitch = 0;
float roll = 0;
float yaw = 0;

// Communication buffers
volatile uint8_t args[8] = {0};
volatile uint8_t argCount = 0;

union {
  volatile uint32_t singleUInt;

  volatile uint32_t uintArray32[4];
  volatile int32_t  intArray32[4];

  volatile uint16_t uintArray16[8];
  volatile uint8_t  bytes[16];       // raw byte access
} result;

volatile uint8_t replyLength = 0;

void setServo(uint8_t ch, uint8_t angle) {
  angle = constrain(angle, 0, 180);
  if (ch == 9) {
    if (!servo9.attached()) servo9.attach(9);
    servo9.write(angle);
  } else if (ch == 10) {
    if (!servo10.attached()) servo10.attach(10);
    servo10.write(angle);
  }

  replyLength = 4;
  result.singleUInt = 0;
}

void playTone(uint8_t pin, uint8_t value, uint8_t mul) {
  if (value == 0 || mul == 0) {
    noTone(pin);
    return;
  }
  uint16_t freq = constrain((uint16_t)value * mul, 31, 16000);  // tone() min ≈ 31 Hz
  tone(pin, freq);

  replyLength = 4;
  result.singleUInt = 0;
}

void stopTone(uint8_t pin) {
  noTone(pin);
}

void read8Pin() {
  replyLength = 16;

  for (uint8_t i = 0; i < 8; i++) {
    if (args[i] < 14)
      result.uintArray16[i] = digitalRead(args[i]);
    else if (args[i] < 22)                       // A0–A7 on Nano
      result.uintArray16[i] = analogRead(args[i]);
  }
}

void read1Pin() {
  replyLength = 4;

  if (args[0] < 14)
    result.singleUInt = digitalRead(args[0]);
  else if (args[0] < 22)
    result.singleUInt = analogRead(args[0]);
}

void writePin() {
  bool value = args[0];
  for (uint8_t i = 1; i < argCount; i++)
    digitalWrite(args[i], value);

  replyLength = 4;
  result.singleUInt = 0;
}

void setPin() {
  bool value = args[0];
  for (uint8_t i = 1; i < argCount; i++)
    pinMode(args[i], value ? OUTPUT : INPUT);

  replyLength = 4;
  result.singleUInt = 0;
}

void initDisp() {
  lc.shutdown(0, false);       // wake up
  lc.setIntensity(0, 8);       // 0–15
  lc.clearDisplay(0);

  replyLength = 4;
  result.singleUInt = 0;
}

void maxWrite(uint8_t reg, uint8_t val) {
  // LedControl uses digit 0–7 and a value (0–15 for digits, or raw segment byte)
  lc.setRow(0, reg, val);

  replyLength = 4;
  result.singleUInt = 0;
}

uint32_t calculateUltrasonicDistance(uint8_t trigger, uint8_t echo) {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  uint32_t duration = pulseIn(echo, HIGH, 30000UL);

  if (duration == 0) {
    return 0;
  }

  return (duration * 343UL) / 2000UL;
}

void getUltrasonicDistance() {
  if (argCount == 2) {
    replyLength = 4;
    result.singleUInt = calculateUltrasonicDistance(args[0], args[1]);
  }
  if (argCount == 4) {
    replyLength = 8;
    result.uintArray32[0] = calculateUltrasonicDistance(args[0], args[1]);
    result.uintArray32[1] = calculateUltrasonicDistance(args[2], args[3]);
  }
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
      if (argCount == 2)
        setServo(args[0], args[1]);
      break;

    case 9: // display
      if (argCount == 1)
        initDisp();
      if (argCount == 2)
        maxWrite(args[0], args[1]);
      break;

    case 10: // PWM / analogWrite
      if (argCount == 2)
        analogWrite(args[0], constrain(args[1], 0, 255));
      break;

    case 11: // tone
      if (argCount == 3)
        playTone(args[0], args[1], args[2]);
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
  Wire.begin(SLAVE_ADDR);
  Wire.setClock(400000); // use 200 kHz I2C
  #if defined(WIRE_HAS_TIMEOUT) || defined(WIRE_TIMEOUT)
    Wire.setWireTimeout(25000, true);
  #endif

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  delay(50);

  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  // Calibrate gyroscope. The calibration must be at rest.
  mpu.calibrateGyro();
  mpu.setThreshold(3);

}

void loop() {
  unsigned long now = millis();
  if (now - gyroTimer >= GYRO_PERIOD) {
    gyroTimer = now;
    Vector norm = mpu.readNormalizeGyro();

    pitch = pitch + norm.YAxis * TIME_STEP;
    roll = roll + norm.XAxis * TIME_STEP;
    yaw = yaw + norm.ZAxis * TIME_STEP;
  }  
}