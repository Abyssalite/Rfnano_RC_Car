#include <Wire.h>
#include <Servo.h>
#include <LedControl.h> 

#define SLAVE_ADDR 8

// Pins for MAX7219
#define DIN 11
#define CS  12
#define CLK 13

// Global objects
Servo servo9;
Servo servo10;
LedControl lc = LedControl(DIN, CLK, CS, 1);  // 1 device

// Communication buffers
volatile uint8_t args[8] = {0};
volatile uint8_t argCount = 0;

union {
  volatile uint16_t singleInt;
  volatile uint8_t  intArray[8];
} result;

volatile bool returnSingleInt = true;

void setServo(uint8_t ch, uint8_t angle) {
  angle = constrain(angle, 0, 180);
  if (ch == 9) {
    if (!servo9.attached()) servo9.attach(9);
    servo9.write(angle);
  } else if (ch == 10) {
    if (!servo10.attached()) servo10.attach(10);
    servo10.write(angle);
  }
}

void playTone(uint8_t pin, uint8_t value, uint8_t mul) {
  if (value == 0 || mul == 0) {
    noTone(pin);
    return;
  }
  uint16_t freq = constrain((uint16_t)value * mul, 31, 16000);  // tone() min ≈ 31 Hz
  tone(pin, freq);
}

void stopTone(uint8_t pin) {
  noTone(pin);
}

void read8Pin() {
  returnSingleInt = false;
  for (uint8_t i = 0; i < 8; i++) {
    if (args[i] < 14)
      result.intArray[i] = digitalRead(args[i]);
    else if (args[i] < 22)                       // A0–A7 on Nano
      result.intArray[i] = map(analogRead(args[i]), 0, 1023, 0, 255);
  }
}

void read1Pin() {
  if (args[0] < 14)
    result.singleInt = digitalRead(args[0]);
  else if (args[0] < 22)
    result.singleInt = analogRead(args[0]);
}

void writePin() {
  bool value = args[0];
  for (uint8_t i = 1; i < argCount; i++)
    digitalWrite(args[i], value);
  result.singleInt = value;
}

void setPin() {
  bool value = args[0];
  for (uint8_t i = 1; i < argCount; i++)
    pinMode(args[i], value ? OUTPUT : INPUT);
  result.singleInt = value;
}

void initDisp() {
  lc.shutdown(0, false);       // wake up
  lc.setIntensity(0, 8);       // 0–15
  lc.clearDisplay(0);
}

void maxWrite(uint8_t reg, uint8_t val) {
  // LedControl uses digit 0–7 and a value (0–15 for digits, or raw segment byte)
  // If you need the exact raw register write of the original code:
  // you can keep a low-level version, but normally you use the library helpers.
  // For compatibility with your original protocol we do a raw write:
  lc.setRow(0, reg, val);      // simplest mapping – adjust if your master expects something else
}

void getUltrasonicDistance() {
  digitalWrite(args[0], LOW);
  delayMicroseconds(2);
  digitalWrite(args[0], HIGH);
  delayMicroseconds(10);
  digitalWrite(args[0], LOW);

  uint32_t duration = pulseIn(args[1], HIGH, 30000UL);

  if (duration == 0) {
    result.singleInt = 0;          // timeout
    return;
  }

  result.singleInt = (uint16_t)((duration * 343UL) / 2000UL);
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
      if (argCount == 2) getUltrasonicDistance();
      break;

    case 8: // servo
      if (argCount == 1) {
        // init – just attach both (or the one you need)
        servo9.attach(9);
        servo10.attach(10);
      }
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
      if (argCount == 1)
        stopTone(args[0]);               // or any pin
      if (argCount == 3)
        playTone(args[0], args[1], args[2]);
      break;

    default:
      result.singleInt = 32001;
      break;
  }
}

void receiveEvent(int howMany) {
  if (howMany < 1) return;

  uint8_t functionId = Wire.read();
  argCount = howMany - 1;

  for (uint8_t i = 0; i < argCount && Wire.available(); i++)
    args[i] = Wire.read();

  returnSingleInt = true;
  switchFunction(functionId);
}

void requestEvent() {
  if (returnSingleInt)
    Wire.write((uint8_t*)&result.singleInt, 2);
  else
    Wire.write((uint8_t*)result.intArray, 8);
}

void setup() {
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

}

void loop() {
}