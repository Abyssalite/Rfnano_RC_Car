#include <Wire.h>
#include <Servo.h>
#define SLAVE_ADDR 8

Servo servo1;
Servo servo2;
#define DIN 7
#define CLK 4
#define CS 8

// Store received data temporarily
uint8_t args[8] = { 0 };  // max arguments
uint8_t argCount = 0;
union {
  uint16_t singleInt;
  uint8_t intArray[8];
} result;
bool returnSingleInt = true;

void read8Pin() {
  returnSingleInt = false;

  for (uint8_t i = 0; i < 8; i++) {
    if (args[i] < 14)
      result.intArray[i] = digitalRead(args[i]);
    else if (args[i] < 18)
      result.intArray[i] = map(analogRead(args[i]), 0, 1023, 0, 255);
  }
}

void read1Pin() {
  if (args[0] < 14)
    result.singleInt = digitalRead(args[0]);
  else if (args[0] < 18)
    result.singleInt = analogRead(args[0]);
}

void writePin() {
  bool value = args[0];
  for (uint8_t i = 1; i <= argCount - 1; i++) {
    if (args[i] < 18)
      digitalWrite(args[i], value);
  }

  result.singleInt = 32003;
}

void setPin() {
  bool value = args[0];
  for (uint8_t i = 1; i <= argCount - 1; i++)
    if (args[i] < 18) {
      pinMode(args[i], value);
    }

  result.singleInt = 32002;
}

void maxWrite(uint8_t reg, uint8_t val) {
  digitalWrite(CS, LOW);
  shiftOut(DIN, CLK, MSBFIRST, reg);
  shiftOut(DIN, CLK, MSBFIRST, val);
  digitalWrite(CS, HIGH);
}

void maxInit() {
  digitalWrite(CS, HIGH);

  maxWrite(0x0C, 1);   // normal operation
  maxWrite(0x0F, 0);   // test off
  maxWrite(0x0B, 7);   // scan limit 8 digits
  maxWrite(0x09, 0);   // no decode (raw segments)
  maxWrite(0x0A, 7);   // intensity
}



void getUltrasonicDistance() {
  digitalWrite(args[0], 0);
  delayMicroseconds(2);
  digitalWrite(args[0], 1);
  delayMicroseconds(10);
  digitalWrite(args[0], 0);
  uint32_t val = pulseIn(args[1], 1, 30000) * 340 / 1000;

  result.singleInt = (uint16_t)val / 2;
}

void switchFunction(uint8_t* functionId) {

  switch (*functionId) {
    case 1:
      {
        if (argCount == 1)
          maxInit();
        if (argCount == 2)
          maxWrite(args[0],args[1]);

        break;
      }

    case 2:
      {  // set pinmode
        if (argCount > 1)
          setPin();

        break;
      }

    case 3:
      {  // write pin
        if (argCount > 1)
          writePin();

        break;
      }

    case 4:
      {  // read 1 pin
        if (argCount == 1)
          read1Pin();

        break;
      }

    case 5:
      {  // read 8 pins
        if (argCount == 8)
          read8Pin();

        break;
      }

    case 7:
      {  // calculate Ultrasonic distance
        if (argCount == 2)
          getUltrasonicDistance();

        break;
      }

    case 8:
      {  // attach servo
          if (argCount == 2) {
            servo1.attach(args[0]);
            servo2.attach(args[1]);
          }

        break;
      }

    case 9:
      {  // control servo
        if (argCount == 2) {
          servo1.write(args[0]);
          servo2.write(args[1]);
        }

        break;
      }

    default:

      break;
  }
}

void receiveEvent(uint8_t receive) {
  if (receive < 1) return;

  uint8_t functionId = Wire.read();  // First byte is function number
  argCount = receive - 1;

  // Read all remaining bytes as arguments
  for (uint8_t i = 0; i < argCount && Wire.available(); i++) {
    args[i] = Wire.read();
  }
  returnSingleInt = true;

  switchFunction(&functionId);
}

void requestEvent() {
  if (returnSingleInt) {
    Wire.write((uint8_t*)&result.singleInt, (uint8_t)2);  // 2 byte
  } else {
    Wire.write((uint8_t*)&result.intArray, (uint8_t)8);  // 8 × byte = 8 bytes
  }
}

void setup() {
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);  // When master sends data
  Wire.onRequest(requestEvent);  // When master requests data
}

void loop() {
}