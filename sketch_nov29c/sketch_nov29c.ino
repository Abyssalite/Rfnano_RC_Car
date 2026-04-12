#include <Wire.h>
#include <avr/interrupt.h>
#include <Arduino.h>

#define F_CPU 8000000L
#define SLAVE_ADDR 8
#define DIN 21
#define CS 20
#define CLK 4

// Store received data temporarily
volatile uint8_t args[8] = { 0 };  // max arguments
volatile uint8_t argCount = 0;
union {
  volatile uint16_t singleInt;
  volatile uint8_t intArray[8];
} result;
volatile bool returnSingleInt = true;

void initServoPWM() {
  DDRB |= (1 << PB1) | (1 << PB2); // pins 9,10

  // Fast PWM, non-inverting
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // prescaler 8

  ICR1 = 19999; // 20ms (50Hz at 8MHz)
}

void initTimer2() {
  DDRB |= (1 << PB3); // pins 11

  // CTC mode for tone generation
  TCCR2A = 0;
  TCCR2B = 0;
}

void setServo(uint8_t ch, uint8_t angle) {
  uint16_t us = map(constrain(angle, 0, 180), 0, 180, 500, 2500);

  if (ch == 9) OCR1A = us;
  else if (ch == 10) OCR1B = us;
}

void playTone(uint8_t pin, uint8_t value, uint8_t mul) {
  if (pin != 11) return;

  if (value <= 0 || mul <= 0) {
    stopTone();
    return;
  }
  uint16_t freq = value * mul;
  freq = constrain(freq, 0, 16000);
  uint16_t ocr = (F_CPU / (2UL * 256UL * freq)) - 1;

  if (ocr > 255) ocr = 255;

  OCR2A = (uint8_t)ocr;

  TCCR2A = (1 << COM2A0) | (1 << WGM21); // toggle CTC
  TCCR2B = (1 << CS22);                  // prescaler 256
}

void stopTone() {
  OCR2A = 0;
  TCCR2A = 0;
  TCCR2B = 0;
}

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
  for (uint8_t i = 1; i <= argCount - 1; i++)
    digitalWrite(args[i], value);

  result.singleInt = value;
}

void setPin() {
  bool value = args[0];
  for (uint8_t i = 1; i <= argCount - 1; i++)
    pinMode(args[i], value);

  result.singleInt = value;
}

void maxWrite(uint8_t reg, uint8_t val) {
  digitalWrite(CS, LOW);
  shiftOut(DIN, CLK, MSBFIRST, reg);
  shiftOut(DIN, CLK, MSBFIRST, val);
  digitalWrite(CS, HIGH);
}

void initDisp() {
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
      {  // control Servo
        if (argCount == 1)
          initServoPWM();
        if (argCount == 2)
          setServo(args[0], args[1]); 

        break;
      }

    case 9:
      {
        // control Display
        if (argCount == 1)
          initDisp();
        if (argCount == 2)
          maxWrite(args[0], args[1]);

        break;
      }

    case 10:
      {  // control PWM0
        if (argCount == 2)
          analogWrite(args[0], constrain(args[1], 0, 255));

        break;
      }

    case 11:
      {  // control Tone
        if (argCount == 1)
          initTimer2();
        if (argCount == 3)
          playTone(args[0], args[1], args[2]);

        break;
      }

    default:
      result.singleInt = 32001;

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
