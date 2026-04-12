#include <Wire.h>
#include <RF24.h>
#include <U8g2lib.h>

U8G2_SSD1309_128X64_NONAME0_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define CE_PIN  10
#define CSN_PIN  9
#define RF_PERIOD 30 // 30ms

RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "1Node";
unsigned long rfTimer = 0;

struct SendPayload {
    int8_t joystick1[2];
    int8_t joystick2[2];
    uint8_t digitalButton[6];
    uint8_t analogButton;
    //uint8_t batt;
};
SendPayload sendPayload;

struct ReceivePayload {
  uint8_t digitalIR[4];
  uint8_t analogIR[4];
  uint16_t tofSensors[3];
  uint16_t usSensors[2];
  int16_t gyro[3];
  uint8_t batt;
};
ReceivePayload receivePayload;

void updateDisplay() {
  u8g2.firstPage();                    // Start page mode
  do {
    // Line 1: Distance sensors
    u8g2.setCursor(0, 12);
    u8g2.print("Dist: ");
    u8g2.print(receivePayload.tofSensors[0]);
    u8g2.print(" ");
    u8g2.print(receivePayload.tofSensors[1]);
    u8g2.print(" ");
    u8g2.print(receivePayload.tofSensors[2]);

    // Line 2: Gyro
    u8g2.setCursor(0, 24);
    u8g2.print("Gyro: ");
    u8g2.print(receivePayload.gyro[0]);
    u8g2.print(" ");
    u8g2.print(receivePayload.gyro[1]);
    u8g2.print(" ");
    u8g2.print(receivePayload.gyro[2]);

    // Line 3: Battery
    u8g2.setCursor(0, 36);
    u8g2.print("Batt: ");
    u8g2.print(map(receivePayload.batt, 0, 255, 0, 1023));
    u8g2.print(" ");
    u8g2.print(analogRead(A7));

    // Line 4: Digital IR
    u8g2.setCursor(0, 48);
    u8g2.print("DigIR: ");
    u8g2.print(receivePayload.digitalIR[0]);
    u8g2.print(receivePayload.digitalIR[1]);
    u8g2.print(receivePayload.digitalIR[2]);
    u8g2.print(receivePayload.digitalIR[3]);

    // Line 5: Analog IR
    u8g2.setCursor(0, 60);
    u8g2.print("AnIR: ");
    u8g2.print(receivePayload.analogIR[0]);
    u8g2.print(" ");
    u8g2.print(receivePayload.analogIR[1]);
    u8g2.print(" ");
    u8g2.print(receivePayload.analogIR[2]);
    u8g2.print(" ");
    u8g2.print(receivePayload.analogIR[3]);

  } while (u8g2.nextPage());           // Render page by page
}

void setup() {
  // Button pins
  pinMode(0, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(200000); // use 200 kHz I2C
  Wire.setWireTimeout(40000, true);
  radio.begin();

  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(115);

  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.startListening();

  // Initialize OLED - Page mode
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

  // Low brightness settings
  u8g2.setContrast(30);
  u8g2.sendF("c", 0xD9, 0x22); 
  u8g2.sendF("c", 0xDB, 0x00);

  u8g2.firstPage();                    // Start page mode
  do {
    u8g2.setCursor(0, 36);
    u8g2.print("Batt: ");
    u8g2.print(analogRead(A7));

  } while (u8g2.nextPage());           // Render page by page
}

void loop() {
  // 1. RECEIVE PART
  if (radio.available()) {
    radio.read(&receivePayload, sizeof(receivePayload));
    updateDisplay();
  }

  // 2. TRANSMIT PART
  unsigned long now = millis();
  if (now - rfTimer >= RF_PERIOD) {
    rfTimer = now;
    radio.stopListening();

    sendPayload.joystick1[0] = constrain((map(analogRead(A0), 0, 884, -127, 127)), -127, 127);
    sendPayload.joystick1[1] = constrain((map(analogRead(A1), 0, 884, -127, 127)), -127, 127);
    sendPayload.joystick2[0] = constrain((map(analogRead(A2), 0, 884, -127, 127)), -127, 127);
    sendPayload.joystick2[1] = constrain((map(analogRead(A3), 0, 884, -127, 127)), -127, 127);

    sendPayload.digitalButton[0] = digitalRead(0);
    sendPayload.digitalButton[1] = digitalRead(2);
    sendPayload.digitalButton[2] = digitalRead(3);
    sendPayload.digitalButton[3] = digitalRead(4);
    sendPayload.digitalButton[4] = digitalRead(7);
    sendPayload.digitalButton[5] = digitalRead(8);

    sendPayload.analogButton = constrain((map(analogRead(A6), 0, 680, 0, 130)), 0, 130);
    //sendPayload.batt = map(analogRead(A7), 0, 1023, 0, 255);

    radio.write(&sendPayload, sizeof(sendPayload));
    radio.startListening();
  }
}
