# RF-Nano RC Car

A 4-wheel drive remote-controlled car built around **RF-Nano** boards (Arduino Nano + integrated nRF24L01+).

The system uses two RF-Nano boards communicating over 2.4 GHz:

- **Master / Car** (`sketch_nov29a`) – controls motors, reads sensors, drives a pan-tilt head
- **Controller** (`sketch_nov29b`) – dual joysticks + buttons + OLED feedback
- **I²C Slave / Expander** (`sketch_nov29c`) – runs on an ATmega48 (or similar) and handles:
  - MAX7219 8-digit LED matrix display
  - Servos (pan/tilt head)
  - Ultrasonic sensors
  - IR sensors
  - Optional MPU6050 gyro
  - Extra digital/analog I/O

## Features

- Mecanum / tank-style 4WD motor control (SoftPWM)
- Dual analog joysticks + 6 digital buttons + analog button on the remote
- Real-time telemetry (ToF, ultrasonic, IR, battery, gyro)
- 8×8 MAX7219 matrix on the car for status / bars
- SSD1309 OLED on the remote for live sensor data
- Connection-loss failsafe (motors stop + spinner animation)
- Watchdog timers on all boards
- I²C master/slave architecture to expand I/O beyond the RF-Nano pins

## Hardware Overview

### Car (Master – RF-Nano)
- RF-Nano (CE=10, CSN=9 in this sketch)
- 4 DC motors via SoftPWM (pins A0/A1, 6/5, A2/A3, 4/3)
- 3× VL53L0X ToF sensors (XSHUT on 7, 0, 8)
- I²C connection to the ATmega48 expander
- Battery voltage monitoring on A6
- Servo pan/tilt head (controlled via I²C)

### Remote (Controller – RF-Nano)
- RF-Nano (CE=7, CSN=8)
- 2 analog joysticks (A0–A3)
- 6 digital buttons (pins 0,2,3,4,5,6)
- Analog button / pot (A7)
- Battery sense (A6)
- SSD1309 128×64 OLED (U8g2)

### Expander (ATmega48 – sketch_nov29c)
- I²C slave address `0x08`
- MAX7219 LED matrix (DIN=11, CS=12, CLK=13)
- 2× servos (pins 9 & 10)
- Ultrasonic sensors
- Extra digital/analog pins
- Optional MPU6050

See the included pinout images:
- `rf-nano_master.png`
- `rf-nano_controller.png`
- `nano_extender.png`
- `m48_extender.png`

## Software Structure

### Libraries Required

- **RF24** (nRF24L01)
- **SoftPWM**
- **VL53L0X**
- **U8g2** (for the OLED)
- **LedControl** (MAX7219)
- **Servo**
- **MPU6050_light** (optional)

## RF Configuration (both boards)

```cpp
radio.setAutoAck(false);
radio.setDataRate(RF24_250KBPS);
radio.setPALevel(RF24_PA_MAX);
radio.setChannel(115);
radio.setPayloadSize(32);
```
- Address used: "1Node"

## Building & Flashing

1. Flash sketch_nov29a to the car RF-Nano
2. Flash sketch_nov29b to the remote RF-Nano
3. Flash sketch_nov29c to the ATmega48 expander (example with USBasp):

```bash
avrdude -c usbasp -P /dev/ttyUSB0 -B 200 -p m48 -U flash:w:sketch_nov29c.ino.hex:i
```

## License

This project is currently unlicensed.