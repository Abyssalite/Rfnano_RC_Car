#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#include <Arduino.h>
#include <Wire.h>

#ifndef SLAVE_ADDR
#define SLAVE_ADDR 8
#endif

// Generic low-level function
uint8_t masterCall(uint8_t func,
                   const uint8_t* args, uint8_t argLen,
                   uint8_t* resultBuffer, uint8_t requestLen);

// ----- Single value helpers -----
uint16_t masterCallUInt16(uint8_t func, const uint8_t* args = nullptr, uint8_t argLen = 0);

// ----- Array helpers -----
uint8_t masterCallUInt16Array(uint8_t func,
                             const uint8_t* args, uint8_t argLen,
                             uint16_t* results, uint8_t count);

uint8_t masterCallInt16Array(uint8_t func,
                             const uint8_t* args, uint8_t argLen,
                             int16_t* results, uint8_t count);

// ----- Unused -----

/*uint32_t masterCallUInt32(uint8_t func, const uint8_t* args = nullptr, uint8_t argLen = 0);

uint8_t masterCallUInt32Array(uint8_t func,
                             const uint8_t* args, uint8_t argLen,
                             uint32_t* results, uint8_t count);

uint8_t masterCallInt32Array(uint8_t func,
                             const uint8_t* args, uint8_t argLen,
                             int32_t* results, uint8_t count);
*/
#endif