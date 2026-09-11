#include "I2CMaster.h"

// Generic function
uint8_t masterCall(uint8_t func,
                   const uint8_t* args, uint8_t argLen,
                   uint8_t* resultBuffer, uint8_t requestLen) {

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(func);

  if (args && argLen > 0 && argLen < 13) {
    Wire.write(args, argLen);
  }

  if (Wire.endTransmission(true) != 0) {
    return 1;   // transmission error
  }

  Wire.requestFrom((uint8_t)SLAVE_ADDR, requestLen);

  if (Wire.available() < requestLen) {
    return 2;   // not enough bytes received
  }

  Wire.readBytes(resultBuffer, requestLen);
  return 0;     // success
}

// Single value helpers
uint16_t masterCallUInt16(uint8_t func, const uint8_t* args, uint8_t argLen) {
  uint8_t buf[2];
  if (masterCall(func, args, argLen, buf, 2) != 0) return 0xFFFFUL;
  uint16_t value;
  memcpy(&value, buf, 2);
  return value;
}

// Array helpers
uint8_t masterCallUInt16Array(uint8_t func,
                             const uint8_t* args, uint8_t argLen,
                             uint16_t* results, uint8_t count) {
  if (count == 0 || count > 8 || results == nullptr) return 3;

  uint8_t buf[16];
  uint8_t requestLen = count * 2;

  if (masterCall(func, args, argLen, buf, requestLen) != 0) return 1;

  for (uint8_t i = 0; i < count; i++) {
    results[i] = (uint16_t)(buf[i * 2 + 1] << 8 | buf[i * 2]);
  }
  return 0;
}

uint8_t masterCallInt16Array(uint8_t func,
                             const uint8_t* args, uint8_t argLen,
                             int16_t* results, uint8_t count) {
  if (count == 0 || count > 8 || results == nullptr) return 3;

  uint8_t buf[16];
  uint8_t requestLen = count * 2;

  if (masterCall(func, args, argLen, buf, requestLen) != 0) return 1;

  for (uint8_t i = 0; i < count; i++) {
    results[i] = (int16_t)(buf[i * 2 + 1] << 8 | buf[i * 2]);
  }
  return 0;
}

// ----- Unused -----

/*uint32_t masterCallUInt32(uint8_t func, const uint8_t* args, uint8_t argLen) {
  uint8_t buf[4];
  if (masterCall(func, args, argLen, buf, 4) != 0) return 0xFFFFFFFFUL;
  uint32_t value;
  memcpy(&value, buf, 4);
  return value;
}

uint8_t masterCallInt32Array(uint8_t func,
                             const uint8_t* args, uint8_t argLen,
                             int32_t* results, uint8_t count) {
  if (count == 0 || count > 4 || results == nullptr) return 3;

  uint8_t buf[16];
  uint8_t requestLen = count * 4;

  if (masterCall(func, args, argLen, buf, requestLen) != 0) return 1;

  memcpy(results, buf, requestLen);
  return 0;
}

uint8_t masterCallUInt32Array(uint8_t func,
                             const uint8_t* args, uint8_t argLen,
                             uint32_t* results, uint8_t count) {
  if (count == 0 || count > 4 || results == nullptr) return 3;

  uint8_t buf[16];
  uint8_t requestLen = count * 4;

  if (masterCall(func, args, argLen, buf, requestLen) != 0) return 1;

  memcpy(results, buf, requestLen);
  return 0;
}
*/