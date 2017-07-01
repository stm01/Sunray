// I2C helper

#ifndef I2C_H
#define I2C_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C"{
#endif

void I2C_writeToValue(uint8_t device, uint8_t address, uint8_t val);
void I2C_writeTo(uint8_t device, uint8_t address, int num, uint8_t buff[]);
int I2C_readFrom(uint8_t device, uint8_t address, uint8_t num, uint8_t buff[], int retryCount );
void I2C_reset();

#ifdef __cplusplus
}
#endif

#endif

