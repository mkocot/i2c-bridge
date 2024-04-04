#ifndef I2C_BRIDGE_BMP280_H
#define I2C_BRIDGE_BMP280_H

#include <Arduino.h>
#include <SoftWire.h>

void init_bmp280();

void deinit_bmp280();

uint8_t bmp280_probe(SoftWire *sw);

uint8_t bmp280_fetch(float *temp, float *pressure);

#endif