#ifndef I2C_BRIDGE_AHT20_H
#define I2C_BRIDGE_AHT20_H

#include <driver_aht20.h>
#include <SoftWire.h>


void init_aht20();

void deinit_aht20();

uint8_t aht20_probe(SoftWire *sw);

uint8_t aht20_fetch(float *temp, float *hum);

#endif