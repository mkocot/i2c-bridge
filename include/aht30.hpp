#ifndef I2C_BRIDGE_AHT30_H
#define I2C_BRIDGE_AHT30_H

#include <driver_aht30.h>
#include <SoftWire.h>


void init_aht30();

void deinit_aht30();

uint8_t aht30_probe(SoftWire *sw);

uint8_t aht30_fetch(float *temp, float *hum);

#endif