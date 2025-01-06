#ifndef I2C_BRIDGE_AHT30_H
#define I2C_BRIDGE_AHT30_H

#include "common_driver.hpp"
#include <driver_aht30.h>


class Aht3x : public Sensor
{
    aht30_handle_t *aht30 {nullptr};
    uint32_t temp_raw{0};
    uint32_t humidity_raw{0};
    uint8_t humidity_percent{0};
    public:
    Aht3x();
    virtual ~Aht3x();
    virtual uint8_t begin();
    virtual uint8_t end();
    // virtual float temperature();
    // virtual float humidity();
    uint8_t t_and_h(float *t, float *h);
};

#endif