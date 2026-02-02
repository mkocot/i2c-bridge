#ifndef I2C_BRIDGE_MAX31865_H
#define I2C_BRIDGE_MAX31865_H

#include "common_driver.hpp"

#include <stdint.h>
#include <driver_max31865.h>

class Max31865 : public Sensor
{
    max31865_handle_t *max31865{nullptr};
    uint32_t buffer{0};

public:
    Max31865();
    virtual ~Max31865();
    virtual uint8_t begin();
    virtual uint8_t end();
    // virtual float temperature();
    // virtual float humidity();
    uint8_t t_and_h(float *t, float *h);
    virtual const SensorType sensor_id() const
    {
        return MAX31865;
    }
};
#endif