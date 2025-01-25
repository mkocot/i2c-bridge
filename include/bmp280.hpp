#ifndef I2C_BRIDGE_BMP280_H
#define I2C_BRIDGE_BMP280_H

#include "common_driver.hpp"

#include <Arduino.h>
#include <driver_bmp280.h>

class Bmp280 : public Sensor
{
    bmp280_handle_t *bmp280{nullptr};
    uint32_t buffer{0};

public:
    Bmp280();
    virtual ~Bmp280();
    virtual uint8_t begin();
    virtual uint8_t end();
    // virtual float temperature();
    // virtual float humidity();
    uint8_t t_and_h(float *t, float *h);
    virtual constexpr const SensorType sensor_id() const
    {
        return BMP280;
    }
};
#endif