#ifndef I2C_BRIDGE_AHT20_H
#define I2C_BRIDGE_AHT20_H

#include "common_driver.hpp"
#include <driver_aht20.h>

class Aht2x : public Sensor
{
    aht20_handle_t *aht20 {nullptr};
    uint32_t temp_raw{0};
    uint32_t humidity_raw{0};
    uint8_t humidity_percent{0};
    public:
    Aht2x();
    virtual ~Aht2x();
    virtual uint8_t begin();
    virtual uint8_t end();
    // virtual float temperature();
    // virtual float humidity();
    uint8_t t_and_h(float *t, float *h);

    virtual constexpr const SensorType sensor_id() const
    {
        return AHTxx;
    }
};

#endif