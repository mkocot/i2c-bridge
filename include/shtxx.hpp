#ifndef I2C_BRIDGE_SHTXX_H
#define I2C_BRIDGE_SHTXX_H

#include <common_driver.hpp>
#include <Arduino.h>
#include <SHTSensor.h>

// 1364
// 21144
template <SHTSensor::SHTSensorType T>
class Shtxx : public Sensor
{
    SHTSensor mSensor{T};

public:
    virtual uint8_t begin()
    {
        if (mSensor.init())
        {
            return 0;
        }

        return 1;
    }
    virtual uint8_t end()
    {
        return 0;
    }
    virtual uint8_t t_and_h(float *t, float *h)
    {
        if (!mSensor.readSample())
        {
            return 1;
        }

        *t = mSensor.getTemperature();
        *h = mSensor.getHumidity();

        return 0;
    }

    virtual constexpr const SensorType sensor_id() const
    {
        switch (T)
        {
        case SHTSensor::SHT3X:
        case SHTSensor::SHT4X:
            return SHTxx;
        case SHTSensor::SHTC1:
        case SHTSensor::SHTC3:
            return SHTCx;
        default:
            return NONE;
        }
    }
};

#endif