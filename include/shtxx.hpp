#ifndef I2C_BRIDGE_SHTXX_H
#define I2C_BRIDGE_SHTXX_H

#include <common_driver.hpp>
#include <Arduino.h>
#include <driver_sht4x.h>
#include <SHT31.h>

// 1364
// 21144
class Sht3x : public Sensor
{
    SHT31 mSht3x{};

public:
    Sht3x()
    {
    }
    ~Sht3x()
    {
    }

    virtual uint8_t t_and_h(float *t, float *h)
    {
        if (!mSht3x.read(false))
        {
            Serial.println("3x read failed");
            return 1;
        }
        *t = mSht3x.getTemperature();
        *h = mSht3x.getHumidity();
        return 0;
    }

    virtual uint8_t begin()
    {
        // sht3x
        // if (sht35_get_status)
        if (mSht3x.readStatus() != 0xFFFF)
        {
            Serial.println("Selected 1");

            return 0;
        }

        Serial.println("Unable to read 3x");

        return 1;
    }

    virtual uint8_t end()
    {
        return 0;
    }
};

class Sht4x : public Sensor
{
public:
    sht4x_handle_t *sht4x;

    union
    {
        struct
        {
            uint16_t temp_raw{0};
            uint16_t h_raw{0};
        };
        uint8_t serial[4];
    };
    uint8_t selected{0};

    Sht4x() : sht4x(new sht4x_handle_t)
    {
        DRIVER_SHT4X_LINK_INIT(sht4x, sht4x_handle_t);
        DRIVER_SHT4X_LINK_IIC_INIT(sht4x, dummy_uint8_t_no_op);
        DRIVER_SHT4X_LINK_IIC_DEINIT(sht4x, dummy_uint8_t_no_op);
        DRIVER_SHT4X_LINK_IIC_READ_COMMAND(sht4x, generic_i2c_read_cmd);
        DRIVER_SHT4X_LINK_IIC_WRITE_COMMAND(sht4x, generic_i2c_write_cmd);
        DRIVER_SHT4X_LINK_DELAY_MS(sht4x, delay_ms);
        DRIVER_SHT4X_LINK_DEBUG_PRINT(sht4x, dummy_debug_print);

        sht4x_set_addr(sht4x, SHT4X_ADDRESS_0);

        sht4x->inited = 1;
    }

    ~Sht4x()
    {
        delete sht4x;
    }

    virtual uint8_t t_and_h(float *t, float *h)
    {
        return sht4x_read(sht4x, SHT4X_MODE_HIGH_PRECISION_WITH_NO_HEATER, &temp_raw, t, &h_raw, h);
    }

    virtual uint8_t begin()
    {
        end();

        if (sht4x_get_serial_number(sht4x, serial))
        {
            Serial.println("Unable to read 4x");
            return 1;
        }

        Serial.println("Selected 2");

        return 0;
    }

    virtual uint8_t end()
    {
        // sht4x_deinit(sht4x);

        return 0;
    }
};
#endif