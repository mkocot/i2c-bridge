#ifndef I2C_BRIDGE_BANK_H
#define I2C_BRIDGE_BANK_H

#include <common_driver.hpp>

#include <Arduino.h>
#include <roo_collections/flat_small_hash_set.h>

typedef enum scl_bank_e : uint8_t
{
    SCL_BANK_0 = D2,
    SCL_BANK_1 = D3,
    SCL_BANK_2 = D5,
    SCL_BANK_3 = D6,
    SCL_BANK_4 = D7,
} scl_bank_t;

typedef enum sda_bank_e : uint8_t
{
    SDA_BANK_O = D4,
    SDA_BANK_1 = D4,
    SDA_BANK_2 = D4,
    SDA_BANK_3 = D4,
    SDA_BANK_4 = D4,
} sda_bank_t;

struct bank_t
{
    sda_bank_t sda : 4;
    scl_bank_t scl : 4;

    roo_collections::FlatSmallHashSet<Sensor *> sensors{};

    bank_t(sda_bank_t sda, scl_bank_t scl) : sda(sda), scl(scl)
    {
    }

    void begin();

    bool has(Sensor &sensor)
    {
        return sensors.find(&sensor) != sensors.end();
    }
};
#endif