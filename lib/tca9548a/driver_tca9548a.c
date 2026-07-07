#include "driver_tca9548a.h"
#include <stdint.h>
#include <stddef.h>

#include <stdio.h>

static uint8_t update_channel( tca9548a_handle_t *handle);
static uint8_t read_channels(tca9548a_handle_t *handle);

/**
 * @brief     initialize the chip
 * @param[in] *handle points to a bmp280 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic or spi initialization failed
 *            - 2 handle is NULL
 *            - 3 linked functions is NULL
 *            - 4 id is error
 *            - 5 get nvm calibration failed
 *            - 6 read calibration failed
 * @note      none
 */
uint8_t tca9548a_init(tca9548a_handle_t *handle)
{
    if (!handle)
    {
        return TCA9548A_NULL_HANDLE;
    }

    if (handle->inited)
    {
        return TCA9548A_OK;
    }

    if (!handle->delay_ms)
    {
        return TCA9548A_LINKED_FUNC_NULL;
    }

    if (!handle->iic_read)
    {
        return TCA9548A_LINKED_FUNC_NULL;
    }

    if (!handle->iic_write)
    {
        return TCA9548A_LINKED_FUNC_NULL;
    }

    /* get currently configured channels */
    if (read_channels(handle))
    {
        return 1;
    }

    handle->device_channels = 0x00;

    /* try updating to new config */
    if (update_channel(handle))
    {
        return 1;
    }


    handle->inited = 1;

    return TCA9548A_OK;
}

/**
 * @brief     close the chip
 * @param[in] *handle points to a bmp280 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic deinit failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 power down failed
 * @note      none
 */
uint8_t tca9548a_deinit(tca9548a_handle_t *handle)
{
    if (!handle)
    {
        return TCA9548A_NULL_HANDLE;
    }

    if (!handle->inited)
    {
        return 3;
    }

    return TCA9548A_OK;
}

uint8_t tca9548a_set_addr_pin(tca9548a_handle_t *handle, tca9548a_address_t addr_pin)
{
    if (handle->inited)
    {
        return -1;
    }

    handle->addr = addr_pin;

    return TCA9548A_OK;
}

uint8_t tca9548a_get_addr_pin(const tca9548a_handle_t *handle, tca9548a_address_t *addr_pin)
{
    *addr_pin = handle->addr;

    return TCA9548A_OK;
}

uint8_t update_channel(tca9548a_handle_t *handle)
{
    /* udpate to new config or ignore if unchanged */
    if (handle->channels == handle->device_channels)
    {
        return TCA9548A_OK;
    }

    uint8_t ret = handle->iic_write(&handle->channels, 1);

    if (ret)
    {
        printf("unable to set device channels: %X\n", handle->channels);
        return ret;
    }

    handle->device_channels = handle->channels;

    return TCA9548A_OK;
}

static uint8_t read_channels(tca9548a_handle_t *handle)
{
    uint8_t ret = handle->iic_read(&handle->device_channels, 1);
    if (ret)
    {
        return ret;
    }

    return TCA9548A_OK;
}

uint8_t tca9548a_channel_open(tca9548a_handle_t *handle, tca9548a_channel_t channel)
{
    if (!handle->inited)
    {
        return -1;
    }

    handle->channels |= channel;

    return update_channel(handle);
}

uint8_t tca9548a_channel_open_all(tca9548a_handle_t *handle)
{
    return tca9548a_channel_set(handle, TCA9548A_CHANNEL_ALL);
}

uint8_t tca9548a_channel_close(tca9548a_handle_t *handle, tca9548a_channel_t channel)
{
    if (!handle->inited)
    {
        return -1;
    }

    handle->channels &= ~channel;

    return update_channel(handle);
}

uint8_t tca9548a_channel_close_all(tca9548a_handle_t *handle)
{
    return tca9548a_channel_set(handle, TCA9548A_CHANNEL_NONE);
}

uint8_t tca9548a_channel_set(tca9548a_handle_t *handle, tca9548a_channel_t channel)
{
    if (!handle->inited)
    {
        return -1;
    }

    handle->channels = channel;

    return update_channel(handle);
}

uint8_t tca9548a_channel_peek(tca9548a_handle_t *handle, tca9548a_channel_t *channel)
{
    uint8_t err = read_channels(handle);
    if (err)
    {
        return err;
    }

    *channel = handle->device_channels;

    return TCA9548A_OK;
}

uint8_t tca9548a_channel_get(const tca9548a_handle_t *handle, tca9548a_channel_t *channel)
{
    if (!handle->inited)
    {
        return -1;
    }

    *channel = handle->channels;

    return TCA9548A_OK;
}