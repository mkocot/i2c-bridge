#ifndef DRIVER_TCA9548A
#define DRIVER_TCA9548A

#include <stdint.h>

typedef enum {
    TCA9548A_CHANNEL_NONE = 0x00,
    TCA9548A_CHANNEL_0 = 0x01,
    TCA9548A_CHANNEL_1 = 0x02,
    TCA9548A_CHANNEL_2 = 0x04,
    TCA9548A_CHANNEL_3 = 0x08,
    TCA9548A_CHANNEL_4 = 0x10,
    TCA9548A_CHANNEL_5 = 0x20,
    TCA9548A_CHANNEL_6 = 0x40,
    TCA9548A_CHANNEL_7 = 0x80,
    TCA9548A_CHANNEL_ALL = 0xFF,
} tca9548a_channel_t;

typedef enum tca9548a_error_e {
    TCA9548A_OK = 0,
    TCA9548A_SET_FAILED = 1,
    TCA9548A_NULL_HANDLE = 2,
    TCA9548A_LINKED_FUNC_NULL = 3,
    TCA9548A_IIC_INIT_FAILED = 4,
} tca9548a_error_t;

typedef enum tca9548a_address_e {
    TCA9548A_ADDRESS_DEFAULT = 0x70,
    TCA9548A_ADDRESS_A0 = TCA9548A_ADDRESS_DEFAULT + (1 << 0),
    TCA9548A_ADDRESS_A1 = TCA9548A_ADDRESS_DEFAULT + (1 << 1),
    TCA9548A_ADDRESS_A2 = TCA9548A_ADDRESS_DEFAULT + (1 << 2),
} tca9548a_address_t;

struct tca9548a_handle_s {
    uint8_t addr;
    uint8_t channels;
    uint8_t device_channels;
    uint8_t inited;
    uint8_t (*iic_read)(uint8_t *buf, uint16_t len);
    uint8_t (*iic_write)(uint8_t *buf, uint16_t len);
    void (*delay_ms)(uint32_t delay);
};

typedef struct tca9548a_handle_s tca9548a_handle_t;

#define DRIVER_TCA9548A_LINK_INIT(HANDLE)           memset(HANDLE, 0, sizeof(*HANDLE))
#define DRIVER_TCA9548A_LINK_IIC_READ(HANDLE, FUC)  (HANDLE)->iic_read = FUC
#define DRIVER_TCA9548A_LINK_IIC_WRITE(HANDLE, FUC) (HANDLE)->iic_write = FUC
#define DRIVER_TCA9548A_LINK_DELAY_MS(HANDLE, FUC)  (HANDLE)->delay_ms = FUC

/**
 * @brief     set the iic address pin configuration
 * @param[in] *handle points to a tca9548a handle structure
 * @param[in] addr_pin is the iic address pin
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t tca9548a_set_addr_pin(tca9548a_handle_t *handle, tca9548a_address_t addr_cfg);

/**
 * @brief      get the iic address pins configuration
 * @param[in]  *handle points to a tca9548a handle structure
 * @param[out] *addr_pin points to an iic address pin buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t tca9548a_get_addr_pin(const tca9548a_handle_t *handle, tca9548a_address_t *addr_cfg);

/*
 *            - 0 success
 *            - 1 set pressure oversampling failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 */
uint8_t tca9548a_init(tca9548a_handle_t *handle);

uint8_t tca9548a_deinit(tca9548a_handle_t *handle);

uint8_t tca9548a_channel_open(tca9548a_handle_t *handle, tca9548a_channel_t channel);

uint8_t tca9548a_channel_open_all(tca9548a_handle_t *handle);

uint8_t tca9548a_channel_close(tca9548a_handle_t *handle, tca9548a_channel_t channel);

uint8_t tca9548a_channel_close_all(tca9548a_handle_t *handle);

uint8_t tca9548a_channel_set(tca9548a_handle_t *handle, tca9548a_channel_t channel);

/**
 * @brief get mux configuration from device
 */
uint8_t tca9548a_channel_peek(tca9548a_handle_t *handle, tca9548a_channel_t *channel);

/**
 * @brief get expected configuration from device
 */
uint8_t tca9548a_channel_get(const tca9548a_handle_t *handle, tca9548a_channel_t *channel);

#endif