// #include "max31865.hpp"
// #include "common_driver.hpp"

// #include <driver_max31865.h>

// static uint8_t noop_spi(uint8_t x, uint8_t *xx, uint16_t xxx)
// {
//   return 0;
// }

// Max31865::~Max31865()
// {
//   if (max31865 == nullptr)
//   {
//     return;
//   }

//   end();

//   delete max31865;

//   max31865 = nullptr;
// }

// uint8_t Max31865::end()
// {
//   return max31865_deinit(max31865);
// }

// uint8_t Max31865::begin()
// {
//   max31865_deinit(max31865);

//   if (max31865_init(max31865) != 0)
//   {
//     return 1;
//   }

//   max31865_set_filter_select(max31865, MAX31865_FILTER_SELECT_50HZ);
//   max31865_set_resistor(max31865, MAX31865_RESISTOR_100PT);
//   max31865_set_reference_resistor(max31865, 430);
//   max31865_set_fault_detection_cycle_control(max31865, MAX31865_FAULT_DETECTION_CYCLE_CONTROL_AUTOMATIC_DELAY);
//   max31865_set_high_fault_threshold(max31865, 0xFFFE);
//   max31865_set_low_fault_threshold(max31865, 0);

//   return 0;
// }

// uint8_t Max31865::t_and_h(float *temp, float *pressure)
// {
//   uint16_t raw;
//   return max31865_single_read(max31865, &raw, temp);
// }

// Max31865::Max31865():
//   max31865(new max31865_handle_t)
// {
//   DRIVER_MAX31865_LINK_INIT(max31865, max31865_handle_t);
//   DRIVER_MAX31865_LINK_SPI_INIT(max31865, generic_interface_spi_init);
//   DRIVER_MAX31865_LINK_SPI_DEINIT(max31865, generic_interface_spi_deinit);
//   DRIVER_MAX31865_LINK_SPI_READ(max31865, generic_interface_spi_read);
//   DRIVER_MAX31865_LINK_SPI_WRITE(max31865, generic_interface_spi_write);
//   DRIVER_MAX31865_LINK_DELAY_MS(max31865, delay_ms);
//   DRIVER_MAX31865_LINK_DEBUG_PRINT(max31865, dummy_debug_print);
// }
