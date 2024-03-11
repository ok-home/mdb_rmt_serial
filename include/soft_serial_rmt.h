#include "driver/gpio.h"

esp_err_t soft_serial_init(gpio_num_t rx_pin, gpio_num_t tx_pin);
esp_err_t soft_serial_deinit(void);
esp_err_t soft_serial_write_data(uint8_t *data, size_t count);
esp_err_t soft_serial_read_data(uint8_t *data, size_t count);