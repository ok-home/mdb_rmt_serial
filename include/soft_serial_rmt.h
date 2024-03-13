#include "driver/gpio.h"

typedef struct
{
    union
    {
        struct
        {
            uint16_t data   : 8;    /*!< mdb data */
            uint16_t cmd    : 1;    /*!< mdb cmd  */
            uint16_t spare  : 7;    /*!< spare for future*/
        };
        uint16_t val; /*!< Equivalent unsigned value for the MDB item */
    };
} mdb_item16_t;

esp_err_t soft_serial_init(gpio_num_t rx_pin, gpio_num_t tx_pin);
esp_err_t soft_serial_deinit(void);
esp_err_t soft_serial_write_data(mdb_item16_t *data, size_t count);
esp_err_t soft_serial_read_data(mdb_item16_t *data, size_t count);