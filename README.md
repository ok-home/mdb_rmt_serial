# Simple exchange according to the MDB standard
  - 9-bit/9600 baud only, using RMT
  - maximum data packet size 36 mdb_item16_t records
## interface functions
  - esp_err_t soft_serial_init(gpio_num_t rx_pin, gpio_num_t tx_pin);
  - esp_err_t soft_serial_deinit(void);
  - esp_err_t soft_serial_write_data(mdb_item16_t *data, size_t count,TickType_t wait_time);
  - esp_err_t soft_serial_read_data(mdb_item16_t *data, size_t count,TickType_t wait_time);
## transmit/receive data format
```
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
```
### Instead of
  - soft_serial_write_data
  - soft_serial_read_data
### It's better to use direct read/write in the queue
  - QueueHandle_t soft_serial_receive_queue;
  - QueueHandle_t soft_serial_transmit_queue;

## connects as a standard ESP-IDF component
## Simple example of use
  - soft_serial_rmt_example/main/soft_serial_rmt_example_main.c
  - for example, you need to short-circuit RMT_TX_GPIO and RMT_RX_GPIO