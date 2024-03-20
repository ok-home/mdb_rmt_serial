#include "driver/gpio.h"

typedef struct
{
    union
    {
        struct
        {
            uint16_t data : 8;  /*!< mdb data */
            uint16_t cmd : 1;   /*!< mdb cmd  */
            uint16_t spare : 7; /*!< spare for future*/
        };
        uint16_t val; /*!< Equivalent unsigned value for the MDB item */
    };
} mdb_item16_t;
enum ackack_nak_ret
{
    MDB_ACK = 0,
    MDB_RET = 0xAA,
    MDB_NAK = 0xFF
};

typedef struct
{
    union
    {
        struct
        {
            uint16_t packet_size : 8;
            uint16_t spare : 8;
        };
        uint16_t value;
    };
} mdb_packet_hdr_t;

typedef struct
{
    mdb_packet_hdr_t packet_hdr;
    mdb_item16_t packet_data[39];
} mdb_packet_t;

esp_err_t mdb_init(gpio_num_t rx_pin, gpio_num_t tx_pin);
esp_err_t mdb_deinit(void);
esp_err_t mdb_tx_packet(mdb_packet_t *packet, TickType_t wait_time);
esp_err_t mdb_rx_packet(mdb_packet_t *packet, TickType_t wait_time);

#define DBG 1

#if DBG

#define TX_TEST_GPIO (25)
#define RX_TEST_GPIO (26)

#endif