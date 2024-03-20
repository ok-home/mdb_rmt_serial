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

enum ack_nak_ret_enum
{
    PACKET_DATA = 0,
    PACKET_ACK  = 1,
    PACKET_RET  = 2,
    PACKET_NAK = 3
};

typedef struct
{
    struct
    {
        uint16_t packet_size    : 8;
        uint16_t ack_nak_ret    : 2;
        uint16_t no_crc         : 1;
        uint16_t crc_err        : 1;
        uint16_t spare          : 4;
    }
} struct mdb_packet_hdr_t;
typedef struct
{
    mdb_packet_hdr_t    hdr;
    mdb_item16_t        packet_data[40];
} mdb_packet_t;

esp_err_t soft_serial_init(gpio_num_t rx_pin, gpio_num_t tx_pin);
esp_err_t soft_serial_deinit(void);
esp_err_t soft_serial_write_data(mdb_item16_t *data, size_t count,TickType_t wait_time);
esp_err_t soft_serial_read_data(mdb_item16_t *data, size_t count,TickType_t wait_time);

#define DBG 1 

#if DBG 


#define TX_TEST_GPIO (25)
#define RX_TEST_GPIO (26)

#endif