# Simple exchange according to the MDB standard (ESP32)
  - 9-bit/9600 baud only, using RMT
  - maximum data packet size 36 mdb_item16_t records
## interface functions
```
esp_err_t mdb_init(gpio_num_t rx_pin, gpio_num_t tx_pin);   // init mdb/rmt task and queue
esp_err_t mdb_deinit(void);                                 // deinit mdb/rmt task and queue
void      mdb_tx_packet(mdb_packet_t *packet);              // send mdb packet ( return after all data transmitted )
esp_err_t mdb_rx_packet(mdb_packet_t *packet, TickType_t wait_time); // receive mdb packet
void      mdb_clear_rx_queue(void);                         // clear rx buffer
void      mdb_hw_reset(void);                               // hw reset mdb bus - 100 mS->Break, 200 ms->Setup 
  ```
## transmit/receive data format
```
// mdb item
typedef struct
{
    union
    {
        struct
        {
            uint16_t data : 8;  // mdb data 
            uint16_t cmd : 1;   // mdb cmd  
            uint16_t spare : 7; // spare for future
        };
        uint16_t val; // Equivalent unsigned value for the MDB item 
    };
} mdb_item16_t;
// mdb packet header
typedef struct 
{
    union
    {
        struct
        {
            uint16_t packet_size : 8; // transmit/receive packet size
            uint16_t spare : 8;       // spare for future  
        };
        uint16_t value; // Equivalent unsigned value for the MDB packet header
    };
} mdb_packet_hdr_t;
// mdb packet struct (header+items)
typedef struct
{
    mdb_packet_hdr_t packet_hdr; // mdb packet header
    mdb_item16_t packet_data[39];// mdb packet data
} mdb_packet_t;
```
## connects as a standard ESP-IDF component
## Simple example of use
  - soft_serial_rmt_example/main/soft_serial_rmt_example_main.c
  - for example, you need to short-circuit RMT_TX_GPIO and RMT_RX_GPIO
