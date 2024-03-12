/* software serial / MDB 9 byte

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "freertos/queue.h"
#include "driver/rmt.h"

static const char *TAG = "SoftSerialRmt";

#define RX_CHANNEL RMT_CHANNEL_0
#define RMT_DIV 8
#define RMT_RX_IDLE_THRES 12000
#define RX_BIT_DIVIDER 1040
#define BIT_IN_WORD (10)  // start+8bit+stop 
// single rmt item
typedef struct {
    union {
        struct {
            uint16_t duration : 15; /*!< Duration of level */
            uint16_t level : 1;     /*!< Level  */
        };
        uint16_t val; /*!< Equivalent unsigned value for the RMT item */
    };
} rmt_item16_t;

esp_err_t soft_serial_init(gpio_num_t rx_pin, gpio_num_t tx_pin)
{
    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(rx_pin, RX_CHANNEL);
    rmt_rx_config.clk_div = RMT_DIV;
    rmt_rx_config.mem_block_num = 4;
    // rmt_rx_config.flags = RMT_CHANNEL_FLAGS_INVERT_SIG;
    rmt_rx_config.rx_config.idle_threshold = RMT_RX_IDLE_THRES;
    rmt_rx_config.rx_config.filter_en = 0;

    rmt_config(&rmt_rx_config);
    rmt_driver_install(RX_CHANNEL, 4096, 0);
    return ESP_OK;
}

esp_err_t soft_serial_deinit(void)
{
    return ESP_OK;
}

esp_err_t soft_serial_write_data(uint8_t *data, size_t count)
{
    return ESP_OK;
}

esp_err_t soft_serial_read_data(uint8_t *data, size_t count)
{
    size_t length = 0;
    // bool repeat = false;
    RingbufHandle_t rb = NULL;
    rmt_item16_t *items = NULL;
    uint8_t data[64] = {0};
    int cnt_bit=0; // wait start bit
    int cnt_byte=0;  // byte in packet
    rmt_get_ringbuf_handle(RX_CHANNEL, &rb);
    rmt_rx_start(RX_CHANNEL, true);
    while (1)
    {
        items = (rmt_item16_t *)xRingbufferReceive(rb, &length, portMAX_DELAY);
        if (items)
        {
            length /= 2; // one RMT = 2 Bytes
            for (int i = 0; i < length; i++)
            {
                int cnt_in_duration = items[i].duration/RX_BIT_DIVIDER;
                int lvl = items[i].level;
                ESP_LOGI(TAG, "%d lvl=%d, bit_in=%d,dur=%d",i, level,cnt_in_duration,items[i].duration);
                if(cnt_bit == 0){
                    if (lvl == 0 && cnt_in_duration > 0 && cnt_in_duration < BIT_IN_WORD) // start bit
                    {
                        data[cnt_byte] = 0; // first  bits in byte
                        cnt_bit = cnt_in_duration; // start bit + some bits=0
                    }
                    else
                    {
                        ESP_LOGE(TAG,"receive frame err START bit");
                    }
                } 
                else if(cnt_in_duration == 0) // last item -> stop bit -> lvl=1
                {
                    lvl=1; //??
                    for(;cnt_bit<BIT_IN_WORD-1;cnt_bit++)
                    {
                        data[cnt_byte] |= lvl<<7; // 8 with cmd or parity check //BIT_IN_WORD-3
                        data[cnt_byte] >>= 1;
                    }
                    ESP_LOGI(TAG,"last item cnt %d data %0x %c",cnt_byte,data[cnt_byte],data[cnt_byte]);
                }
                else 
                {
                    for(;(cnt_bit<BIT_IN_WORD-1) && (cnt_bit < cnt_in_duration) ;cnt_bit++)
                    {
                        data[cnt_byte] |= lvl<<7; // 8 with cmd or parity check //BIT_IN_WORD-3
                        data[cnt_byte] >>= 1;
                    }
                    if(cnt_bit >= BIT_IN_WORD-1) // all bits converted, next byte
                    {
                        ESP_LOGI(TAG,"last bit in byte cnt %d data %0x %c",cnt_byte,data[cnt_byte],data[cnt_byte]);
                        cnt_byte++;
                        cnt_bit = 0; // wait next start bit
                    }
                }
            }
        }
        // after parsing the data, return spaces to ringbuffer.
        ESP_LOGI(TAG,"all item converted");
        vRingbufferReturnItem(rb, (void *)items);
    }

    return ESP_OK;
}
