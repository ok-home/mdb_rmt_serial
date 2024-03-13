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
#include <string.h>

#include "freertos/queue.h"
#include "driver/rmt.h"

static const char *TAG = "SoftSerialRmt";

#define RX_CHANNEL RMT_CHANNEL_0
#define RMT_RX_DIV 8
#define RMT_RX_IDLE_THRES 12000
#define RX_BIT_DIVIDER 1040

#define TX_CHANNEL RMT_CHANNEL_4
#define RMT_TX_DIV 8
#define TX_BIT_DIVIDER 1042

#define BIT_IN_WORD (10) // start+8bit+stop

static QueueHandle_t soft_serial_receive_queue;
static QueueHandle_t soft_serial_transmit_queue;
static TaskHandle_t receive_task_handle;
static TaskHandle_t transmit_task_handle;



// single rmt item
typedef struct
{
    union
    {
        struct
        {
            uint16_t duration : 15; /*!< Duration of level */
            uint16_t level : 1;     /*!< Level  */
        };
        uint16_t val; /*!< Equivalent unsigned value for the RMT item */
    };
} rmt_item16_t;

static void soft_serial_receive_task(void *p)
{
    size_t length = 0;
    // bool repeat = false;
    RingbufHandle_t rb = NULL;
    rmt_item16_t *items = NULL;
    uint8_t data = {0};
    int cnt_bit = 0;  // wait start bit
    int cnt_byte = 0; // byte in packet
    rmt_get_ringbuf_handle(RX_CHANNEL, &rb);
    rmt_rx_start(RX_CHANNEL, true);
    while (1)
    {
        items = (rmt_item16_t *)xRingbufferReceive(rb, &length, portMAX_DELAY);
        if (items)
        {
            gpio_set_level(26,1);
            length /= 2; // one RMT = 2 Bytes
            for (int i = 0; i < length; i++)
            {
                int cnt_in_duration = items[i].duration / RX_BIT_DIVIDER;
                int lvl = items[i].level;
                // ESP_LOGI(TAG, "%d lvl=%d, bit_in=%d,dur=%d", i, lvl, cnt_in_duration, items[i].duration);
                if (cnt_bit == 0) // start bit 
                {
                    if (lvl == 0 && cnt_in_duration > 0 && cnt_in_duration < BIT_IN_WORD) // start bit
                    {
                        data = 0;        // first  bits in byte
                        cnt_bit = cnt_in_duration; // start bit + some bits=0
                    }
                    else
                    {
                        ESP_LOGE(TAG, "receive frame err START bit");
                    }
                }
                else if (cnt_in_duration == 0 || (cnt_bit + cnt_in_duration) > (BIT_IN_WORD - 1)) // last item && stop bit 
                {
                    for (; cnt_bit < BIT_IN_WORD - 1; cnt_bit++)
                    {
                        data >>= 1;
                        data |= lvl << (BIT_IN_WORD-3); // 8 with cmd or parity check //BIT_IN_WORD-3
                    }
                    //ESP_LOGI(TAG, "byte decoded cnt %d data %0x ", cnt_byte, data);
                    xQueueSend(soft_serial_receive_queue,&data,portMAX_DELAY);
                    cnt_byte++;
                    cnt_bit = 0; // wait next start bit
                }
                else
                {
                    for (int j = 0; j < cnt_in_duration; cnt_bit++, j++)
                    {
                        data >>= 1;
                        data |= lvl << (BIT_IN_WORD-3); // 8 with cmd or parity check //BIT_IN_WORD-3
                    }
                }
            }
        }
        // after parsing the data, return spaces to ringbuffer.
        gpio_set_level(26,0);
        //ESP_LOGI(TAG, "all item converted %d byte ",cnt_byte);
        cnt_byte = 0;
        cnt_bit = 0; // wait next start bit
        vRingbufferReturnItem(rb, (void *)items);
    }
}
static void soft_serial_transmit_task(void *p)
{
    rmt_item32_t rmt_item[8] ;
    rmt_item16_t *rmt_data = (rmt_item16_t *)rmt_item;
    int cnt = 0;
    uint8_t data = 0;
    while(1)
    {
        xQueueReceive(soft_serial_transmit_queue, &data, portMAX_DELAY);
        memset((void*)rmt_data,0,sizeof(rmt_item));
        cnt = 0;
        rmt_data[cnt].level = 0; // start bit
        rmt_data[cnt].duration = TX_BIT_DIVIDER;
        cnt++;
        for(;cnt<BIT_IN_WORD-1;cnt++)
        {
            rmt_data[cnt].level = data&1; 
            rmt_data[cnt].duration = TX_BIT_DIVIDER;            
            data >>=1 ;
        }
        rmt_data[cnt].level = 1; // stop bit
        rmt_data[cnt].duration = TX_BIT_DIVIDER;
        cnt++;
        rmt_data[cnt].level = 1; // end transfer
        rmt_data[cnt].duration = 0;
        cnt++;
        rmt_data[cnt].level = 1; // end transfer
        rmt_data[cnt].duration = 0;
        cnt++;
        rmt_write_items(TX_CHANNEL, rmt_item, 8, 1);  //start & wait done

    }
}

esp_err_t soft_serial_init(gpio_num_t rx_pin, gpio_num_t tx_pin)
{
    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(rx_pin, RX_CHANNEL);
    rmt_rx_config.clk_div = RMT_RX_DIV;
    rmt_rx_config.mem_block_num = 4;
    rmt_rx_config.rx_config.idle_threshold = RMT_RX_IDLE_THRES;
    rmt_rx_config.rx_config.filter_en = 0;

    soft_serial_receive_queue = xQueueCreate(64, sizeof(mdb_item16_t));
    rmt_config(&rmt_rx_config);
    rmt_driver_install(RX_CHANNEL, 4096, 0);
    xTaskCreate(soft_serial_receive_task, "rmt rx", 4096, NULL, 5, &receive_task_handle);

    rmt_config_t rmt_tx_config = RMT_DEFAULT_CONFIG_TX(tx_pin, TX_CHANNEL);
    rmt_tx_config.clk_div = RMT_TX_DIV;
    rmt_tx_config.mem_block_num = 4;
    rmt_tx_config.tx_config.loop_count = 1;
    rmt_tx_config.tx_config.carrier_en = 0;
    rmt_tx_config.tx_config.loop_en = 0;
    rmt_tx_config.tx_config.idle_level = 1;
    rmt_tx_config.tx_config.idle_output_en = 1;

    soft_serial_transmit_queue = xQueueCreate(64, sizeof(mdb_item16_t));
    rmt_config(&rmt_tx_config);
    rmt_driver_install(TX_CHANNEL, 0, 0);
    xTaskCreate(soft_serial_transmit_task, "rmt tx", 4096, NULL, 5, &transmit_task_handle);

    return ESP_OK;
}

esp_err_t soft_serial_deinit(void)
{
    vTaskDelete(receive_task_handle);
    rmt_driver_uninstall(RX_CHANNEL);
    vQueueDelete(soft_serial_receive_queue);

    vTaskDelete(transmit_task_handle);
    rmt_driver_uninstall(TX_CHANNEL);
    vQueueDelete(soft_serial_transmit_queue);




    return ESP_OK;
}

esp_err_t soft_serial_write_data(uint8_t *data, size_t count)
{
    for (int i = 0; i < count; i++)
    {
        xQueueSend(soft_serial_transmit_queue, &data[i], portMAX_DELAY);
    }
    return ESP_OK;
}

esp_err_t soft_serial_read_data(uint8_t *data, size_t count)
{
    for (int i = 0; i < count; i++)
    {
        xQueueReceive(soft_serial_receive_queue, &data[i], portMAX_DELAY);
    }
    return ESP_OK;
}



