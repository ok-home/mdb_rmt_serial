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

#include "soft_serial_rmt.h"

static const char *TAG = "SoftSerialRmt";

/*
* Divider set
* 9600-1%+2% about 9500-9800 - MDB baud standart
* tx baud calculate 80 000 000 / RMT_TX_DIV / TX_BIT_DIVIDER
* rx bit divider should be a little more then max tx rate(9800) 
* RX_BIT_DIVIDER = ( 80 000 000/9800/RMT_RX_DIV ) = 100-102
* RMT_RX_IDLE_THRES may be  greater then (80 000 000/RX_BIT_DIVIDER/9500*11) = 1158(1200-2400)
*/

#define RX_CHANNEL RMT_CHANNEL_0
#define RMT_RX_DIV (80)   //8
#define RMT_RX_IDLE_THRES (2500)  //12000
#define RX_BIT_DIVIDER (100)  //1040

#define TX_CHANNEL RMT_CHANNEL_4
//tx baud=80000000/80/104 = 9615 baud
#define RMT_TX_DIV (80)  //8 // esp32_mdb = 82
#define TX_BIT_DIVIDER (104)  //1042 // esp32_mdb=100

#define BIT_IN_WORD (11) // start+9bit+stop

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
    RingbufHandle_t rb = NULL;
    rmt_item16_t *items = NULL;
    mdb_item16_t data = {0};
    int cnt_bit = 0;  // wait start bit, bit count
    int cnt_byte = 0; // byte in packet 
    rmt_get_ringbuf_handle(RX_CHANNEL, &rb);
    rmt_rx_start(RX_CHANNEL, true);
    while (1)
    {
        items = (rmt_item16_t *)xRingbufferReceive(rb, &length, portMAX_DELAY);
        if (items)
        {
            #if DBG
            gpio_set_level(RX_TEST_GPIO,1);
            #endif
            length /= 2; // one RMT = 2 Bytes
            for (int i = 0; i < length; i++)
            {
                int duration = (items[i].duration+25) / RX_BIT_DIVIDER;
                int lvl = items[i].level;
                 ESP_LOGI(TAG, "%d lvl=%d, bit_in=%d,dur=%d", i, lvl, duration, items[i].duration);
                if (cnt_bit == 0) // start bit 
                {
                    if (lvl == 0 && duration > 0 && duration < BIT_IN_WORD) // start bit
                    {
                        data.val = 0;        // first  bits in byte
                        cnt_bit = duration; // start bit + some bits=0
                    }
                    else
                    {
                        ESP_LOGE(TAG, "receive frame err START bit %d lvl=%d, bit_in=%d,dur=%d", i, lvl, duration, items[i].duration);
                    }
                }
                else if (duration == 0 || (cnt_bit + duration) > (BIT_IN_WORD - 1)) // last item && stop bit 
                {
                    for (; cnt_bit < BIT_IN_WORD - 1; cnt_bit++)
                    {
                        data.val >>= 1;
                        data.val |= lvl << (BIT_IN_WORD-3); // 8 with cmd or parity check //BIT_IN_WORD-3
                    }
                    ESP_LOGI(TAG, "byte decoded cnt %d data.val %0x ", cnt_byte, data.val);
                    xQueueSend(soft_serial_receive_queue,&data,portMAX_DELAY);
                    cnt_byte++;
                    cnt_bit = 0; // wait next start bit
                }
                else
                {
                    for (int j = 0; j < duration; cnt_bit++, j++)
                    {
                        data.val >>= 1;
                        data.val |= lvl << (BIT_IN_WORD-3); // 8 with cmd or parity check //BIT_IN_WORD-3
                    }
                }
            }
        }
        // after parsing the data, return spaces to ringbuffer.
        #if DBG
        gpio_set_level(RX_TEST_GPIO,0);
        #endif
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
    mdb_item16_t data = {0};
    while(1)
    {
        xQueueReceive(soft_serial_transmit_queue, &data, portMAX_DELAY);
        cnt = 0;
        rmt_data[cnt].level = 0; // start bit
        rmt_data[cnt].duration = TX_BIT_DIVIDER;
        cnt++;
        for(;cnt<BIT_IN_WORD-1;cnt++)
        {
            rmt_data[cnt].level = data.val&1; 
            rmt_data[cnt].duration = TX_BIT_DIVIDER;            
            data.val >>=1 ;
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

    rmt_config_t rmt_tx_config = RMT_DEFAULT_CONFIG_TX(tx_pin, TX_CHANNEL);
    rmt_tx_config.clk_div = RMT_TX_DIV;
    rmt_tx_config.mem_block_num = 4;
    rmt_tx_config.tx_config.loop_count = 1;
    rmt_tx_config.tx_config.carrier_en = 0;
    rmt_tx_config.tx_config.loop_en = 0;
    rmt_tx_config.tx_config.idle_level = 1;
    rmt_tx_config.tx_config.idle_output_en = 1;
    //
    //rmt_tx_config.flags=RMT_CHANNEL_FLAGS_INVERT_SIG;
    //
    soft_serial_transmit_queue = xQueueCreate(64, sizeof(mdb_item16_t));
    rmt_config(&rmt_tx_config);
    rmt_driver_install(TX_CHANNEL, 0, 0);
    xTaskCreate(soft_serial_transmit_task, "rmt tx", 4096, NULL, 5, &transmit_task_handle);

    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(rx_pin, RX_CHANNEL);
    rmt_rx_config.clk_div = RMT_RX_DIV;
    rmt_rx_config.mem_block_num = 4;
    rmt_rx_config.rx_config.idle_threshold = RMT_RX_IDLE_THRES;
    rmt_rx_config.rx_config.filter_en = 0;
    rmt_rx_config.rx_config.filter_en = true;
    rmt_rx_config.rx_config.filter_ticks_thresh = 5;
    //
    //rmt_rx_config.flags=RMT_CHANNEL_FLAGS_INVERT_SIG;
    //

    soft_serial_receive_queue = xQueueCreate(64, sizeof(mdb_item16_t));
    rmt_config(&rmt_rx_config);
    rmt_driver_install(RX_CHANNEL, 4096, 0);
    xTaskCreate(soft_serial_receive_task, "rmt rx", 4096, NULL, 5, &receive_task_handle);

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

esp_err_t soft_serial_write_data(mdb_item16_t *data, size_t count, TickType_t wait_time)
{
    for (int i = 0; i < count; i++)
    {
        if(xQueueSend(soft_serial_transmit_queue, &data[i], wait_time) != pdTRUE)
            { return ESP_ERR_TIMEOUT ;}
    }
    return ESP_OK;
}

esp_err_t soft_serial_read_data(mdb_item16_t *data, size_t count,TickType_t wait_time)
{
    for (int i = 0; i < count; i++)
    {
        if(xQueueReceive(soft_serial_receive_queue, &data[i], wait_time)!= pdTRUE)
        { return ESP_ERR_TIMEOUT ;}
    }
    return ESP_OK;
}



