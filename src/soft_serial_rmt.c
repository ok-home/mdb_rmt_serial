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
#define RMT_RX_DIV (80)          // 8
#define RMT_RX_IDLE_THRES (2000) // 12000
#define RX_BIT_DIVIDER (100)     // 1040
#define RX_PULSE_EDGE_DELAY_COMPENSATION (25)

#define TX_CHANNEL RMT_CHANNEL_4
// tx baud=80000000/80/104 = 9615 baud
#define RMT_TX_DIV (80)      // 8 // esp32_mdb = 82
#define TX_BIT_DIVIDER (104) // 1042 // esp32_mdb=100

#define BIT_IN_WORD (11) // start+9bit+stop

static QueueHandle_t mdb_rx_packet_queue;
static QueueHandle_t mdb_tx_packet_queue;
static TaskHandle_t mdb_rx_packet_task_handle;
static TaskHandle_t mdb_tx_packet_task_handle;

#define MDB_TX_DONE_BIT BIT0
static EventGroupHandle_t mdb_tx_event_group;

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
static void mdb_rx_packet_task(void *p)
{
    size_t length = 0;
    RingbufHandle_t rb = NULL;
    rmt_item16_t *items = NULL;
    mdb_item16_t data = {0};
    mdb_packet_t packet = {0};
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
            gpio_set_level(RX_TEST_GPIO, 1);
#endif
            length /= 2; // one RMT = 2 Bytes
            for (int i = 0; i < length; i++)
            {
                int duration = (items[i].duration + RX_PULSE_EDGE_DELAY_COMPENSATION) / RX_BIT_DIVIDER;
                int lvl = items[i].level;
                // ESP_LOGI(TAG, "%d lvl=%d, bit_in=%d,dur=%d", i, lvl, duration, items[i].duration);
                if (cnt_bit == 0) // start bit
                {
                    if (lvl == 0 && duration > 0 && duration < BIT_IN_WORD) // start bit
                    {
                        data.val = 0;       // first  bits in byte
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
                        data.val |= lvl << (BIT_IN_WORD - 3); // 8 with cmd or parity check //BIT_IN_WORD-3
                    }
                    packet.packet_data[cnt_byte].val = data.val;
                    cnt_byte++;
                    cnt_bit = 0; // wait next start bit
                }
                else
                {
                    for (int j = 0; j < duration; cnt_bit++, j++)
                    {
                        data.val >>= 1;
                        data.val |= lvl << (BIT_IN_WORD - 3); // 8 with cmd or parity check //BIT_IN_WORD-3
                    }
                }
            }
        }
#if DBG
        gpio_set_level(RX_TEST_GPIO, 0);
#endif
        packet.packet_hdr.packet_size = cnt_byte;
        // ESP_LOGI(TAG, "all item converted %d byte ",cnt_byte);
        xQueueSend(mdb_rx_packet_queue, &packet, portMAX_DELAY);
        cnt_byte = 0;
        cnt_bit = 0; // wait next start bit
        memset(&packet, 0, sizeof(packet));
        // after parsing the data, return spaces to ringbuffer.
        vRingbufferReturnItem(rb, (void *)items);
    }
}
static void mdb_item_to_rmt_item_cvt(rmt_item16_t *rmt_data, mdb_item16_t data)
{
    int cnt = 0;
    rmt_data[cnt].level = 0; // start bit
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    for (; cnt < BIT_IN_WORD - 1; cnt++)
    {
        rmt_data[cnt].level = data.val & 1;
        rmt_data[cnt].duration = TX_BIT_DIVIDER;
        data.val >>= 1;
    }
    rmt_data[cnt].level = 1; // stop bit
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    rmt_data[cnt].level = 1; // end transfer
    rmt_data[cnt].duration = 0;
    cnt++;
    rmt_data[cnt].level = 1; // end transfer
    rmt_data[cnt].duration = 0;
}
static void mdb_tx_packet_task(void *p)
{
    rmt_item32_t rmt_item[8];
    rmt_item16_t *rmt_data = (rmt_item16_t *)rmt_item;
    int cnt = 0;
    mdb_packet_t packet = {0};
    while (1)
    {
        xQueueReceive(mdb_tx_packet_queue, &packet, portMAX_DELAY);
        for (cnt = 0; cnt < packet.packet_hdr.packet_size; cnt++)
        {
            mdb_item_to_rmt_item_cvt(rmt_data, packet.packet_data[cnt]);
            rmt_write_items(TX_CHANNEL, rmt_item, 8, 1); // start & wait done
        }
        xEventGroupSetBits(mdb_tx_event_group, MDB_TX_DONE_BIT);
    }
}
esp_err_t mdb_init(gpio_num_t rx_pin, gpio_num_t tx_pin)
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
    // rmt_tx_config.flags=RMT_CHANNEL_FLAGS_INVERT_SIG;
    //
    mdb_tx_event_group = xEventGroupCreate();
    mdb_tx_packet_queue = xQueueCreate(1, sizeof(mdb_packet_t));
    rmt_config(&rmt_tx_config);
    rmt_driver_install(TX_CHANNEL, 0, 0);
    xTaskCreate(mdb_tx_packet_task, "rmt tx", 4096, NULL, 5, &mdb_tx_packet_task_handle);

    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(rx_pin, RX_CHANNEL);
    rmt_rx_config.clk_div = RMT_RX_DIV;
    rmt_rx_config.mem_block_num = 4;
    rmt_rx_config.rx_config.idle_threshold = RMT_RX_IDLE_THRES;
    rmt_rx_config.rx_config.filter_en = 0;
    rmt_rx_config.rx_config.filter_en = true;
    rmt_rx_config.rx_config.filter_ticks_thresh = 5;
    //
    // rmt_rx_config.flags=RMT_CHANNEL_FLAGS_INVERT_SIG;
    //
    mdb_rx_packet_queue = xQueueCreate(4, sizeof(mdb_packet_t));
    rmt_config(&rmt_rx_config);
    rmt_driver_install(RX_CHANNEL, 4096, 0);
    xTaskCreate(mdb_rx_packet_task, "rmt rx", 4096, NULL, 5, &mdb_rx_packet_task_handle);

    return ESP_OK;
}
esp_err_t mdb_deinit(void)
{
    vTaskDelete(mdb_rx_packet_task_handle);
    rmt_driver_uninstall(RX_CHANNEL);
    vQueueDelete(mdb_rx_packet_queue);

    vTaskDelete(mdb_tx_packet_task_handle);
    rmt_driver_uninstall(TX_CHANNEL);
    vQueueDelete(mdb_tx_packet_queue);
    vEventGroupDelete(mdb_tx_event_group);

    return ESP_OK;
}
void mdb_tx_packet(mdb_packet_t *packet)
{
    xQueueSend(mdb_tx_packet_queue, packet, portMAX_DELAY); // data send to tx queue, start transmit
    xEventGroupWaitBits(mdb_tx_event_group,MDB_TX_DONE_BIT,pdTRUE,pdFALSE,portMAX_DELAY); // all data transmitted
}
esp_err_t mdb_rx_packet(mdb_packet_t *packet, TickType_t wait_time)
{
    if (xQueueReceive(mdb_rx_packet_queue, packet, wait_time) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}
void mdb_clear_rx_queue(void)
{
    xQueueReset(mdb_rx_packet_queue);
}
// hw reset mdb bus - 100 mS->Break, 200 ms->Setup 
void mdb_hw_reset(void)
{
    rmt_item32_t rmt_item[3];
    vTaskSuspend(mdb_tx_packet_task_handle);
    vTaskSuspend(mdb_rx_packet_task_handle);
    rmt_rx_stop(RX_CHANNEL);

    rmt_item[0].level0 = 0; 
    rmt_item[0].duration0 = 30000;
    rmt_item[0].level1 = 0; 
    rmt_item[0].duration1 = 30000;
    rmt_item[1].level0 = 0; 
    rmt_item[1].duration0 = 30000;
    rmt_item[1].level1 = 0; 
    rmt_item[1].duration1 = 30000;
    rmt_item[2].level0 = 1; 
    rmt_item[2].duration0 = 0;
    rmt_item[2].level1 = 1; 
    rmt_item[2].duration1 = 0;
    rmt_write_items(TX_CHANNEL, rmt_item, 3, 1); // set mdb bus to 0-level

    vTaskDelay(pdMS_TO_TICKS(250));                      // wait 200 mSek to setup mdb devices

    rmt_tx_memory_reset(TX_CHANNEL);
    xQueueReset(mdb_tx_packet_queue);
    vTaskResume(mdb_tx_packet_task_handle);

    rmt_rx_memory_reset(RX_CHANNEL);
    xQueueReset(mdb_rx_packet_queue);
    vTaskResume(mdb_rx_packet_task_handle);
    rmt_rx_start(RX_CHANNEL,true);
}
