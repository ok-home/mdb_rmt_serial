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
#include "esp_clk_tree.h"

static const char *TAG = "SoftSerial";

#define RMT_DIV 80
#define RMT_RX_IDLE_THRES 1000
#define RX_CHANNEL 0

esp_err_t soft_serial_init(gpio_num_t rx_pin, gpio_num_t tx_pin)
{

    size_t length = 0;
    bool repeat = false;
    RingbufHandle_t rb = NULL;
    rmt_item32_t *items = NULL;

    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(rx_pin, RX_CHANNEL);
    rmt_rx_config.clk_div = 80;
    rmt_config(&rmt_rx_config);
    rmt_driver_install(RX_CHANNEL, 1000, 0);
    rmt_get_ringbuf_handle(RX_CHANNEL, &rb);
    rmt_rx_start(RX_CHANNEL, true);
        while (1) {
        items = (rmt_item32_t *) xRingbufferReceive(rb, &length, portMAX_DELAY);
        if (items) {
            length /= 4; // one RMT = 4 Bytes
            for(int i=0;i<length;i++)
                ESP_LOGI(TAG,"lvl=%d,dur=%d,lvl=%d,dur=%d",items[i].level0,items[i].duration0,items[i].level1,items[i].duration1)
            }
            //after parsing the data, return spaces to ringbuffer.
            vRingbufferReturnItem(rb, (void *) items);
        }
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
    return ESP_OK;
}
