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
#include "driver/timer.h"
#include "esp_clk_tree.h"

#undef SOFT_SERIAL_DBG

#define TIMER_RESOLUTION_HZ (10 * 1000 * 1000) // 10 MHz resolution
#define TIMER_ALARM_CNT 1042 / 3               // 9600 bod

#define TIMER_GROUP 0
#define TIMER_IDX 0

static int soft_serial_timer_group = TIMER_GROUP;
static int soft_serial_timer_idx = TIMER_IDX;
static int soft_serial_alarm_value = TIMER_ALARM_CNT;

static QueueHandle_t soft_serial_receive_queue;
static QueueHandle_t soft_serial_transmit_queue;
static gpio_num_t soft_serial_rx_gpio;
static gpio_num_t soft_serial_tx_gpio;

// static TaskHandle_t task_dbg_handle;

static int soft_serial_receive_curr_lvl = 0;
static int soft_serial_receive_pre_lvl = 1;
static int soft_serial_receive_bit_cnt = 0;
static int soft_serial_receive_data = 0;
static int soft_serial_curr_receive_div = 0;
static int soft_serial_cnt_div = 0;
static int soft_serial_curr_transmit_div = 0;
static int soft_serial_transmit_status = 0;
static int soft_serial_transmit_data = 0;
static int soft_serial_transmit_bit_cnt = 0;

// static const char *TAG = "SoftSerial";

static bool IRAM_ATTR soft_serial_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;

    soft_serial_receive_curr_lvl = gpio_get_level(soft_serial_rx_gpio) & 1;

    soft_serial_cnt_div++;
    if (soft_serial_cnt_div > 2)
    {
        soft_serial_cnt_div = 0;
    }

    if (soft_serial_receive_curr_lvl != soft_serial_receive_pre_lvl)
    {
        soft_serial_receive_pre_lvl = soft_serial_receive_curr_lvl;
        soft_serial_curr_receive_div = soft_serial_cnt_div >= 2 ? 0 : soft_serial_cnt_div + 1;
        soft_serial_curr_transmit_div = soft_serial_curr_receive_div >= 2 ? 0 : soft_serial_curr_receive_div + 1;
    }

    if (soft_serial_cnt_div == soft_serial_curr_receive_div) // receive serial data
    {
        switch (soft_serial_receive_bit_cnt)
        {
        case 0:
            if (soft_serial_receive_curr_lvl == 0)
            {
                soft_serial_receive_bit_cnt++;
            }
            break;
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
            soft_serial_receive_data >>= 1;
            soft_serial_receive_data |= soft_serial_receive_curr_lvl << 7;
            soft_serial_receive_bit_cnt++;
            break;
            //    case 9: //parity or CMD bit
            //        soft_serial_receive_bit_cnt++;
            //        break;
        case 9: // stop ( 9 or 10 )
            if (soft_serial_receive_curr_lvl == 1)
            {
                xQueueSendFromISR(soft_serial_receive_queue, &soft_serial_receive_data, &high_task_awoken);
            }
            soft_serial_receive_bit_cnt = 0;
            soft_serial_receive_data = 0;
            break;
        default:
            soft_serial_receive_bit_cnt = 0;
            soft_serial_receive_data = 0;
            break;
        }
    }
    if (soft_serial_cnt_div == soft_serial_curr_transmit_div) // transmit serial data
    {
        if (soft_serial_transmit_status == 0)
        {
            soft_serial_transmit_status = xQueueReceiveFromISR(soft_serial_transmit_queue, &soft_serial_transmit_data, &high_task_awoken);
            soft_serial_transmit_bit_cnt = 0;
        }
        else
        {
            switch (soft_serial_transmit_bit_cnt)
            {
            case 0: // send start bit
                gpio_set_level(soft_serial_tx_gpio, 0);
                soft_serial_transmit_bit_cnt++;
                break;
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
                int tmp = (soft_serial_transmit_data & 0xff) >> (soft_serial_transmit_bit_cnt - 1);
                gpio_set_level(soft_serial_tx_gpio, tmp & 1);
                soft_serial_transmit_bit_cnt++;
                break;
                //    case 9: //parity or CMD bit
                //        soft_serial_transmit_bit_cnt++;
                //        break;
            case 9: // stop (9 or 10 )
                gpio_set_level(soft_serial_tx_gpio, 1);
                soft_serial_transmit_bit_cnt = 0;
                soft_serial_transmit_status = 0;
                break;
            default:
                soft_serial_transmit_bit_cnt = 0;
                soft_serial_transmit_status = 0;
                break;
            }
        }
    }
    return high_task_awoken == pdTRUE; // return whether a task switch is needed
}

static void soft_serial_timer_init(void)
{

    uint32_t clk_src_hz = 0;
    ESP_ERROR_CHECK(esp_clk_tree_src_get_freq_hz((soc_module_clk_t)TIMER_SRC_CLK_DEFAULT, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &clk_src_hz));
    timer_config_t config = {
        .clk_src = TIMER_SRC_CLK_DEFAULT,
        .divider = clk_src_hz / TIMER_RESOLUTION_HZ,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 1,
    };
    ESP_ERROR_CHECK(timer_init(soft_serial_timer_group, soft_serial_timer_idx, &config));
    // For the timer counter to a initial value
    ESP_ERROR_CHECK(timer_set_counter_value(soft_serial_timer_group, soft_serial_timer_idx, 0));
    // Set alarm value and enable alarm interrupt
    ESP_ERROR_CHECK(timer_set_alarm_value(soft_serial_timer_group, soft_serial_timer_idx, soft_serial_alarm_value));
    ESP_ERROR_CHECK(timer_enable_intr(soft_serial_timer_group, soft_serial_timer_idx));
    // Hook interrupt callback
    ESP_ERROR_CHECK(timer_isr_callback_add(soft_serial_timer_group, soft_serial_timer_idx, soft_serial_isr_callback, NULL, 0));
    // Start timer
    ESP_ERROR_CHECK(timer_start(soft_serial_timer_group, soft_serial_timer_idx));
}

#ifdef SOFT_SERIAL_DBG
static void soft_serial_receive_dbg_task(void *p)
{
    uint32_t ch;
    while (1)
    {
        xQueueReceive(soft_serial_receive_queue, &ch, portMAX_DELAY);
        putchar(ch);
    }
}
#endif
#ifdef SOFT_SERIAL_DBG
static void soft_serial_transmit_dbg_task(void *p)
{
    char send_data[] = "0123456789qwertyuiopasdfghjklzxcvbnm\n";
    while (1)
    {
        for (int i = 0; i < strlen(send_data); i++)
            xQueueSend(soft_serial_transmit_queue, &send_data[i], portMAX_DELAY);
        vTaskDelay(20);
    }
}
#endif

static void soft_serial_start_task(void *p)
{
    soft_serial_timer_init();
    vTaskDelete(NULL);
}

esp_err_t soft_serial_init(gpio_num_t rx_pin, gpio_num_t tx_pin)
{
    soft_serial_rx_gpio = rx_pin;
    soft_serial_tx_gpio = tx_pin;

    soft_serial_receive_queue = xQueueCreate(1024, 2);
    soft_serial_transmit_queue = xQueueCreate(1024, 2);

    gpio_reset_pin(soft_serial_tx_gpio);
    gpio_set_direction(soft_serial_tx_gpio, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(soft_serial_tx_gpio, 1);

    gpio_reset_pin(soft_serial_rx_gpio);
    gpio_set_direction(soft_serial_rx_gpio, GPIO_MODE_INPUT);
    xTaskCreatePinnedToCore(soft_serial_start_task, "start", 4096, NULL, 2, NULL, 1);

#ifdef SOFT_SERIAL_DBG
    xTaskCreatePinnedToCore(soft_serial_receive_dbg_task, "dbg rx", 4096, NULL, 20, &task_dbg_handle, 0);
    xTaskCreatePinnedToCore(soft_serial_transmit_dbg_task, "dbg tx", 4096, NULL, 2, &task_dbg_handle, 0);
#endif

    return ESP_OK;
}

esp_err_t soft_serial_deinit(void)
{
    timer_pause(soft_serial_timer_group, soft_serial_timer_idx);
    timer_disable_intr(soft_serial_timer_group, soft_serial_timer_idx);
    timer_isr_callback_remove(soft_serial_timer_group, soft_serial_timer_idx);
    timer_deinit(soft_serial_timer_group, soft_serial_timer_idx);
    vQueueDelete(soft_serial_receive_queue);
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
