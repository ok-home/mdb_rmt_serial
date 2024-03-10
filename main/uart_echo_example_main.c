/* UART Echo Example

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

#include "logic_analyzer_ws_server.h"
/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define ECHO_UART_BAUD_RATE (9600)
#define ECHO_UART_PORT_NUM (0)
#define ECHO_TASK_STACK_SIZE (2048)

#define SEND_UART_PORT_NUM (1)
#define SEND_TEST_TXD (18)
#define SEND_TEST_RXD (19)

#define RECEIVE_UART_PORT_NUM (2)
#define RECEIVE_TEST_TXD (21)
#define RECEIVE_TEST_RXD (18)

#define SEND_TEST_GPIO (25)
#define RECEIVE_TEST_GPIO (26)

static const char *TAG = "UART TEST";

#define BUF_SIZE (1024)

/* Configure parameters of an UART driver,
 * communication pins and install the driver */
static uart_config_t uart_config = {
    .baud_rate = ECHO_UART_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};
int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
static intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

uint8_t send_data[] = "0123456789abcdefghijklmnopqrst\n";
uint8_t send_data1[] = "0123456789abcdefghijklmnopqrst__\n";
uint8_t send_data2[] = "0123456789abcdefghijklmnopqrst___\n";
static void echo_task(void *arg)
{

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    while (1)
    {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, 1, portMAX_DELAY);
        // Write data back to the UART
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)data, len);
    }
}

static void sender_task(void *p)
{
    while (1)
    {
        gpio_set_level(SEND_TEST_GPIO, 1);
        uart_write_bytes(SEND_UART_PORT_NUM, (const char *)send_data, sizeof(send_data) - 1);
        uart_write_bytes(SEND_UART_PORT_NUM, (const char *)send_data1, sizeof(send_data1) - 1);
        uart_write_bytes(SEND_UART_PORT_NUM, (const char *)send_data2, sizeof(send_data2) - 1);
        gpio_set_level(SEND_TEST_GPIO, 0);

        vTaskDelay(100);
    }
}
static void receiver_task(void *p)
{

    uint8_t data[128] = {0};
    int len;

    while (1)
    {
        gpio_set_level(RECEIVE_TEST_GPIO, 1);
        uart_read_bytes(RECEIVE_UART_PORT_NUM, data, 1, portMAX_DELAY);
        gpio_set_level(RECEIVE_TEST_GPIO, 0);
        uart_get_buffered_data_len(RECEIVE_UART_PORT_NUM, (size_t *)&len);
        gpio_set_level(RECEIVE_TEST_GPIO, 1);
        uart_read_bytes(RECEIVE_UART_PORT_NUM, data + 1, len, portMAX_DELAY);

        gpio_set_level(RECEIVE_TEST_GPIO, 0);
        // ESP_LOGI(TAG,"%s",data);
    }
}

#define TIMER_RESOLUTION_HZ (10 * 1000 * 1000) // 1MHz resolution
#define TIMER_ALARM_PERIOD_S 0.0001            // Alarm period 0.5s
#define TIMER_ALARM_CNT 1042 / 3               // 1042
#define TDBG_PIN_1 32
#define TDBG_PIN_2 33

int timer_group = 0;
int timer_idx = 0;
int alarm_value = TIMER_ALARM_CNT;

QueueHandle_t tstQ;

TaskHandle_t task_dbg_handle;
static uint32_t cntr = 0;

int curr_lvl = 0;
int pre_lvl = 1;
int cnt_bit = 0;
int data_byte = 0;
int cl = 0;
int cnt_cl = 0;

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;

    // Send the event data back to the main program task
    gpio_set_level(TDBG_PIN_1, 1);
    curr_lvl = gpio_get_level(18) & 1;
    // cl=0;
    cnt_cl++;
    if (cnt_cl > 2)
    {
        cnt_cl = 0;
    }

    if (curr_lvl != pre_lvl)
    {
        pre_lvl = curr_lvl;
        cl = cnt_cl >= 2 ? 0 : cnt_cl + 1;
    }

    if (cnt_cl == cl)
    {
        switch (cnt_bit)
        {
        case 0:
            if (curr_lvl == 0)
            {
                cnt_bit++;
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
            data_byte >>= 1;
            data_byte |= curr_lvl << 7;
            cnt_bit++;
            break;
            //    case 9: //parity
            //        cnt_bit++;
            //        break;
        case 9: // stop
            gpio_set_level(TDBG_PIN_2, 1);
            // xTaskNotifyFromISR(task_dbg_handle,data_byte,eSetValueWithOverwrite,&high_task_awoken);
            xQueueSendFromISR(tstQ, &data_byte, &high_task_awoken);
            cnt_bit = 0;
            data_byte = 0;
            break;
        default:
            break;
        }
    }
    gpio_set_level(TDBG_PIN_1, 0);
    return high_task_awoken == pdTRUE; // return whether a task switch is needed
}

static void example_tg_timer_init(void)
{
    int group = timer_group;
    int timer = timer_idx;

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
    ESP_ERROR_CHECK(timer_init(group, timer, &config));

    // For the timer counter to a initial value
    ESP_ERROR_CHECK(timer_set_counter_value(group, timer, 0));
    // Set alarm value and enable alarm interrupt
    ESP_ERROR_CHECK(timer_set_alarm_value(group, timer, alarm_value));
    ESP_ERROR_CHECK(timer_enable_intr(group, timer));
    // Hook interrupt callback
    ESP_ERROR_CHECK(timer_isr_callback_add(group, timer, timer_group_isr_callback, NULL, 0));
    // Start timer
    ESP_ERROR_CHECK(timer_start(group, timer));
}

static void IRAM_ATTR task_dbg(void *p)
{
    uint32_t ch;
    xQueueReset(tstQ);
    while (1)
    {
        // xTaskNotifyWait(pdFALSE,pdFALSE,&ch,portMAX_DELAY);
        xQueueReceive(tstQ, &ch, portMAX_DELAY);
        putchar(ch);
        // printf("%lx %c\n",ch&0xff,(char)ch&0xff);
        gpio_set_level(TDBG_PIN_2, 0);
        // vTaskDelay(1);
    }
}

static void IRAM_ATTR task_start(void *p)
{
    ESP_LOGI(TAG, "Init timer with auto-reload");
    gpio_set_level(TDBG_PIN_1, 0);
    example_tg_timer_init();
    vTaskDelete(NULL);
}

void app_main(void)

{
    logic_analyzer_ws_server();
    // esp_log_level_set("*", ESP_LOG_NONE);
    gpio_reset_pin(SEND_TEST_GPIO);
    gpio_set_direction(SEND_TEST_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(RECEIVE_TEST_GPIO);
    gpio_set_direction(RECEIVE_TEST_GPIO, GPIO_MODE_OUTPUT);
    // xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);

    ESP_ERROR_CHECK(uart_driver_install(RECEIVE_UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(RECEIVE_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RECEIVE_UART_PORT_NUM, RECEIVE_TEST_TXD, RECEIVE_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(SEND_UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(SEND_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SEND_UART_PORT_NUM, SEND_TEST_TXD, SEND_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // uart_set_rx_full_threshold(RECEIVE_UART_PORT_NUM,8);
    uart_set_rx_timeout(RECEIVE_UART_PORT_NUM, 1);
    // gpio_set_direction(SEND_TEST_TXD,GPIO_MODE_INPUT_OUTPUT);

    gpio_reset_pin(TDBG_PIN_1);
    gpio_set_direction(TDBG_PIN_1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(TDBG_PIN_2);
    gpio_set_direction(TDBG_PIN_2, GPIO_MODE_OUTPUT);

    tstQ = xQueueCreate(1024, 4);
    xTaskCreatePinnedToCore(task_dbg, "dbg", 4096, NULL, 20, &task_dbg_handle, 0);
    xTaskCreatePinnedToCore(task_start, "start", 4096, NULL, 2, NULL, 1);

    xTaskCreate(receiver_task, "receiver_task", ECHO_TASK_STACK_SIZE * 2, NULL, 10, NULL);
    xTaskCreate(sender_task, "sender_task", ECHO_TASK_STACK_SIZE * 2, NULL, 10, NULL);
}
