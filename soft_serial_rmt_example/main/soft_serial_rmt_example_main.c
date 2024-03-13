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
#include <rom/ets_sys.h>

#include "logic_analyzer_ws_server.h"
#include "soft_serial_rmt.h"


#define UART_BAUD_RATE (9600)
#define TASK_STACK_SIZE (2048)

#define UART_PORT_NUM (1)
#define TEST_TXD (18)
#define TEST_RXD (19)

#define SEND_TEST_GPIO (25)
#define RECEIVE_TEST_GPIO (26)

#define BUF_SIZE (1024)

static const char *TAG = "UART TEST";


/* Configure parameters of an UART driver,
 * communication pins and install the driver */
static uart_config_t uart_config = {
    .baud_rate = UART_BAUD_RATE,
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

uint8_t send_data[] = "0123456789abcdefghijklmnopqrst";
uint8_t send_data1[] = "0";
uint8_t send_data2[] = "0123456789abcdefghijklmnopqrst___\n";
uint8_t send_data3[] = {
    0x1,0x2,0x4,0x8,0x10,0x20,0x40,0x80,
    0xfe,0xfd,0xfb,0xf7,0xef,0xdf,0xbf,0x7f,
    0xaa,0x55,0x0f,0xf0,0x00,0xff

/*    0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,
    0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,
    0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,
    0xaa,0xaa,0xaa,0xaa,0xaa,0xaa*/
};

static void sender_task(void *p)
{
    while (1)
    {
        vTaskDelay(200);
        ESP_LOGI(TAG,"-----");
        gpio_set_level(SEND_TEST_GPIO, 1);
        uart_write_bytes(UART_PORT_NUM, (const char *)send_data3, sizeof(send_data3));
 
        gpio_set_level(SEND_TEST_GPIO, 0);
    }
}
static void receiver_task(void *p)
{

    uint8_t data[128] = {0};
    int len;

    while (1)
    {
        gpio_set_level(RECEIVE_TEST_GPIO, 1);
        uart_read_bytes(UART_PORT_NUM, data, 1, portMAX_DELAY);
        gpio_set_level(RECEIVE_TEST_GPIO, 0);
        uart_get_buffered_data_len(UART_PORT_NUM, (size_t *)&len);
        gpio_set_level(RECEIVE_TEST_GPIO, 1);
        uart_read_bytes(UART_PORT_NUM, data + 1, len, portMAX_DELAY);
        gpio_set_level(RECEIVE_TEST_GPIO, 0);
        ESP_LOGI(TAG,"--%s",data);
    }
}

void app_main(void)

{
    logic_analyzer_ws_server();
    // esp_log_level_set("*", ESP_LOG_NONE);
    gpio_reset_pin(SEND_TEST_GPIO);
    gpio_set_direction(SEND_TEST_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(RECEIVE_TEST_GPIO);
    gpio_set_direction(RECEIVE_TEST_GPIO, GPIO_MODE_OUTPUT);

    soft_serial_init(TEST_TXD, TEST_RXD); // loopback

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TEST_TXD, TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));


    //gpio_set_direction(TEST_RXD, GPIO_MODE_INPUT_OUTPUT);


    //xTaskCreate(receiver_task, "receiver_task", TASK_STACK_SIZE * 2, NULL, 10, NULL);
    xTaskCreate(sender_task, "sender_task", TASK_STACK_SIZE * 2, NULL, 10, NULL);

    uint8_t data[128];    
    while(1){
        vTaskDelay(100);
        soft_serial_read_data(data,sizeof(send_data3) - 1);
        //ESP_LOGI(TAG,"%x",data);
        //soft_serial_write_data(data,sizeof(send_data) - 1);
    }

}
