#include "stdio.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "periph_hd44780.h"

esp_periph_handle_t hd44780_handle = NULL;
periph_hd44780_screen_handle_t screen_handle = NULL;
static i2cdev_handle_t i2cdev = NULL;

static esp_err_t periph_event_handle(audio_event_iface_msg_t *event, void *context)
{
    return ESP_OK;
}

void app_main(void)
{
    esp_periph_config_t config = {
        .event_handle = periph_event_handle,
        .max_parallel_connections = 9,
    };
    esp_periph_init(&config);

    i2cdev_cfg_t i2cdev_cfg = {
        .port = I2C_NUM_1,
        .mode  = I2C_MODE_MASTER,
        .sda_io_num = 5,
        .scl_io_num = 4,
        .clk_speed = 100000,
    };
    i2cdev = i2cdev_init(&i2cdev_cfg);

    periph_hd44780_cfg_t periph_hd44780_cfg = {
        .tag = NULL,
    };
    hd44780_handle = periph_hd44780_init(&periph_hd44780_cfg);

    hd44780_hw_info_t screen_hw_info = {
        .i2cdev = i2cdev,
    };

    periph_hd44780_screen_cfg_t screen_cfg = {
        .size = HD44780_SIZE_16_2,
        .comm_mode = HD44780_COMM_MODE_SERIAL,
        .hw_info = screen_hw_info,
    };
    screen_handle = periph_hd44780_screen_init(&screen_cfg);
    esp_periph_start(hd44780_handle);

    uint8_t *data = (uint8_t *)"PERIPH_HD44780";
    periph_hd44780_goto(screen_handle, 1, 0);
    periph_hd44780_write_string(screen_handle, data);

    while(1) 
    {
        vTaskDelay(100/portTICK_RATE_MS);
    }
}
