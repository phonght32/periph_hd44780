#ifndef _PERIPH_HD44780_H_
#define _PERIPH_HD44780_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_peripherals.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "i2cdev.h"

typedef struct screen_item *periph_hd44780_screen_handle_t;

typedef enum {
	HD44780_SIZE_16_2 = 0,
	HD44780_SIZE_16_4,
	HD44780_SIZE_20_4,
	HD44780_SIZE_MAX,
} hd44780_size_t;

typedef enum {
	HD44780_COMM_MODE_4BIT = 0,
	HD44780_COMM_MODE_8BIT,
	HD44780_COMM_MODE_SERIAL,
	HD44780_COMM_MODE_MAX,
} hd44780_comm_mode_t;

typedef struct {
	int rs;
	int rw;
	int en;
	int d0;
	int d1;
	int d2;
	int d3;
	int d4;
	int d5;
	int d6;
	int d7;
	i2cdev_handle_t i2cdev;
} hd44780_hw_info_t;

typedef struct {
	hd44780_size_t 				size;
	hd44780_comm_mode_t 		comm_mode;
	hd44780_hw_info_t 			hw_info;
} periph_hd44780_screen_cfg_t;

typedef struct {
	const char *tag;
} periph_hd44780_cfg_t;

esp_periph_handle_t periph_hd44780_init(periph_hd44780_cfg_t *config);
periph_hd44780_screen_handle_t periph_hd44780_screen_init(periph_hd44780_screen_cfg_t *config);
esp_err_t periph_hd44780_clear(periph_hd44780_screen_handle_t screen_item);
esp_err_t periph_hd44780_home(periph_hd44780_screen_handle_t screen_item);
esp_err_t periph_hd44780_write_char(periph_hd44780_screen_handle_t screen_item, uint8_t chr);
esp_err_t periph_hd44780_write_string(periph_hd44780_screen_handle_t screen_item, uint8_t *str);
esp_err_t periph_hd44780_write_int(periph_hd44780_screen_handle_t screen_item, int number);
esp_err_t periph_hd44780_write_float(periph_hd44780_screen_handle_t screen_item, float number, uint8_t precision);
esp_err_t periph_hd44780_goto(periph_hd44780_screen_handle_t screen_item, uint8_t col, uint8_t row);
esp_err_t periph_hd44780_shift_cursor_forward(periph_hd44780_screen_handle_t screen_item, uint8_t step);
esp_err_t periph_hd44780_shift_cursor_backward(periph_hd44780_screen_handle_t screen_item, uint8_t step);


#ifdef __cplusplus
}
#endif

#endif /* _PERIPH_HD44780_H_ */