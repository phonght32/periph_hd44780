#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "periph_hd44780.h"

#define HD44780_I2C_ADDR					(0x27)

#define HD44780_WRITE_CMD_ERR_STR			"lcd write command error"
#define HD44780_WRITE_DATA_ERR_STR			"lcd write data error"
#define HD44780_READ_ERR_STR				"lcd read error"

#define mutex_lock(x)       				while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x)     				xSemaphoreGive(x)
#define mutex_create()      				xSemaphoreCreateMutex()
#define mutex_destroy(x)    				vQueueDelete(x)

#define VALIDATE_HD44780(periph, ret) if(!esp_periph_validate(periph, PERIPH_ID_HD44780)) {		\
	ESP_LOGE(TAG, "Invalid PERIPH_ID_HD44780");													\
	return ret;																					\
}

#define HD44780_CHECK(a, str, action) if(!(a)) {                            \
    ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);  \
    action;                                                                 \
}

typedef esp_err_t (*init_func)(hd44780_hw_info_t hw_info);
typedef esp_err_t (*write_func)(hd44780_hw_info_t hw_info, uint8_t data);
typedef esp_err_t (*read_func)(hd44780_hw_info_t hw_info, uint8_t *buf);
typedef void (*wait_func)(periph_hd44780_screen_handle_t screen_item);

typedef struct screen_item {
	hd44780_size_t 				size;
	hd44780_comm_mode_t 		comm_mode;
	hd44780_hw_info_t 			hw_info;
	write_func 					_write_cmd;
	write_func 					_write_data;
	wait_func 					_wait;
	SemaphoreHandle_t 			lock;
	STAILQ_ENTRY(screen_item) 	next;
} periph_hd44780_screen_t;

typedef struct periph_hd44780 {
	bool is_started;
	STAILQ_HEAD(screen_list, screen_item) list_screen;
} periph_hd44780_t;

static const char *TAG  = "PERIPH_HD44780";
static esp_periph_handle_t g_hd44780;

static esp_err_t _init_mode_4bit(hd44780_hw_info_t hw_info)
{
	uint64_t bit_mask = 0;
	bit_mask |= ((1ULL << hw_info.rs) | (1ULL << hw_info.en) | (1ULL << hw_info.d4) | (1ULL << hw_info.d5) | (1ULL << hw_info.d6) | (1ULL << hw_info.d7));
	if (hw_info.rw != -1) {
		bit_mask |= (1ULL << hw_info.rw);
	}

	gpio_config_t io_conf = {
		.intr_type = GPIO_PIN_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = bit_mask,
		.pull_down_en = 1,
		.pull_up_en = 0,
	};
	HD44780_CHECK(!gpio_config(&io_conf), "error config gpio", return ESP_FAIL);

	gpio_set_level(hw_info.rs, 0);
	gpio_set_level(hw_info.en, 0);
	gpio_set_level(hw_info.d4, 0);
	gpio_set_level(hw_info.d5, 0);
	gpio_set_level(hw_info.d6, 0);
	gpio_set_level(hw_info.d7, 0);
	if (hw_info.rw != -1) {
		gpio_set_level(hw_info.rw, 0);
	}

	return ESP_OK;
}

static esp_err_t _init_mode_8bit(hd44780_hw_info_t hw_info)
{
	return ESP_OK;
}

static esp_err_t _init_mode_serial(hd44780_hw_info_t hw_info)
{
	return ESP_OK;
}

static esp_err_t _write_cmd_4bit(hd44780_hw_info_t hw_info, uint8_t cmd)
{
	bool bit_data;
	uint8_t nibble_h = cmd >> 4 & 0x0F;
	uint8_t nibble_l = cmd & 0x0F;

	/* Set hw_info RS to write to command register */
	HD44780_CHECK(!gpio_set_level(hw_info.rs, false), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);

	if (hw_info.rw != -1) {
		HD44780_CHECK(!gpio_set_level(hw_info.rw, false), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);
	}

	/* Write high nibble */
	bit_data = (nibble_h >> 0) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d4, bit_data), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);
	bit_data = (nibble_h >> 1) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d5, bit_data), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);
	bit_data = (nibble_h >> 2) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d6, bit_data), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);
	bit_data = (nibble_h >> 3) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d7, bit_data), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);

	HD44780_CHECK(!gpio_set_level(hw_info.en, true), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	HD44780_CHECK(!gpio_set_level(hw_info.en, false), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);
	vTaskDelay(1 / portTICK_PERIOD_MS);

	bit_data = (nibble_l >> 0) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d4, bit_data), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);
	bit_data = (nibble_l >> 1) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d5, bit_data), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);
	bit_data = (nibble_l >> 2) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d6, bit_data), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);
	bit_data = (nibble_l >> 3) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d7, bit_data), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);

	HD44780_CHECK(!gpio_set_level(hw_info.en, true), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	HD44780_CHECK(!gpio_set_level(hw_info.en, false), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);
	vTaskDelay(1 / portTICK_PERIOD_MS);

	return ESP_OK;
}

static esp_err_t _write_cmd_8bit(hd44780_hw_info_t hw_info, uint8_t cmd)
{
	return ESP_OK;
}

static esp_err_t _write_cmd_serial(hd44780_hw_info_t hw_info, uint8_t cmd)
{
	uint8_t buf_send[4];
	buf_send[0] = (cmd & 0xF0) | 0x04;
	buf_send[1] = (cmd & 0xF0);
	buf_send[2] = ((cmd << 4) & 0xF0) | 0x04;
	buf_send[3] = ((cmd << 4) & 0xF0) | 0x08;

	i2cdev_set_addr(hw_info.i2cdev, HD44780_I2C_ADDR);
	HD44780_CHECK(!i2cdev_write(hw_info.i2cdev, 0, buf_send, sizeof(buf_send)), HD44780_WRITE_CMD_ERR_STR, return ESP_FAIL);

	return ESP_OK;
}

static esp_err_t _write_data_4bit(hd44780_hw_info_t hw_info, uint8_t data)
{
	bool bit_data;
	uint8_t nibble_h = data >> 4 & 0x0F;
	uint8_t nibble_l = data & 0x0F;

	/* Set hw_info RS to write to command register */
	HD44780_CHECK(!gpio_set_level(hw_info.rs, true), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);

	if (hw_info.rw != -1) {
		HD44780_CHECK(!gpio_set_level(hw_info.rw, false), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);
	}

	/* Write high nibble */
	bit_data = (nibble_h >> 0) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d4, bit_data), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);
	bit_data = (nibble_h >> 1) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d5, bit_data), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);
	bit_data = (nibble_h >> 2) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d6, bit_data), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);
	bit_data = (nibble_h >> 3) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d7, bit_data), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);

	HD44780_CHECK(!gpio_set_level(hw_info.en, true), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	HD44780_CHECK(!gpio_set_level(hw_info.en, false), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);
	vTaskDelay(1 / portTICK_PERIOD_MS);

	bit_data = (nibble_l >> 0) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d4, bit_data), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);
	bit_data = (nibble_l >> 1) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d5, bit_data), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);
	bit_data = (nibble_l >> 2) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d6, bit_data), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);
	bit_data = (nibble_l >> 3) & 0x01;
	HD44780_CHECK(!gpio_set_level(hw_info.d7, bit_data), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);

	HD44780_CHECK(!gpio_set_level(hw_info.en, true), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	HD44780_CHECK(!gpio_set_level(hw_info.en, false), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);
	vTaskDelay(1 / portTICK_PERIOD_MS);

	return ESP_OK;
}

static esp_err_t _write_data_8bit(hd44780_hw_info_t hw_info, uint8_t data)
{
	return ESP_OK;
}

static esp_err_t _write_data_serial(hd44780_hw_info_t hw_info, uint8_t data)
{
	uint8_t buf_send[4];
	buf_send[0] = (data & 0xF0) | 0x0D;
	buf_send[1] = (data & 0xF0) | 0x09;
	buf_send[2] = ((data << 4) & 0xF0) | 0x0D;
	buf_send[3] = ((data << 4) & 0xF0) | 0x09;

	i2cdev_set_addr(hw_info.i2cdev, HD44780_I2C_ADDR);
	HD44780_CHECK(!i2cdev_write(hw_info.i2cdev, 0, buf_send, sizeof(buf_send)), HD44780_WRITE_DATA_ERR_STR, return ESP_FAIL);

	return ESP_OK;
}

static esp_err_t _read_4bit(hd44780_hw_info_t hw_info, uint8_t *buf)
{
	uint64_t bit_mask = 0;
	bit_mask |= ((1ULL << hw_info.d4) | (1ULL << hw_info.d5) | (1ULL << hw_info.d6) | (1ULL << hw_info.d7));
	gpio_config_t io_conf = {
		.intr_type = GPIO_PIN_INTR_DISABLE,
		.mode = GPIO_MODE_INPUT,
		.pin_bit_mask = bit_mask,
		.pull_down_en = 1,
		.pull_up_en = 0,
	};
	HD44780_CHECK(!gpio_config(&io_conf), HD44780_READ_ERR_STR, return ESP_FAIL);

	bool bit_data;
	uint8_t nibble_h = 0, nibble_l = 0;

	HD44780_CHECK(!gpio_set_level(hw_info.en, true), HD44780_READ_ERR_STR, return ESP_FAIL);
	vTaskDelay(1 / portTICK_PERIOD_MS);

	bit_data = gpio_get_level(hw_info.d4);
	if (bit_data)
		nibble_h |= (1 << 0);
	bit_data = gpio_get_level(hw_info.d5);
	if (bit_data)
		nibble_h |= (1 << 1);
	bit_data = gpio_get_level(hw_info.d6);
	if (bit_data)
		nibble_h |= (1 << 2);
	bit_data = gpio_get_level(hw_info.d7);
	if (bit_data)
		nibble_h |= (1 << 3);
	HD44780_CHECK(!gpio_set_level(hw_info.en, false), HD44780_READ_ERR_STR, return ESP_FAIL);
	vTaskDelay(1 / portTICK_PERIOD_MS);

	HD44780_CHECK(!gpio_set_level(hw_info.en, true), HD44780_READ_ERR_STR, return ESP_FAIL);
	vTaskDelay(1 / portTICK_PERIOD_MS);

	bit_data = gpio_get_level(hw_info.d4);
	if (bit_data)
		nibble_l |= (1 << 0);
	bit_data = gpio_get_level(hw_info.d5);
	if (bit_data)
		nibble_l |= (1 << 1);
	bit_data = gpio_get_level(hw_info.d6);
	if (bit_data)
		nibble_l |= (1 << 2);
	bit_data = gpio_get_level(hw_info.d7);
	if (bit_data)
		nibble_l |= (1 << 3);
	HD44780_CHECK(!gpio_set_level(hw_info.en, false), HD44780_READ_ERR_STR, return ESP_FAIL);
	vTaskDelay(1 / portTICK_PERIOD_MS);

	io_conf.mode = GPIO_MODE_OUTPUT;
	HD44780_CHECK(!gpio_config(&io_conf), HD44780_READ_ERR_STR, return ESP_FAIL);

	*buf = ((nibble_h << 4) | nibble_l);

	return ESP_OK;
}

static esp_err_t _read_8bit(hd44780_hw_info_t hw_info, uint8_t *buf)
{
	return ESP_OK;
}

static void _wait_with_delay(periph_hd44780_screen_handle_t screen_item)
{
	vTaskDelay(2 / portTICK_PERIOD_MS);
}

static void _wait_with_pinrw(periph_hd44780_screen_handle_t screen_item)
{
	read_func _read;
	uint8_t temp_val;

	if (screen_item->comm_mode == HD44780_COMM_MODE_4BIT) {
		_read = _read_4bit;
	} else if (screen_item->comm_mode == HD44780_COMM_MODE_8BIT) {
		_read = _read_8bit;
	} else {
		_read = NULL;
	}

	while (1) {
		gpio_set_level(screen_item->hw_info.rs, false);
		gpio_set_level(screen_item->hw_info.rw, true);

		_read(screen_item->hw_info, &temp_val);
		if ((temp_val & 0x80) == 0)
			break;
	}
}

static init_func _get_init_func(hd44780_comm_mode_t comm_mode)
{
	if (comm_mode == HD44780_COMM_MODE_4BIT) {
		return _init_mode_4bit;
	} else if (comm_mode == HD44780_COMM_MODE_8BIT) {
		return _init_mode_8bit;
	} else {
		return _init_mode_serial;
	}

	return NULL;
}

static write_func _get_write_cmd_func(hd44780_comm_mode_t comm_mode)
{
	if (comm_mode == HD44780_COMM_MODE_4BIT) {
		return _write_cmd_4bit;
	} else if (comm_mode == HD44780_COMM_MODE_8BIT) {
		return _write_cmd_8bit;
	} else {
		return _write_cmd_serial;
	}

	return NULL;
}

static write_func _get_write_data_func(hd44780_comm_mode_t comm_mode)
{
	if (comm_mode == HD44780_COMM_MODE_4BIT) {
		return _write_data_4bit;
	} else if (comm_mode == HD44780_COMM_MODE_8BIT) {
		return _write_data_8bit;
	} else {
		return _write_data_serial;
	}

	return NULL;
}

static wait_func _get_wait_func(hd44780_hw_info_t hw_info)
{
	if (hw_info.rw == -1) {
		return _wait_with_delay;
	} else {
		return _wait_with_pinrw;
	}

	return NULL;
}

static esp_err_t _hd44780_init(esp_periph_handle_t self)
{
	VALIDATE_HD44780(self, ESP_FAIL);
	periph_hd44780_t *periph_hd44780 = esp_periph_get_data(self);
	if (!periph_hd44780->is_started) {
		periph_hd44780->is_started = true;
	}

	return ESP_OK;
}

static esp_err_t _hd44780_run(esp_periph_handle_t self, audio_event_iface_msg_t *msg)
{
	return ESP_OK;
}

static esp_err_t _hd44780_destroy(esp_periph_handle_t self)
{
	VALIDATE_HD44780(self, ESP_FAIL);
	periph_hd44780_t *periph_hd44780 = esp_periph_get_data(self);

	periph_hd44780_screen_t *screen_item = NULL;
	STAILQ_FOREACH(screen_item, &periph_hd44780->list_screen, next) {
		STAILQ_REMOVE(&periph_hd44780->list_screen, screen_item, screen_item, next);
		mutex_destroy(screen_item->lock);
		free(screen_item);
	}
	free(periph_hd44780);

	return ESP_OK;
}

esp_periph_handle_t periph_hd44780_init(periph_hd44780_cfg_t *config)
{
	HD44780_CHECK(config, "error config null", return NULL);

	esp_periph_handle_t periph = esp_periph_create(PERIPH_ID_HD44780, config->tag ? config->tag : "periph_hd44780");
	periph_hd44780_t *periph_hd44780 = calloc(1, sizeof(periph_hd44780_t));
	HD44780_CHECK(periph_hd44780, "error allocate memory", return NULL;);

	periph_hd44780->is_started = false;
	STAILQ_INIT(&periph_hd44780->list_screen);

	esp_periph_set_data(periph, periph_hd44780);
	esp_periph_set_function(periph, _hd44780_init, _hd44780_run, _hd44780_destroy);
	g_hd44780 = periph;

	return periph;
}

periph_hd44780_screen_handle_t periph_hd44780_screen_init(periph_hd44780_screen_cfg_t *config)
{
	HD44780_CHECK(g_hd44780, "error periph handle null", return NULL);
	HD44780_CHECK(config, "error config null", return NULL);

	periph_hd44780_t *periph_hd44780 = esp_periph_get_data(g_hd44780);
	periph_hd44780_screen_t *screen_item = calloc(1, sizeof(periph_hd44780_screen_t));
	HD44780_CHECK(screen_item, "error calloc screen item", return NULL);

	if (config->comm_mode == HD44780_COMM_MODE_SERIAL) {
		config->hw_info.rw = -1;
	}

	init_func _init_func;
	write_func _write_cmd;

	_init_func = _get_init_func(config->comm_mode);
	HD44780_CHECK(!_init_func(config->hw_info), "error init screen", {free(screen_item); return NULL;});

	_write_cmd = _get_write_cmd_func(config->comm_mode);

	HD44780_CHECK(!_write_cmd(config->hw_info, 0x02), "error init screen", {free(screen_item); return NULL;});
	vTaskDelay(10 / portTICK_PERIOD_MS);

	HD44780_CHECK(!_write_cmd(config->hw_info, 0x28), "error init screen", {free(screen_item); return NULL;});
	vTaskDelay(10 / portTICK_PERIOD_MS);

	HD44780_CHECK(!_write_cmd(config->hw_info, 0x06), "error init screen", {free(screen_item); return NULL;});
	vTaskDelay(10 / portTICK_PERIOD_MS);

	HD44780_CHECK(!_write_cmd(config->hw_info, 0x0C), "error init screen", {free(screen_item); return NULL;});
	vTaskDelay(10 / portTICK_PERIOD_MS);

	HD44780_CHECK(!_write_cmd(config->hw_info, 0x01), "error init screen", {free(screen_item); return NULL;});
	vTaskDelay(10 / portTICK_PERIOD_MS);

	screen_item->size = config->size;
	screen_item->comm_mode = config->comm_mode;
	screen_item->hw_info = config->hw_info;
	screen_item->_write_cmd = _write_cmd;
	screen_item->_write_data = _get_write_data_func(config->comm_mode);
	screen_item->_wait = _get_wait_func(config->hw_info);
	screen_item->lock = mutex_create();

	STAILQ_INSERT_TAIL(&periph_hd44780->list_screen, screen_item, next);

	return screen_item;
}

esp_err_t periph_hd44780_clear(periph_hd44780_screen_handle_t screen_item)
{
	HD44780_CHECK(screen_item, "error screen item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(screen_item->lock);
	int ret = screen_item->_write_cmd(screen_item->hw_info, 0x01);
	if (ret) {
		ESP_LOGE(TAG, "clear screen error");
		mutex_unlock(screen_item->lock);
		return ESP_FAIL;
	}
	screen_item->_wait(screen_item);
	mutex_unlock(screen_item->lock);

	return ESP_OK;
}

esp_err_t periph_hd44780_home(periph_hd44780_screen_handle_t screen_item)
{
	HD44780_CHECK(screen_item, "error screen item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(screen_item->lock);
	int ret = screen_item->_write_cmd(screen_item->hw_info, 0x02);
	if (ret) {
		ESP_LOGE(TAG, "set cursor to home error");
		mutex_unlock(screen_item->lock);
		return ESP_FAIL;
	}
	screen_item->_wait(screen_item);
	mutex_unlock(screen_item->lock);

	return ESP_OK;
}

esp_err_t periph_hd44780_write_char(periph_hd44780_screen_handle_t screen_item, uint8_t chr)
{
	HD44780_CHECK(screen_item, "error screen item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(screen_item->lock);
	int ret = screen_item->_write_data(screen_item->hw_info, chr);
	if (ret) {
		ESP_LOGE(TAG, "write char error");
		mutex_unlock(screen_item->lock);
		return ESP_FAIL;
	}
	mutex_unlock(screen_item->lock);

	return ESP_OK;
}

esp_err_t periph_hd44780_write_string(periph_hd44780_screen_handle_t screen_item, uint8_t *str)
{
	HD44780_CHECK(screen_item, "error screen item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(screen_item->lock);
	int ret;
	while (*str) {
		ret = screen_item->_write_data(screen_item->hw_info, *str);
		if (ret) {
			ESP_LOGE(TAG, "write string error");
			mutex_unlock(screen_item->lock);
			return ESP_FAIL;
		}
		str++;
	}
	mutex_unlock(screen_item->lock);

	return ESP_OK;
}

esp_err_t periph_hd44780_write_int(periph_hd44780_screen_handle_t screen_item, int number)
{
	HD44780_CHECK(screen_item, "error screen item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(screen_item->lock);

	int ret;

	if (number < 0) {
		ret = screen_item->_write_data(screen_item->hw_info, '-');
		if (ret) {
			ESP_LOGE(TAG, "write integer error");
			mutex_unlock(screen_item->lock);
			return ESP_FAIL;
		}
		number *= -1;
	}

	int num_digit = 1;
	int temp = number;

	while (temp > 9) {
		num_digit++;
		temp /= 10;
	}

	uint8_t buf[num_digit];
	sprintf((char*)buf, "%d", number);

	for (int i = 0; i < num_digit; i++) {
		ret = screen_item->_write_data(screen_item->hw_info, buf[i]);
		if (ret) {
			ESP_LOGE(TAG, "write integer error");
			mutex_unlock(screen_item->lock);
			return ESP_FAIL;
		}
	}

	mutex_unlock(screen_item->lock);

	return ESP_OK;
}

esp_err_t periph_hd44780_write_float(periph_hd44780_screen_handle_t screen_item, float number, uint8_t precision)
{
	HD44780_CHECK(screen_item, "error screen item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(screen_item->lock);

	int ret;

	if (number < 0) {
		ret = screen_item->_write_data(screen_item->hw_info, '-');
		if (ret) {
			ESP_LOGE(TAG, "write float error");
			mutex_unlock(screen_item->lock);
			return ESP_FAIL;
		}
		number *= -1;
	}

	int num_digit = 1;
	int temp = (int)number;

	while (temp > 9) {
		num_digit++;
		temp /= 10;
	}

	uint8_t buf[num_digit + 1 + precision];
	uint8_t float_format[7];

	sprintf((char*)float_format, "%%.%df", precision);
	sprintf((char*)buf, (const char*)float_format, number);

	for (int i = 0; i < (num_digit + 1 + precision); i++) {
		ret = screen_item->_write_data(screen_item->hw_info, buf[i]);
		if (ret) {
			ESP_LOGE(TAG, "write float error");
			mutex_unlock(screen_item->lock);
			return ESP_FAIL;
		}
	}

	mutex_unlock(screen_item->lock);

	return ESP_OK;
}

esp_err_t periph_hd44780_goto(periph_hd44780_screen_handle_t screen_item, uint8_t col, uint8_t row)
{
	HD44780_CHECK(screen_item, "error screen item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(screen_item->lock);

	int ret;
	if (row == 0) {
		ret = screen_item->_write_cmd(screen_item->hw_info, 0x80 + col);
		if (ret) {
			ESP_LOGE(TAG, "goto {row, col}: {%d, %d} error", row, col);
			mutex_unlock(screen_item->lock);
			return ESP_FAIL;
		}
	}
	else if (row == 1) {
		ret = screen_item->_write_cmd(screen_item->hw_info, 0xC0 + col);
		if (ret) {
			ESP_LOGE(TAG, "goto {row, col}: {%d, %d} error", row, col);
			mutex_unlock(screen_item->lock);
			return ESP_FAIL;
		}
	}
	else if (row == 2) {
		ret = screen_item->_write_cmd(screen_item->hw_info, 0x94 + col);
		if (ret) {
			ESP_LOGE(TAG, "goto {row, col}: {%d, %d} error", row, col);
			mutex_unlock(screen_item->lock);
			return ESP_FAIL;
		}
	}
	else {
		ret = screen_item->_write_cmd(screen_item->hw_info, 0xD4 + col);
		if (ret) {
			ESP_LOGE(TAG, "goto {row, col}: {%d, %d} error", row, col);
			mutex_unlock(screen_item->lock);
			return ESP_FAIL;
		}
	}

	mutex_unlock(screen_item->lock);

	return ESP_OK;
}

esp_err_t periph_hd44780_shift_cursor_forward(periph_hd44780_screen_handle_t screen_item, uint8_t step)
{
	HD44780_CHECK(screen_item, "error screen item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(screen_item->lock);

	int ret;
	for (uint8_t i = 0; i < step; i++) {
		ret = screen_item->_write_cmd(screen_item->hw_info, 0x14);
		if (ret) {
			ESP_LOGE(TAG, "shift cursor forward error");
			mutex_unlock(screen_item->lock);
			return ESP_FAIL;
		}
	}

	mutex_unlock(screen_item->lock);

	return ESP_OK;
}

esp_err_t periph_hd44780_shift_cursor_backward(periph_hd44780_screen_handle_t screen_item, uint8_t step)
{
	HD44780_CHECK(screen_item, "error screen item null", return ESP_ERR_INVALID_ARG);

	mutex_lock(screen_item->lock);

	int ret;
	for (uint8_t i = 0; i < step; i++) {
		ret = screen_item->_write_cmd(screen_item->hw_info, 0x10);
		if (ret) {
			ESP_LOGE(TAG, "shift cursor backward error");
			mutex_unlock(screen_item->lock);
			return ESP_FAIL;
		}
	}

	mutex_unlock(screen_item->lock);

	return ESP_OK;
}