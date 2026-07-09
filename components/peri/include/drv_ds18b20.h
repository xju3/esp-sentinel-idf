#ifndef DRV_DS18B20_H_
#define DRV_DS18B20_H_

#include <stdbool.h>
#include <stdint.h>


#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file drv_ds18b20.h
 * @brief DS18B20 1-Wire digital temperature sensor driver.
 *
 * @note Board DQ is routed to GPIO16.
 */

#define DS18B20_PIN GPIO_NUM_16

#define DS18B20_CMD_SKIP_ROM          0xCC
#define DS18B20_CMD_CONVERT_T         0x44
#define DS18B20_CMD_READ_SCRATCHPAD   0xBE
#define DS18B20_CMD_WRITE_SCRATCHPAD  0x4E
#define DS18B20_CMD_COPY_SCRATCHPAD   0x48
#define DS18B20_CMD_RECALL_E2         0xB8
#define DS18B20_CMD_READ_POWER_SUPPLY 0xB4

typedef enum {
    DS18B20_RESOLUTION_9BIT = 0,
    DS18B20_RESOLUTION_10BIT = 1,
    DS18B20_RESOLUTION_11BIT = 2,
    DS18B20_RESOLUTION_12BIT = 3,
} ds18b20_resolution_t;

typedef void (*ds18b20_temp_cb_t)(float temperature_celsius);

esp_err_t drv_ds18b20_init(void);
esp_err_t drv_ds18b20_set_resolution(ds18b20_resolution_t resolution);
esp_err_t drv_ds18b20_start_conversion(void);
bool drv_ds18b20_is_conversion_done(void);
esp_err_t drv_ds18b20_read_temperature(float *temperature);
esp_err_t drv_ds18b20_read_temperature_async(ds18b20_temp_cb_t callback);
esp_err_t drv_ds18b20_self_test(void);
esp_err_t isolate_ds18b20_pin(void);
esp_err_t deisolate_ds18b20_pin(void);

extern bool g_ds18b20_initialized;

#ifdef __cplusplus
}
#endif

#endif /* DRV_DS18B20_H_ */
