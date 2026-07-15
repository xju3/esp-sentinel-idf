#include "init.h"
#include "bsp_board.h"
#include "config_manager.h"
#include "drv_4g.h"
#include "drv_ds18b20.h"
#include "drv_iis3dwb.h"
#include "drv_lis2dh12.h"
#include "logger.h"
#include "off_sleep_manager.h"
#include "task_daq.h"
#include "wom_lis2dh12.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>

static esp_err_t reset_sensor_power_rail(void) {
  const gpio_config_t sensor_power_cfg = {
      .pin_bit_mask = 1ULL << BOARD_GPIO_SENSOR_EN,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t ret = gpio_config(&sensor_power_cfg);
  if (ret != ESP_OK) {
    LOG_ERRORF("Failed to configure sensor power rail: %s",
               esp_err_to_name(ret));
    return ret;
  }

  gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);
  vTaskDelay(pdMS_TO_TICKS(100));
  gpio_set_level(BOARD_GPIO_SENSOR_EN, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  return ESP_OK;
}

static void init_sensors() {
  (void)reset_sensor_power_rail();

  esp_err_t ds18b20_err = drv_ds18b20_init();
  if (ds18b20_err == ESP_OK) {
    ds18b20_err = drv_ds18b20_start_conversion();
  }
  if (ds18b20_err != ESP_OK) {
    LOG_WARNF("DS18B20 conversion could not be started during sensor init: %s",
              esp_err_to_name(ds18b20_err));
  }

#if LIS2
  drv_lis2dh12_init();
#endif
  drv_iis3dwb_init();
}

static esp_err_t enable_tasks() {
#if LIS2
  esp_err_t ret = start_off_sleep_manager();
  if (ret != ESP_OK) {
    return ret;
  }
#endif

  return ESP_OK;
}

esp_err_t init_nvs() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ret = nvs_flash_erase();
    if (ret != ESP_OK) {
      return ret;
    }
    ret = nvs_flash_init();
  }
  return ret;
}

esp_err_t start_local_services() {
  esp_err_t err = ESP_OK;

  init_sensors();

  err = enable_tasks();
  if (err != ESP_OK) {
    LOG_ERROR("Tasks initialization failed.");
    return err;
  }

  // LOG_INFO("Local services ready without network.");
  return ESP_OK;
}
