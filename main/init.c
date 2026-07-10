#include "init.h"
#include "bsp_board.h"
#include "drv_4g.h"
#include "bsp_wifi.h"
#include "config_manager.h"
#include "drv_iis3dwb.h"
#include "drv_ds18b20.h"
#include "logger.h"
#include "task_daq.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>

static esp_err_t reset_sensor_power_rail(void)
{
    const gpio_config_t sensor_power_cfg = {
        .pin_bit_mask = 1ULL << BOARD_GPIO_SENSOR_EN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&sensor_power_cfg);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("Failed to configure sensor power rail: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(BOARD_GPIO_SENSOR_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static void init_sensors()
{
    (void)reset_sensor_power_rail();
    drv_iis3dwb_init();
    drv_ds18b20_init();
}

static esp_err_t enable_tasks()
{
    return ESP_OK;
}

esp_err_t init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ret = nvs_flash_erase();
        if (ret != ESP_OK)
        {
            return ret;
        }
        ret = nvs_flash_init();
    }
    return ret;
}

esp_err_t start_local_services()
{
    esp_err_t err = ESP_OK;

    init_sensors();

    err = enable_tasks();
    if (err != ESP_OK)
    {
        LOG_ERROR("Tasks initialization failed.");
        return err;
    }

    LOG_INFO("Local services ready without network.");
    return ESP_OK;
}
