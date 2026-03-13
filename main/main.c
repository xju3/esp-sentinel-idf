#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>

#include "init.h"
#include "config_manager.h"
#include "logger.h"
#include "task_baseline.h"
#include "web_server.h"

void app_main(void)
{
    // 初始化 NVS (Wi-Fi 驱动必须用到)
    ESP_ERROR_CHECK(init_nvs());
    ESP_ERROR_CHECK(config_manager_load(&g_user_config));
    if (!g_user_config.is_configured)
    {
        enable_config_service();
        return;
    }
    ESP_ERROR_CHECK(init_imu_sensors());
    ESP_ERROR_CHECK(set_device_baseline(100, g_user_config.device_id)); // 采样 100ms 来建立基线
    ESP_ERROR_CHECK(init_comm_channel());
}