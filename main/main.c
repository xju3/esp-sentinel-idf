#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>

#include "bsp_wifi.h"
#include "bsp_4g.h"
#include "config_manager.h"
#include "data_dispatcher.h"
#include "daq_icm_42688_p.h"
#include "drv_lis2dh12.h"
#include "logger.h"
#include "mqtt_proxy.h"
#include "task_daq.h"
#include "task_fft.h"
#include "task_rms.h"
#include "web_server.h"
#include "wom_lis2dh12.h"

void enable_config_service()
{
    wifi_init_softap();
    ESP_ERROR_CHECK(web_server_start());
}

void start_tasks()
{
    ESP_ERROR_CHECK(start_task_daq());
    ESP_ERROR_CHECK(start_rms_task());
    ESP_ERROR_CHECK(start_fft_task());
    ESP_ERROR_CHECK(start_wom_lis2dh12_listener());
    // (void)esp_light_sleep_start();
}

void app_main(void)
{
    // 初始化 NVS (Wi-Fi 驱动必须用到)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(config_manager_load(&g_user_config));
    if (!g_user_config.is_configured)
    {
        enable_config_service();
        return;
    }
    bool enable_network_channel = false;
    esp_err_t err = ESP_OK;
    if (g_user_config.network == 1)
    {
        LOG_INFO("Initializing 4G Module...");
        if (ppp_4g_init() == ESP_OK) // Ensure ppp_4g_init returns esp_err_t
        {
            enable_network_channel = true;
        }
        else
        {
            LOG_ERROR("4G Module initialization failed.");
        }
    }
    else
    {
        err = wifi_init_sta(g_user_config.wifi.ssid, g_user_config.wifi.pass, start_tasks);
        if (err == ESP_OK)
        {
            enable_network_channel = true;
        }
    }

    if (!enable_network_channel)
    {
        enable_config_service();
        return;
    }
#ifdef CONFIG_DEV_MODE // 开发模式下仍保留 Web 调试入口
    ESP_ERROR_CHECK(web_server_start());
#endif
}
