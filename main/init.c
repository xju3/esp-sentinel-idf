#include "init.h"
#include "bsp_4g.h"
#include "bsp_wifi.h"
#include "config_manager.h"
#include "drv_icm_42688_p.h"
#include "drv_lis2dh12.h"
#include "logger.h"
#include "mqtt_proxy.h"
#include "task_daq.h"
#include "task_baseline.h"
#include "task_fft.h"
#include "task_rms.h"
#include "task_kurtosis.h"
#include "task_envelope.h"
#include "wom_lis2dh12.h"
#include "web_server.h"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>

bool is_network_available = false;

void enable_tasks()
{
    ESP_ERROR_CHECK(start_task_daq());
    ESP_ERROR_CHECK(start_rms_task());
    ESP_ERROR_CHECK(start_fft_task());
    ESP_ERROR_CHECK(start_kurtosis_task());
    ESP_ERROR_CHECK(start_envelope_task());
    ESP_ERROR_CHECK(start_wom_lis2dh12_listener());
}

esp_err_t init_imu_sensors()
{
    // ESP_ERROR_CHECK(drv_icm42688_init());
    // ESP_ERROR_CHECK(drv_lis2dh12_init());
    drv_lis2dh12_init();
    drv_icm42688_init();
    return ESP_OK;
}

esp_err_t init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

esp_err_t init_comm_channel()
{
    esp_err_t err = ppp_4g_init(); // Ensure ppp_4g_init returns esp_err_t
    if (g_user_config.network == 1)
    {
        LOG_INFO("Initializing 4G Module...");
        if (ppp_4g_init() == ESP_OK) // Ensure ppp_4g_init returns esp_err_t
        {
            is_network_available = true;
        }
        else
        {
            LOG_ERROR("4G Module initialization failed.");
        }
    }
    else
    {
        err = wifi_init_sta(g_user_config.wifi.ssid, g_user_config.wifi.pass, enable_tasks);
        if (err == ESP_OK)
        {
            is_network_available = true;
        }
    }
    return err;
}

esp_err_t enable_mqtt_proxy()
{
    if (is_network_available)
    {
        return mqtt_client_init();
    }
    LOG_ERROR("Cannot enable MQTT proxy: network channel is not enabled");
    return ESP_ERR_INVALID_STATE;
}

void enable_config_service()
{
    wifi_init_softap();
    ESP_ERROR_CHECK(web_server_start());
}