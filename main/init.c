#include "init.h"
#include "bsp_4g.h"
#include "bsp_wifi.h"
#include "config_manager.h"
#include "drv_icm_42688_p.h"
#include "drv_lis2dh12.h"
#include "drv_ds18b20.h"
#include "drv_iis3dwb.h"
#include "logger.h"
#include "mqtt_proxy.h"
#include "data_dispatcher.h"
#include "task_daq.h"
#include "task_baseline.h"
#include "task_fft.h"
#include "task_rms.h"
#include "task_kurtosis.h"
#include "task_state_machine.h"
#include "task_envelope.h"
#include "wom_lis2dh12.h"
#include "web_server.h"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>

#ifndef IMU
#define IMU 1
#endif

static esp_err_t enable_tasks()
{
    esp_err_t ret = start_task_daq();
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = start_rms_task();
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = start_fft_task();
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = start_kurtosis_task();
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = start_envelope_task();
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Create the task that will handle state determination logic.
    // create_state_check_handler_task();

    // ret = start_wom_lis2dh12_listener();
    // if (ret != ESP_OK)
    // {
    //     // Avoid reboot loops when LIS2DH12 is absent/miswired.
    //     LOG_ERRORF("LIS2DH12 WoM listener disabled: %s", esp_err_to_name(ret));
    // }
    return ESP_OK;
}

static void network_channel_established_handler(void)
{
    esp_err_t err = init_mqtt_client();
    if (err != ESP_OK)
    {
        LOG_ERROR("MQTT proxy initialization failed.");
        return;
    }
    
    err = data_dispatcher_start();
    if (err != ESP_OK)
    {
        LOG_ERROR("Data dispatcher initialization failed.");
        return;
    }

    err = set_device_baseline(g_user_config.device_id);
    if (err != ESP_OK)
    {
        LOG_ERROR("Set device baseline failed.");
        return;
    }

    err = enable_tasks();
    if (err != ESP_OK)
    {
        LOG_ERROR("Tasks initialization failed.");
        return;
    }


#ifdef CONFIG_DEV_MODE // 开发模式下仍保留 Web 调试入口
    err = web_server_start();
    if (err != ESP_OK)
    {
        LOG_ERROR("Web server initialization failed.");
        return;
    }
#endif
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

static esp_err_t init_ds18b20()
{
    // 直接调用，不要传递任何参数
    esp_err_t ret = drv_ds18b20_init();

    if (ret != ESP_OK)
    {
        LOG_DEBUGF("DS18B20 初始化失败: %s\n", esp_err_to_name(ret));
    }
    LOG_INFO("DS18B20 初始化成功！");
    return ret;
}

esp_err_t establish_communication_channel()
{
    esp_err_t err = ESP_OK;
    if (g_user_config.network == 1)
    {
        LOG_INFO("Initializing 4G Module...");
        err = init_ppp_4g(network_channel_established_handler);
        if (err == ESP_OK) // Ensure ppp_4g_init returns esp_err_t
        {
            LOG_DEBUG("communication channel established by 4G.");
        }
        else
        {
            LOG_ERROR("4G Module initialization failed.");
        }
    }
    else
    {
        LOG_DEBUGF("ssid: %s, pass: %s", g_user_config.wifi.ssid, g_user_config.wifi.pass);
        err = wifi_init_sta(g_user_config.wifi.ssid, g_user_config.wifi.pass, network_channel_established_handler);
        if (err == ESP_OK)
        {
            LOG_DEBUG("communication channel established by WIFI.");
        } else {
            LOG_WARN("communication channel failed.");
        }
    }
    return err;
}


void enable_config_service()
{
    wifi_init_softap();
    esp_err_t ret = web_server_start();
    if (ret != ESP_OK)
    {
        return;
    }
}
