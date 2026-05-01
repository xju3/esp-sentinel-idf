#include "init.h"
#include "bsp_4g.h"
#include "bsp_wifi.h"
#include "config_manager.h"
#include "drv_lis2dh12.h"
#include "drv_t1820b.h"
#include "drv_iis3dwb.h"
#include "logger.h"
#include "mqtt_proxy.h"
#include "data_dispatcher.h"
#include "task_daq.h"
#include "task_fft.h"
#include "task_diag_fusion.h"
#include "task_rms.h"
#include "task_kurtosis.h"
#include "task_envelope.h"
#include "off_sleep_manager.h"
#include "wom_lis2dh12.h"
#include "web_server.h"

#include "esp_err.h"
#include "driver/temperature_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>

static void init_drivers()
{
    drv_t1820b_init();
    drv_lis2dh12_init();
    drv_iis3dwb_init();
}

esp_err_t log_soc_temperature_once(void)
{
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);

    esp_err_t err = temperature_sensor_install(&temp_sensor_config, &temp_sensor);
    if (err != ESP_OK)
    {
        LOG_WARNF("Failed to install ESP32 temperature sensor: %s", esp_err_to_name(err));
        return err;
    }

    err = temperature_sensor_enable(temp_sensor);
    if (err != ESP_OK)
    {
        LOG_WARNF("Failed to enable ESP32 temperature sensor: %s", esp_err_to_name(err));
        temperature_sensor_uninstall(temp_sensor);
        return err;
    }

    float temperature_c = 0.0f;
    err = temperature_sensor_get_celsius(temp_sensor, &temperature_c);
    if (err == ESP_OK)
    {
        LOG_INFOF("ESP32-S3 startup die temperature: %.2f C", temperature_c);
    }
    else
    {
        LOG_WARNF("Failed to read ESP32 temperature sensor: %s", esp_err_to_name(err));
    }

    esp_err_t disable_err = temperature_sensor_disable(temp_sensor);
    if (disable_err != ESP_OK)
    {
        LOG_WARNF("Failed to disable ESP32 temperature sensor: %s", esp_err_to_name(disable_err));
        if (err == ESP_OK)
        {
            err = disable_err;
        }
    }

    esp_err_t uninstall_err = temperature_sensor_uninstall(temp_sensor);
    if (uninstall_err != ESP_OK)
    {
        LOG_WARNF("Failed to uninstall ESP32 temperature sensor: %s", esp_err_to_name(uninstall_err));
        if (err == ESP_OK)
        {
            err = uninstall_err;
        }
    }

    return err;
}

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
    ret = start_diag_fusion_task();
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

    ret = start_off_sleep_manager();
    if (ret != ESP_OK)
    {
        return ret;
    }

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
    LOG_INFO("Network services ready.");
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
        }
        else
        {
            LOG_WARN("communication channel failed.");
        }
    }
    return err;
}

esp_err_t start_local_services()
{
    esp_err_t err = ESP_OK;
    init_drivers();
    err = data_dispatcher_start();
    if (err != ESP_OK)
    {
        LOG_ERROR("Data dispatcher initialization failed.");
        return err;
    }
    err = enable_tasks();
    if (err != ESP_OK)
    {
        LOG_ERROR("Tasks initialization failed.");
        return err;
    }
    LOG_INFO("Local services ready without network.");
    return ESP_OK;
}

esp_err_t start_network_services()
{
    return establish_communication_channel();
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

esp_err_t disable_config_service(void)
{
    web_server_stop();

    esp_err_t err = wifi_stop_softap();
    if (err != ESP_OK)
    {
        LOG_WARNF("Failed to stop config Wi-Fi AP: %s", esp_err_to_name(err));
        return err;
    }

    LOG_INFO("Configuration service stopped.");
    return ESP_OK;
}
