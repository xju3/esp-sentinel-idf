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
#include "esp_netif_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>
#include <time.h>

#ifndef IMU
#define IMU 1
#endif

#define NTP_SYNC_WAIT_TIMEOUT_MS 30000

static bool s_ntp_initialized = false;
static TaskHandle_t s_ntp_wait_task_handle = NULL;

static void ntp_time_sync_notification_cb(struct timeval *tv)
{
    time_t now = tv->tv_sec;
    struct tm utc_time = {0};
    char time_buf[32] = {0};

    gmtime_r(&now, &utc_time);
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S UTC", &utc_time);
    LOG_INFOF("NTP time synchronized: %s", time_buf);
}

static void ntp_sync_wait_task(void *arg)
{
    (void)arg;

    esp_err_t err = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(NTP_SYNC_WAIT_TIMEOUT_MS));
    if (err == ESP_OK)
    {
        LOG_INFO("NTP initial synchronization completed.");
    }
    else if (err == ESP_ERR_TIMEOUT)
    {
        LOG_WARN("NTP initial synchronization timed out.");
    }
    else
    {
        LOG_WARNF("NTP sync wait failed: %s", esp_err_to_name(err));
    }

    s_ntp_wait_task_handle = NULL;
    vTaskDelete(NULL);
}

static esp_err_t start_ntp_sync(void)
{
    esp_err_t err = ESP_OK;

    if (!s_ntp_initialized)
    {
        esp_sntp_config_t ntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(
            3,
            ESP_SNTP_SERVER_LIST("ntp.aliyun.com", "ntp1.aliyun.com", "pool.ntp.org"));
        ntp_cfg.wait_for_sync = true;
        ntp_cfg.sync_cb = ntp_time_sync_notification_cb;

        err = esp_netif_sntp_init(&ntp_cfg);
        if (err != ESP_OK)
        {
            return err;
        }

        s_ntp_initialized = true;
        LOG_INFO("NTP service started.");
    }
    else
    {
        err = esp_netif_sntp_start();
        if (err != ESP_OK)
        {
            return err;
        }
        LOG_INFO("NTP resynchronization triggered.");
    }

    if (s_ntp_wait_task_handle == NULL)
    {
        BaseType_t created = xTaskCreate(
            ntp_sync_wait_task,
            "ntp_sync_wait",
            3072,
            NULL,
            tskIDLE_PRIORITY + 1,
            &s_ntp_wait_task_handle);
        if (created != pdTRUE)
        {
            s_ntp_wait_task_handle = NULL;
            LOG_WARN("Failed to create NTP sync wait task.");
            return ESP_ERR_NO_MEM;
        }
    }

    return ESP_OK;
}

static void init_drivers()
{
#if IMU == 1
    drv_iis3dwb_init();
#else
    drv_ds18b20_init();
    drv_lis2dh12_init();
    drv_icm42688_init();
#endif
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
    create_state_check_handler_task();

    ret = start_wom_lis2dh12_listener();
    if (ret != ESP_OK)
    {
        // Avoid reboot loops when LIS2DH12 is absent/miswired.
        LOG_ERRORF("LIS2DH12 WoM listener disabled: %s", esp_err_to_name(ret));
    }
    return ESP_OK;
}

static void network_channel_established_handler(void)
{
    esp_err_t err = start_ntp_sync();
    if (err != ESP_OK)
    {
        LOG_WARNF("NTP synchronization startup failed: %s", esp_err_to_name(err));
    }

    init_drivers();
    err = init_mqtt_client();
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

void enable_config_service()
{
    wifi_init_softap();
    esp_err_t ret = web_server_start();
    if (ret != ESP_OK)
    {
        return;
    }
}
