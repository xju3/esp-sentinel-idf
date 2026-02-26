#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "nvs_flash.h"
#include "core_wifi.h"
#include "ppp_4g.h"
#include "mqtt_proxy.h"
#include "config_manager.h"
#include "web_server.h"
#include "logger.h"
#include "sdkconfig.h"
#include "task_baseline.h"
#include "task_monitor.h"
#include "data_dispatcher.h"
#include "daq_icm_42688_p.h"

extern esp_err_t ppp_4g_init(void);

void enable_config_service()
{
    wifi_init_softap();
    ESP_ERROR_CHECK(web_server_start());
}

void start_tasks()
{
    esp_err_t err = ESP_OK;
    // 网络连接成功后，初始化 MQTT 客户端
    LOG_INFO("Network connected, initializing MQTT client...");
    err = mqtt_client_init();
    if (err != ESP_OK)
    {
        LOG_WARNF("MQTT client initialization failed: %d", err);
        // MQTT 初始化失败不影响设备运行，数据会存储在队列中等待网络恢复
    }
    else
    {
        LOG_INFO("MQTT client initialized successfully");
    }

    // 初始化硬件底层 (只调用一次)
    err = drv_icm42688_init();
    if (err != ESP_OK)
    {
        LOG_WARNF("ICM42688P initialization failed: %d", err);
        return;
    }

    // 初始化 data acquisition engine.
    err = daq_icm_42688_p_init();
    if (err != ESP_OK)
    {
        LOG_WARNF("System DAQ initialization failed: %d", err);
        return;
    }

    // 获取设备 ID
    const char *device_id = (g_user_config.device_id[0] != '\0') ? g_user_config.device_id : "default";
    // 获取设备的基准姿态
    err = set_device_baseline(1000, device_id);
    if (err != ESP_OK)
    {
        LOG_WARNF("Baseline capture failed: %d", err);
        return;
    }

    // 启动传感器监控任务
    err = task_monitor_start();
    if (err != ESP_OK)
    {
        LOG_ERRORF("Failed to start monitor task: %d", err);
        return;
    }
    
    // 启动消费者任务 (data_dispatcher)
    err = data_dispatcher_start();
    if (err != ESP_OK)
    {
        LOG_ERRORF("Failed to start data dispatcher: %d", err);
    }
    LOG_INFO("System initialization complete. Device is now monitoring vibrations.");
}

void app_main(void)
{
    // 初始化 NVS (Wi-Fi 驱动必须用到)
    LOG_INFO("starting device...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // 加载配置
    ESP_ERROR_CHECK(config_manager_load(&g_user_config));
    // 若未配置，启动 AP + Captive Portal
    if (!g_user_config.is_configured)
    {
        enable_config_service();
        return;
    }
    bool enable_netwokd_channel = false;
    esp_err_t err = ESP_OK;

    if (g_user_config.network == 1)
    {
        // 初始化 4G 模组
        LOG_INFO("Initializing 4G Module...");
        if (ppp_4g_init() == ESP_OK)
        {
            enable_netwokd_channel = true;
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
            enable_netwokd_channel = true;
        }
    }

    if (!enable_netwokd_channel)
    {
        enable_config_service();
        return;
    }

#ifdef CONFIG_DEV_MODE
    // 开发模式下仍保留 Web 调试入口
    ESP_ERROR_CHECK(web_server_start());
#endif
}
