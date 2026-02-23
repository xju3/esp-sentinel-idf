#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "nvs_flash.h"
#include "network_manager.h"
#include "config_manager.h"
#include "web_server.h"
#include "logger.h"
#include "sdkconfig.h"
#include "icm42688p_baseline.h"
#include "task_monitor.h"
#include "data_dispatcher.h"

extern esp_err_t ppp_4g_init(void);

void enable_config_service()
{
    wifi_init_softap();
    ESP_ERROR_CHECK(web_server_start());
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
        err = wifi_init_sta(g_user_config.wifi.ssid, g_user_config.wifi.pass);
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

    // 网络连接成功后，初始化 MQTT 客户端
    LOG_INFO("Network connected, initializing MQTT client...");
    err = mqtt_client_init();
    if (err != ESP_OK) {
        LOG_WARNF("MQTT client initialization failed: %d", err);
        // MQTT 初始化失败不影响设备运行，数据会存储在队列中等待网络恢复
    } else {
        LOG_INFO("MQTT client initialized successfully");
    }

    // 确保ICM42688P基线存在（按设备ID），结果写入全局 g_icm_baseline
    const char *device_id = (g_user_config.device_id[0] != '\0') ? g_user_config.device_id : "default";
    err = icm42688p_ensure_baseline(device_id, &g_icm_baseline);
    if (err == ESP_OK)
    {
        LOG_INFOF("Baseline X: val=%.5f off=%.5f, Y: val=%.5f off=%.5f, Z: val=%.5f off=%.5f",
                  g_icm_baseline.x.val, g_icm_baseline.x.offset,
                  g_icm_baseline.y.val, g_icm_baseline.y.offset,
                  g_icm_baseline.z.val, g_icm_baseline.z.offset);
        
        // 基线校准成功后，启动监控任务和数据分发任务
        LOG_INFO("Starting monitor and dispatcher tasks...");
        
        // 启动生产者任务 (task_monitor)
        err = task_monitor_start();
        if (err != ESP_OK) {
            LOG_ERRORF("Failed to start monitor task: %d", err);
        } else {
            LOG_INFO("Monitor task started successfully");
        }
        
        // 启动消费者任务 (data_dispatcher)
        err = data_dispatcher_start();
        if (err != ESP_OK) {
            LOG_ERRORF("Failed to start data dispatcher: %d", err);
        } else {
            LOG_INFO("Data dispatcher started successfully");
        }
        
        LOG_INFO("System initialization complete. Device is now monitoring vibrations.");
    }
    else
    {
        LOG_WARNF("Baseline ensure failed: %d", err);
        // 基线校准失败，仍启动任务（使用默认基线值）
        LOG_WARN("Starting tasks with default baseline values");
        
        task_monitor_start();
        data_dispatcher_start();
    }

#ifdef CONFIG_DEV_MODE
    // 开发模式下仍保留 Web 调试入口
    ESP_ERROR_CHECK(web_server_start());
#endif
}
