#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>
#include "driver/gpio.h"

#include "mqtt_proxy.h"
#include "config_manager.h"
#include "web_server.h"
#include "logger.h"
#include "bsp_wifi.h"
#include "bsp_4g.h"
#include "wom_icm_42688.h"
#include "wom_lis2dh12.h"
#include "data_dispatcher.h"
#include "daq_icm_42688_p.h"
#include "drv_lis2dh12.h"
#include "task_fft.h"
#include "task_daq.h"
#include "task_rms.h"

extern esp_err_t ppp_4g_init(void);

void config_light_sleep()
{
    // Configure LIS2DH12 Interrupt pins as wakeup sources
    // LIS2DH12 interrupts are Active High (configured in drv_lis2dh12.c)
    gpio_wakeup_enable(LIS2DH12_PIN_NUM_INT1, GPIO_INTR_HIGH_LEVEL);
    gpio_wakeup_enable(LIS2DH12_PIN_NUM_INT2, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_gpio_wakeup();
}


// 引用测试函数 (定义在 components/test/src/test_lis2dh12.c)
extern esp_err_t test_lis2dh12_patrol_run(void);

void enable_config_service()
{
    wifi_init_softap();
    ESP_ERROR_CHECK(web_server_start());
}

void start_tasks()
{
#ifdef CONFIG_DEV_MODE
    // 开发模式下仍保留 Web 调试入口
    // ESP_ERROR_CHECK(web_server_start());
#endif
    start_wom_lis2dh12_listener();
    
    // Configure wakeup sources before sleeping
    config_light_sleep();
    
    LOG_INFO("Entering light sleep...");
    esp_light_sleep_start();
    
    // Wakeup point: Check if we woke up due to WoM and handle missed edges
    LOG_INFO("Woke up from light sleep!");
    wom_lis2dh12_on_wakeup();
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
    // 加载配置
    ESP_ERROR_CHECK(config_manager_load(&g_user_config));
    // 若未配置，启动 AP + Captive Portal
    if (!g_user_config.is_configured)
    {
        enable_config_service();
        return;
    }
    bool enable_network_channel = false;
    esp_err_t err = ESP_OK;
    if (g_user_config.network == 1)
    {
        // 初始化 4G 模组
        LOG_INFO("Initializing 4G Module...");
        if (ppp_4g_init() == ESP_OK)
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
}
