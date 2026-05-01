#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>

#include "init.h"
#include "bsp_power.h"
#include "config_manager.h"
#include "logger.h"
#include "task_baseline.h"
#include "web_server.h"
#include "machine_state.h"
#include "data_dispatcher.h"
#include "startup_gate.h"

void app_main(void)
{
    // 初始化 NVS (Wi-Fi 驱动必须用到)
    init_nvs();
    ESP_ERROR_CHECK(bsp_power_init());
    init_machine_state();
    ESP_ERROR_CHECK(config_manager_load(&g_user_config));
    const bool had_valid_config_on_boot = g_user_config.is_configured;

    startup_gate_reset();
    startup_gate_set_waiting_for_config(true);
    enable_config_service();

    bool ap_client_connected = startup_gate_wait_for_ap_client(
        pdMS_TO_TICKS(CONFIG_SENTINEL_BOOT_AP_CONNECT_WINDOW_SEC * 1000U));
    if (ap_client_connected)
    {
        LOG_INFOF("AP client connected during boot window, waiting up to %d seconds for configuration submission.",
                  CONFIG_SENTINEL_BOOT_AP_CONFIG_WINDOW_SEC);

        bool config_completed = startup_gate_wait_for_config_completed(
            pdMS_TO_TICKS(CONFIG_SENTINEL_BOOT_AP_CONFIG_WINDOW_SEC * 1000U));
        if (config_completed)
        {
            ESP_ERROR_CHECK(config_manager_load(&g_user_config));
        }
        else
        {
            LOG_INFO("Configuration window expired.");
        }
    }
    else
    {
        LOG_INFO("No AP client connected during boot window.");
    }

    startup_gate_set_waiting_for_config(false);

    if (!g_user_config.is_configured)
    {
        if (had_valid_config_on_boot)
        {
            LOG_WARN("Device configuration became invalid during boot gate, staying in configuration mode.");
        }
        else
        {
            LOG_INFO("Device is not configured, staying in configuration mode.");
        }
        return;
    }

    ESP_ERROR_CHECK(bsp_power_prepare_boot_energy());
    ESP_ERROR_CHECK(start_local_services());
}
