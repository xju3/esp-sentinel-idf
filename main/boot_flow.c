#include "boot_flow.h"

#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#include "config_manager.h"
#include "init.h"
#include "logger.h"
#include "startup_gate.h"

static void log_unconfigured_boot_state(bool had_valid_config_on_boot)
{
    if (had_valid_config_on_boot)
    {
        LOG_WARN("Device configuration became invalid during boot gate, staying in configuration mode.");
    }
    else
    {
        LOG_INFO("Device is not configured, staying in configuration mode.");
    }
}

static void wait_for_boot_configuration_window(void)
{
    bool ap_client_connected = startup_gate_wait_for_ap_client(
        pdMS_TO_TICKS(CONFIG_SENTINEL_BOOT_AP_CONNECT_WINDOW_SEC * 1000U));
    if (!ap_client_connected)
    {
        LOG_INFO("No AP client connected during boot window.");
        return;
    }

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

esp_err_t boot_flow_prepare(bool *ready_to_start_local_services)
{
    if (ready_to_start_local_services == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *ready_to_start_local_services = false;

    esp_err_t err = config_manager_load(&g_user_config);
    if (err != ESP_OK)
    {
        return err;
    }
    const bool had_valid_config_on_boot = g_user_config.is_configured;

    startup_gate_reset();
    startup_gate_set_waiting_for_config(true);
    enable_config_service();

    wait_for_boot_configuration_window();
    startup_gate_set_waiting_for_config(false);

    if (!g_user_config.is_configured)
    {
        log_unconfigured_boot_state(had_valid_config_on_boot);
        return ESP_OK;
    }

    ESP_ERROR_CHECK(disable_config_service());
    *ready_to_start_local_services = true;
    return ESP_OK;
}
