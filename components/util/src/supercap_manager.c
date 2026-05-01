#include "supercap_manager.h"

#include "charge_control.h"
#include "power_monitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static esp_err_t supercap_prepare_for_4g_internal(bool *ready)
{
    power_monitor_reading_t reading = {0};
    esp_err_t err = power_monitor_run(&reading);
    if (err != ESP_OK)
    {
        return err;
    }

    if (reading.module_supply_ready)
    {
        *ready = true;
        return ESP_OK;
    }

    if (!reading.charge_allowed)
    {
        return ESP_ERR_INVALID_STATE;
    }

    err = charge_control_init();
    if (err != ESP_OK)
    {
        return err;
    }

    err = charge_control_ramp_to_half_scale();
    if (err != ESP_OK)
    {
        charge_control_stop();
        return err;
    }

    const TickType_t recheck_delay_ticks = pdMS_TO_TICKS(BOARD_CHARGE_RECHECK_INTERVAL_MS);
    uint32_t elapsed_ms = 0U;

    while (elapsed_ms <= BOARD_CHARGE_MAX_HOLD_MS)
    {
        err = power_monitor_run(&reading);
        if (err != ESP_OK)
        {
            charge_control_stop();
            return err;
        }

        if (reading.module_supply_ready)
        {
            charge_control_stop();
            *ready = true;
            return ESP_OK;
        }

        if (!reading.charge_allowed)
        {
            charge_control_stop();
            return ESP_ERR_INVALID_STATE;
        }

        if (elapsed_ms == BOARD_CHARGE_MAX_HOLD_MS)
        {
            break;
        }

        vTaskDelay(recheck_delay_ticks);
        elapsed_ms += BOARD_CHARGE_RECHECK_INTERVAL_MS;
    }

    charge_control_stop();
    return ESP_ERR_TIMEOUT;
}

esp_err_t supercap_prepare_for_4g(bool need_4g, bool *ready)
{
    if (ready == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *ready = false;

    if (!need_4g)
    {
        *ready = true;
        return ESP_OK;
    }

    return supercap_prepare_for_4g_internal(ready);
}
