#include "init.h"
#include "bsp_4g.h"
#include "bsp_wifi.h"
#include "config_manager.h"
#include "drv_lis2dh12.h"
#include "drv_iis3dwb.h"
#include "drv_t1820b.h"
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

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>

static void init_sensors()
{
    drv_lis2dh12_init();
    drv_iis3dwb_init();
    drv_t1820b_init();
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

esp_err_t start_local_services()
{
    esp_err_t err = ESP_OK;

    init_sensors();

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

