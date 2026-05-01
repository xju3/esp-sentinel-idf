#include "daq_iis3dwb.h"
#include "bsp_power.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"

static const char *TAG = "DAQ_IIS3DWB";
static daq_data_handler_t s_capture_handler = NULL;
static void *s_capture_user_ctx = NULL;
static int64_t s_capture_skip_until_us = 0;

static esp_err_t daq_iis3dwb_prepare_sensor_session(void)
{
    esp_err_t err = bsp_power_sensor_enable();
    if (err != ESP_OK)
    {
        return err;
    }

    err = drv_iis3dwb_init();
    if (err != ESP_OK)
    {
        (void)bsp_power_sensor_disable();
        return err;
    }

    return ESP_OK;
}

static void daq_iis3dwb_finish_sensor_session(void)
{
    esp_err_t err = drv_iis3dwb_enter_standby();
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to place IIS3DWB into standby before power-off");
    }

    err = bsp_power_sensor_disable();
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to disable IIS3DWB power rail");
    }
}

esp_err_t daq_iis3dwb_init(void)
{
    return daq_iis3dwb_prepare_sensor_session();
}

static void daq_internal_dma_callback(const imu_raw_data_t *data, size_t count, void *ctx)
{
    (void)ctx;
    if (!s_capture_handler || !data || count == 0U) {
        return;
    }

    if (esp_timer_get_time() >= s_capture_skip_until_us) {
        s_capture_handler(data, count, s_capture_user_ctx);
    }
}

esp_err_t daq_iis3dwb_capture(
    iis3dwb_cfg_t *cfg,
    uint32_t duration_ms,
    daq_data_handler_t handler,
    void *user_ctx,
    int16_t chunck_size,
    uint32_t skip_ms)
{
    if (!cfg || !handler || chunck_size <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = daq_iis3dwb_prepare_sensor_session();
    if (err != ESP_OK) {
        return err;
    }

    err = drv_iis3dwb_config(cfg);
    if (err != ESP_OK) {
        daq_iis3dwb_finish_sensor_session();
        return err;
    }

    s_capture_handler = handler;
    s_capture_user_ctx = user_ctx;

    err = drv_iis3dwb_start_stream_ex(daq_internal_dma_callback, NULL);
    if (err != ESP_OK) {
        s_capture_handler = NULL;
        s_capture_user_ctx = NULL;
        daq_iis3dwb_finish_sensor_session();
        return err;
    }

    int64_t start_time_us = esp_timer_get_time();
    int64_t end_time_us = start_time_us + (int64_t)duration_ms * 1000;
    s_capture_skip_until_us = start_time_us + (int64_t)skip_ms * 1000;

    while (esp_timer_get_time() < end_time_us) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    (void)drv_iis3dwb_stop_stream();
    s_capture_handler = NULL;
    s_capture_user_ctx = NULL;
    s_capture_skip_until_us = 0;
    daq_iis3dwb_finish_sensor_session();
    return ESP_OK;
}
