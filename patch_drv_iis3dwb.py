import os

drv_c = "components/peri/src/drv_iis3dwb.c"
drv_h = "components/peri/include/drv_iis3dwb.h"

with open(drv_c, "a") as f:
    f.write("""
#include "esp_timer.h"

static imu_data_cb_ctx_t s_capture_handler = NULL;
static void *s_capture_user_ctx = NULL;
static int64_t s_capture_skip_until_us = 0;

static void drv_internal_dma_callback(const imu_raw_data_t *data, size_t count, void *ctx)
{
    (void)ctx;
    if (!s_capture_handler || !data || count == 0U) {
        return;
    }

    if (esp_timer_get_time() >= s_capture_skip_until_us) {
        s_capture_handler(data, count, s_capture_user_ctx);
    }
}

static esp_err_t drv_iis3dwb_prepare_sensor_session(void)
{
    gpio_set_level(BOARD_GPIO_SENSOR_EN, 0);
    esp_err_t err = drv_iis3dwb_init();
    if (err != ESP_OK)
    {
        gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);
        return err;
    }
    return ESP_OK;
}

static void drv_iis3dwb_finish_sensor_session(void)
{
    esp_err_t err = drv_iis3dwb_enter_standby();
    if (err != ESP_OK)
    {
        LOG_WARN("Failed to place IIS3DWB into standby before power-off");
    }

    gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);
}

esp_err_t drv_iis3dwb_capture(
    iis3dwb_cfg_t *cfg,
    uint32_t duration_ms,
    imu_data_cb_ctx_t handler,
    void *user_ctx,
    int16_t chunck_size,
    uint32_t skip_ms)
{
    if (!cfg || !handler || chunck_size <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = drv_iis3dwb_prepare_sensor_session();
    if (err != ESP_OK) {
        return err;
    }

    err = drv_iis3dwb_config(cfg);
    if (err != ESP_OK) {
        drv_iis3dwb_finish_sensor_session();
        return err;
    }

    s_capture_handler = handler;
    s_capture_user_ctx = user_ctx;

    err = drv_iis3dwb_start_stream_ex(drv_internal_dma_callback, NULL);
    if (err != ESP_OK) {
        s_capture_handler = NULL;
        s_capture_user_ctx = NULL;
        drv_iis3dwb_finish_sensor_session();
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
    drv_iis3dwb_finish_sensor_session();
    return ESP_OK;
}
""")

with open(drv_h, "r") as f:
    content = f.read()

content = content.replace("extern iis3dwb_cfg_t iis3dwb_accel_fs_cfg_2;", """extern iis3dwb_cfg_t iis3dwb_accel_fs_cfg_2;

esp_err_t drv_iis3dwb_capture(
    iis3dwb_cfg_t *cfg,
    uint32_t duration_ms,
    imu_data_cb_ctx_t handler,
    void *user_ctx,
    int16_t chunck_size,
    uint32_t skip_ms
);
""")

with open(drv_h, "w") as f:
    f.write(content)

