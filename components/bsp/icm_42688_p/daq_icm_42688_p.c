#include "daq_icm_42688_p.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "logger.h"

#define IMU_ODR_SIZE 16384
static icm_cfg_t s_last_cfg = {.odr = 0xFF, .fs = 0xFF};
static int16_t s_last_chunck_count = 0;

static StreamBufferHandle_t s_daq_stream = NULL;
esp_err_t daq_icm_42688_p_init(void)
{
    // 将底层的 SPI 初始化、DMA 资源申请全部包装在这里
    // 如果以后换了传感器，只需要改这里，外面的 app_main 完全不用动
    return drv_icm42688_init();
}
// 永远不变的底层私有抽水机
static void daq_internal_dma_callback(const imu_raw_data_t *data, size_t count)
{
    if (s_daq_stream)
    {
        xStreamBufferSend(s_daq_stream, data, count * sizeof(imu_raw_data_t), 0);
    }
}

// 终极抽象引擎
esp_err_t daq_icm_42688_p_capture(icm_cfg_t *cfg,
                          uint32_t duration_ms,
                          daq_data_handler_t handler,
                          void *user_ctx,
                          int16_t chunck_count)
{
    if (!cfg || !handler)
        return ESP_ERR_INVALID_ARG;

    if (cfg->odr != s_last_cfg.odr || cfg->fs != s_last_cfg.fs)
    {
        LOG_DEBUG("Hardware config changed! Re-configuring sensor...");
        drv_icm42688_config(cfg);
        s_last_cfg.odr = cfg->odr;
        s_last_cfg.fs = cfg->fs;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (s_last_chunck_count == 0) {
        s_daq_stream = xStreamBufferCreate(IMU_ODR_SIZE, sizeof(imu_raw_data_t) * chunck_count);
        if (s_daq_stream == NULL) {
            LOG_WARN("Fatal Error: Failed to create DAQ stream!");
            return ESP_ERR_NO_MEM;
        }
    } else if (s_last_chunck_count != chunck_count)
    {
        xStreamBufferSetTriggerLevel(s_daq_stream, sizeof(imu_raw_data_t) * chunck_count);
    }
    s_last_chunck_count = chunck_count; // 记住当前状态

    drv_icm42688_start_stream(daq_internal_dma_callback);

    int64_t end_time_us = esp_timer_get_time() + (int64_t)duration_ms * 1000;
    imu_raw_data_t rx_buf[chunck_count];

    while (esp_timer_get_time() < end_time_us)
    {
        size_t bytes = xStreamBufferReceive(s_daq_stream, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(100));
        if (bytes > 0)
        {
            size_t count = bytes / sizeof(imu_raw_data_t);
            handler(rx_buf, count, user_ctx);
        }
    }
    drv_icm42688_stop_stream();
    return ESP_OK;
}