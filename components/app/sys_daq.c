#include "sys_daq.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "SYS_DAQ";
static StreamBufferHandle_t s_daq_stream = NULL;

// 永远不变的底层私有抽水机
static void daq_internal_dma_callback(const icm_raw_data_t *data, size_t count)
{
    if (s_daq_stream) {
        xStreamBufferSend(s_daq_stream, data, count * sizeof(icm_raw_data_t), 0);
    }
}

// 终极抽象引擎
esp_err_t sys_daq_capture(icm_cfg_t *cfg, uint32_t duration_ms, daq_data_handler_t handler, void *user_ctx)
{
    if (!cfg || !handler) return ESP_ERR_INVALID_ARG;

    // 1. 基建：建水管 (统一 16KB)
    s_daq_stream = xStreamBufferCreate(16384, 1024);
    if (!s_daq_stream) return ESP_FAIL;

    // 2. 发车：配置并启动底层硬件
    drv_icm42688_config(cfg);
    drv_icm42688_start_stream(daq_internal_dma_callback);

    // 3. 核心循环：等水、舀水、分发给业务层
    int64_t end_time_us = esp_timer_get_time() + (int64_t)duration_ms * 1000;
    icm_raw_data_t rx_buf[128]; 

    while (esp_timer_get_time() < end_time_us) 
    {
        size_t bytes = xStreamBufferReceive(s_daq_stream, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(100));
        if (bytes > 0) {
            size_t count = bytes / sizeof(icm_raw_data_t);
            // 【依赖注入】：将数据丢给业务层传进来的处理函数
            handler(rx_buf, count, user_ctx);
        }
    }

    // 4. 收尾：关硬件，砸水管，完美释放内存
    drv_icm42688_stop_stream();
    vStreamBufferDelete(s_daq_stream);
    s_daq_stream = NULL;

    return ESP_OK;
}