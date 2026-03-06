#include "daq_icm_42688_p.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <stdlib.h> // for malloc/free
#include "esp_heap_caps.h"

#define IMU_ODR_SIZE 16384
static int16_t s_last_chunck_count = 0;
static StreamBufferHandle_t s_daq_stream = NULL;

esp_err_t daq_icm_42688_p_init(void) {
    return drv_icm42688_init();
}

// 底层私有抽水机 (由 DMA Worker 触发)
static void daq_internal_dma_callback(const imu_raw_data_t *data, size_t count) {
    if (s_daq_stream) {
        xStreamBufferSend(s_daq_stream, data, count * sizeof(imu_raw_data_t), 0);
    }
}

// 终极采集引擎：按块将平滑数据回调给上层算法
esp_err_t daq_icm_42688_p_capture(
    icm_cfg_t *cfg,             
    uint32_t duration_ms,       
    daq_data_handler_t handler, 
    void *user_ctx,             
    int16_t chunck_size,        
    uint32_t skip_ms)           
{
    if (!cfg || !handler) return ESP_ERR_INVALID_ARG;

    // 下发 FS / WoM 等配置 (不干扰 ODR)
    drv_icm42688_config(cfg);
    vTaskDelay(pdMS_TO_TICKS(20)); // 给硬件配置生效留出稳定时间

    // 动态调整 StreamBuffer 的触发水位
    if (s_last_chunck_count == 0) {
        s_daq_stream = xStreamBufferCreate(IMU_ODR_SIZE, sizeof(imu_raw_data_t) * chunck_size);
        if (s_daq_stream == NULL) {
            return ESP_ERR_NO_MEM;
        }
    } else if (s_last_chunck_count != chunck_size) {
        xStreamBufferSetTriggerLevel(s_daq_stream, sizeof(imu_raw_data_t) * chunck_size);
    }
    s_last_chunck_count = chunck_size; 

    // 启动底层硬件 DMA 自动流水线
    drv_icm42688_start_stream(daq_internal_dma_callback);

    int64_t start_time_us = esp_timer_get_time();
    int64_t end_time_us = start_time_us + (int64_t)duration_ms * 1000;
    int64_t skip_until_us = start_time_us + (int64_t)skip_ms * 1000;
    
    // 从系统堆分配接收缓冲
    imu_raw_data_t *rx_buf = (imu_raw_data_t *)heap_caps_malloc(sizeof(imu_raw_data_t) * chunck_size, MALLOC_CAP_SPIRAM);
    if (!rx_buf) {
        drv_icm42688_stop_stream();
        return ESP_ERR_NO_MEM;
    }

    // 在预期时长内循环汲取数据并抛给算法处理
    while (esp_timer_get_time() < end_time_us) {
        size_t bytes = xStreamBufferReceive(s_daq_stream, rx_buf, sizeof(imu_raw_data_t) * chunck_size, pdMS_TO_TICKS(100));
        if (bytes > 0) {
            size_t count = bytes / sizeof(imu_raw_data_t);
            int64_t now = esp_timer_get_time();
            // 过滤掉硬件启动初期的不稳定暂态
            if (now >= skip_until_us) {
                handler(rx_buf, count, user_ctx);
            }
        }
    }
    
    heap_caps_free(rx_buf);
    drv_icm42688_stop_stream();
    return ESP_OK;
}