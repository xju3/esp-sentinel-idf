#include "daq_iis3dwb.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/stream_buffer.h"

#define IIS3DWB_STREAM_BYTES 16384

static const char *TAG = "DAQ_IIS3DWB";
static int16_t s_last_chunck_count = 0;
static StreamBufferHandle_t s_daq_stream = NULL;

static void log_heap_snapshot(const char *hint)
{
    ESP_LOGE(TAG,
             "%s | internal free=%u largest=%u, psram free=%u largest=%u",
             hint ? hint : "heap snapshot",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
}

esp_err_t daq_iis3dwb_init(void)
{
    return drv_iis3dwb_init();
}

static void daq_internal_dma_callback(const imu_raw_data_t *data, size_t count, void *ctx)
{
    (void)ctx;
    if (s_daq_stream) {
        xStreamBufferSend(s_daq_stream, data, count * sizeof(imu_raw_data_t), 0);
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

    esp_err_t err = drv_iis3dwb_config(cfg);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    if (s_daq_stream == NULL) {
        s_daq_stream = xStreamBufferCreateWithCaps(IIS3DWB_STREAM_BYTES,
                                                   sizeof(imu_raw_data_t) * chunck_size,
                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_daq_stream == NULL) {
            log_heap_snapshot("xStreamBufferCreateWithCaps failed");
            return ESP_ERR_NO_MEM;
        }
    } else if (s_last_chunck_count != chunck_size) {
        xStreamBufferSetTriggerLevel(s_daq_stream, sizeof(imu_raw_data_t) * chunck_size);
    }
    s_last_chunck_count = chunck_size;

    err = drv_iis3dwb_start_stream_ex(daq_internal_dma_callback, NULL);
    if (err != ESP_OK) {
        log_heap_snapshot("drv_iis3dwb_start_stream_ex failed");
        return err;
    }

    imu_raw_data_t *rx_buf = (imu_raw_data_t *)heap_caps_malloc(sizeof(imu_raw_data_t) * chunck_size,
                                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rx_buf) {
        ESP_LOGW(TAG, "RX buffer PSRAM alloc failed, trying generic 8-bit heap");
        rx_buf = (imu_raw_data_t *)heap_caps_malloc(sizeof(imu_raw_data_t) * chunck_size, MALLOC_CAP_8BIT);
    }
    if (!rx_buf) {
        log_heap_snapshot("RX buffer alloc failed");
        drv_iis3dwb_stop_stream();
        return ESP_ERR_NO_MEM;
    }

    int64_t start_time_us = esp_timer_get_time();
    int64_t end_time_us = start_time_us + (int64_t)duration_ms * 1000;
    int64_t skip_until_us = start_time_us + (int64_t)skip_ms * 1000;

    while (esp_timer_get_time() < end_time_us) {
        size_t bytes = xStreamBufferReceive(s_daq_stream, rx_buf,
                                            sizeof(imu_raw_data_t) * chunck_size,
                                            pdMS_TO_TICKS(100));
        if (bytes > 0) {
            size_t count = bytes / sizeof(imu_raw_data_t);
            int64_t now = esp_timer_get_time();
            if (now >= skip_until_us) {
                handler(rx_buf, count, user_ctx);
            }
        }
    }

    heap_caps_free(rx_buf);
    drv_iis3dwb_stop_stream();
    return ESP_OK;
}
