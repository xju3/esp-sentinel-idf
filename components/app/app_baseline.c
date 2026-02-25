#include "app_baseline.h"
#include "drv_icm_42688_p.h"
#include "fs_utils.h"
#include "config_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "logger.h"
#include "cJSON.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "APP_BASELINE";
#define LSB_TO_G (16.0f / 32768.0f)


static void app_basline_imu_data_callback(const icm_raw_data_t *data, size_t count)
{
    static uint32_t chunk_counter = 0;
    chunk_counter++;
    if (count > 0 && (chunk_counter % 10 == 0)) {
        // ICM 传感器硬件输出是大端(Big-Endian)，ESP32 内存是小端(Little-Endian)
        // 必须进行一次字节翻转才能得到真实的整型数值
        int16_t real_x = (int16_t)__builtin_bswap16((uint16_t)data[0].x);
        int16_t real_y = (int16_t)__builtin_bswap16((uint16_t)data[0].y);
        int16_t real_z = (int16_t)__builtin_bswap16((uint16_t)data[0].z);
        // 顺便把 Header 打出来，如果配置正确，Header 的 Bit6 应该是 1 (代表 Accel)
        ESP_LOGI("IMU_TEST", "Chunk #%lu | Header: 0x%02X | X=%d, Y=%d, Z=%d", 
                 chunk_counter, data[0].header, real_x, real_y, real_z);
    }
}

static bool load_existing(const char *device_id, vib_baseline_t *out)
{
    if (!fsu_file_exists(FILE_PATH_DEVICE_PROFILE))
    {
        return false;
    }
    size_t len = 0;
    char *json = fsu_read_file_alloc(FILE_PATH_DEVICE_PROFILE, &len);
    if (!json)
        return false;
    cJSON *root = cJSON_Parse(json);
    free(json);
    if (!root || !cJSON_IsArray(root))
    {
        if (root)
            cJSON_Delete(root);
        return false;
    }
    size_t count = cJSON_GetArraySize(root);
    for (size_t i = 0; i < count; ++i)
    {
        cJSON *item = cJSON_GetArrayItem(root, i);
        cJSON *did = cJSON_GetObjectItemCaseSensitive(item, "device_id");
        if (cJSON_IsString(did) && strcmp(did->valuestring, device_id) == 0)
        {
            cJSON *freqs = cJSON_GetObjectItemCaseSensitive(item, "frequencies");
            if (!cJSON_IsObject(freqs))
                break;
            cJSON *x = cJSON_GetObjectItemCaseSensitive(freqs, "x");
            cJSON *y = cJSON_GetObjectItemCaseSensitive(freqs, "y");
            cJSON *z = cJSON_GetObjectItemCaseSensitive(freqs, "z");
            if (cJSON_IsObject(x))
            {
                out->x.val = (float)cJSON_GetObjectItemCaseSensitive(x, "val")->valuedouble;
                out->x.offset = (float)cJSON_GetObjectItemCaseSensitive(x, "offset")->valuedouble;
            }
            if (cJSON_IsObject(y))
            {
                out->y.val = (float)cJSON_GetObjectItemCaseSensitive(y, "val")->valuedouble;
                out->y.offset = (float)cJSON_GetObjectItemCaseSensitive(y, "offset")->valuedouble;
            }
            if (cJSON_IsObject(z))
            {
                out->z.val = (float)cJSON_GetObjectItemCaseSensitive(z, "val")->valuedouble;
                out->z.offset = (float)cJSON_GetObjectItemCaseSensitive(z, "offset")->valuedouble;
            }
            cJSON_Delete(root);
            return true;
        }
    }
    cJSON_Delete(root);
    return false;
}

static void append_entry(const char *device_id, const vib_baseline_t *p)
{
    cJSON *root = NULL;
    if (fsu_file_exists(FILE_PATH_DEVICE_PROFILE))
    {
        size_t len = 0;
        char *json = fsu_read_file_alloc(FILE_PATH_DEVICE_PROFILE, &len);
        if (json)
        {
            root = cJSON_Parse(json);
            free(json);
        }
    }

    if (!root || !cJSON_IsArray(root))
    {
        if (root)
            cJSON_Delete(root);
        root = cJSON_CreateArray();
    }

    // Remove existing entry for this device_id if it exists
    if (cJSON_IsArray(root))
    {
        size_t count = cJSON_GetArraySize(root);
        for (size_t i = 0; i < count; ++i)
        {
            cJSON *item = cJSON_GetArrayItem(root, i);
            cJSON *did = cJSON_GetObjectItemCaseSensitive(item, "device_id");
            if (cJSON_IsString(did) && strcmp(did->valuestring, device_id) == 0)
            {
                cJSON_DeleteItemFromArray(root, i);
                break; // Assuming unique device_ids
            }
        }
    }

    cJSON *entry = cJSON_CreateObject();
    cJSON_AddStringToObject(entry, "device_id", device_id);
    cJSON *freqs = cJSON_CreateObject();
    cJSON *x = cJSON_CreateObject();
    cJSON *y = cJSON_CreateObject();
    cJSON *z = cJSON_CreateObject();
    cJSON_AddNumberToObject(x, "val", p->x.val);
    cJSON_AddNumberToObject(x, "offset", p->x.offset);
    cJSON_AddNumberToObject(y, "val", p->y.val);
    cJSON_AddNumberToObject(y, "offset", p->y.offset);
    cJSON_AddNumberToObject(z, "val", p->z.val);
    cJSON_AddNumberToObject(z, "offset", p->z.offset);
    cJSON_AddItemToObject(freqs, "x", x);
    cJSON_AddItemToObject(freqs, "y", y);
    cJSON_AddItemToObject(freqs, "z", z);
    cJSON_AddItemToObject(entry, "frequencies", freqs);
    cJSON_AddItemToArray(root, entry);

    char *out = cJSON_PrintUnformatted(root);
    if (out)
    {
        fsu_write_file(FILE_PATH_DEVICE_PROFILE, out, strlen(out));
        free(out);
    }
    cJSON_Delete(root);
}

esp_err_t set_device_baseline(uint32_t duration_ms,
                               vib_baseline_t *out_bl, const char *device_id)
{

    bool existing = load_existing(device_id, out_bl);
    if (existing) {
        LOG_DEBUGF("baseline: %f, %f, %f", out_bl->x.val, out_bl->y.val, out_bl->z.val);
        return ESP_OK;
    }

    g_imu_stream = xStreamBufferCreate(IMU_STREAM_SIZE, 1);
    if (g_imu_stream == NULL) {
        LOG_ERROR("Failed to create stream buffer!");
        vTaskDelete(NULL);
        return ESP_ERR_INVALID_STATE;
    }

    xStreamBufferReset(g_imu_stream);
    icm_cfg_t cfg = {.odr = ICM_ODR_1KHZ, .fs = ICM_FS_16G};
    drv_icm42688_config(&cfg);
    esp_err_t err = drv_icm42688_start_stream(app_basline_imu_data_callback);
    if (err != ESP_OK)
    {
        LOG_ERROR("Stream start failed!");
    }
    drv_icm42688_stop_stream();
    append_entry(device_id, out_bl);
    return ESP_OK;
    
}

esp_err_t app_baseline_capture(StreamBufferHandle_t stream,
                               uint32_t duration_ms,
                               vib_baseline_t *out_bl, const char *device_id)
{
    if (!stream || !out_bl)
        return ESP_ERR_INVALID_ARG;

    // 1. 使用您独立算法库的上下文结构体
    vib_welford_3d_t welford_st;
    // 假设您的库中有初始化函数（将 mean 和 M2 清零）
    vib_welford_3d_init(&welford_st);

    int64_t start_time_us = esp_timer_get_time();
    int64_t end_time_us = start_time_us + (int64_t)duration_ms * 1000;

    icm_raw_data_t rx_buf[64];
    uint32_t total_samples = 0;
    ESP_LOGI(TAG, "Baseline capture started. Target duration: %lu ms", duration_ms);
    // 2. 管道消费与喂数据
    while (esp_timer_get_time() < end_time_us)
    {
        size_t bytes_read = xStreamBufferReceive(stream, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(100));

        if (bytes_read > 0)
        {
            size_t samples_in_chunk = bytes_read / sizeof(icm_raw_data_t);

            for (size_t i = 0; i < samples_in_chunk; i++)
            {
                // 大小端转换并还原物理真实值 (g)
                float x_g = (int16_t)__builtin_bswap16((uint16_t)rx_buf[i].x) * LSB_TO_G;
                float y_g = (int16_t)__builtin_bswap16((uint16_t)rx_buf[i].y) * LSB_TO_G;
                float z_g = (int16_t)__builtin_bswap16((uint16_t)rx_buf[i].z) * LSB_TO_G;

                // 直接调用您的算法库进行内部状态迭代
                vib_welford_3d_update(&welford_st, x_g, y_g, z_g);
                total_samples++;
            }
        }
    }

    if (total_samples == 0)
    {
        ESP_LOGE(TAG, "Error: No data received from StreamBuffer!");
        return ESP_FAIL;
    }

    // ==============================================================
    // 3. 概念映射：统计学(Stats) -> 业务基线(Baseline)
    // 利用您已有的提取函数，将方差转为 RMS 存入 offset
    // ==============================================================
    out_bl->x.val = vib_welford_1d_mean(&welford_st.x);
    out_bl->y.val = vib_welford_1d_mean(&welford_st.y);
    out_bl->z.val = vib_welford_1d_mean(&welford_st.z);

    // 您的库中提供的是方差(Var)，业务上这里通常记作环境噪声的 RMS (即标准差)
    out_bl->x.offset = sqrtf(vib_welford_1d_var_sample(&welford_st.x));
    out_bl->y.offset = sqrtf(vib_welford_1d_var_sample(&welford_st.y));
    out_bl->z.offset = sqrtf(vib_welford_1d_var_sample(&welford_st.z));
    LOG_DEBUGF("Baseline Capture Done! Processed %lu samples.", total_samples);
    return ESP_OK;
}