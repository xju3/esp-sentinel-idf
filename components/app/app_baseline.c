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

// Global StreamBuffer handle (created/managed by app layer)
StreamBufferHandle_t g_imu_stream = NULL;

static void app_basline_imu_data_callback(const icm_raw_data_t *data, size_t count)
{
    // 1. 安全拦截：如果水管还没建好，直接丢弃数据，防止空指针崩溃
    if (g_imu_stream == NULL)
    {
        return;
    }

    // 2. 计算本次需要灌入水管的字节数 (通常是 128 * 8 = 1024 字节)
    size_t bytes_to_send = count * sizeof(icm_raw_data_t);

    // 3. 核心修复：使用 Task 级别的 API 发送数据
    // 参数 4 为超时时间(Ticks)：设为 0！(非阻塞)
    // 宁可丢数据，也绝不阻塞极高优先级的 DMA 守护任务
    size_t bytes_sent = xStreamBufferSend(g_imu_stream,
                                          (const void *)data,
                                          bytes_to_send,
                                          0);

    // (可选防御) 如果发现水管满了，打印个警告。
    // 如果这里频发警告，说明 app_baseline_capture 处理太慢了，或者水管建得太细
    // if (bytes_sent != bytes_to_send) {
    //     ESP_LOGW("IMU_CB", "StreamBuffer Full! Dropped %d bytes", bytes_to_send - bytes_sent);
    // }

    // 4. 极简心跳打印 (确认数据正常流转)
    static uint32_t chunk_counter = 0;
    chunk_counter++;
    if (count > 0 && (chunk_counter % 10 == 0))
    {
        // 仅供测试肉眼观察，做一次大端翻转打印 Z 轴
        int16_t real_z = (int16_t)__builtin_bswap16((uint16_t)data[0].z);
        ESP_LOGD("IMU_CB", "Chunk #%lu pushed. Z=%d", chunk_counter, real_z);
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
    LOG_DEBUGF("file written: %s", FILE_PATH_DEVICE_PROFILE);
}

static esp_err_t app_baseline_capture(StreamBufferHandle_t stream,
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

esp_err_t set_device_baseline(uint32_t duration_ms, const char *device_id)
{

    bool existing = load_existing(device_id, &g_baseline);
    if (existing)
    {
        return ESP_OK;
    }

    // 1. 先建好水管 (水管总长设为 IMU_STREAM_SIZE，触发水位线设为 1)
    g_imu_stream = xStreamBufferCreate(IMU_STREAM_SIZE, 1);
    if (g_imu_stream == NULL)
    {
        LOG_ERROR("Failed to create stream buffer!");
        return ESP_ERR_INVALID_STATE;
    }

    // 清除Buffer
    xStreamBufferReset(g_imu_stream);

    // 3. 配置传感器
    icm_cfg_t cfg = {.odr = ICM_ODR_1KHZ, .fs = ICM_FS_16G};
    drv_icm42688_config(&cfg);

    // 4. 打开水龙头，此时 Callback 会开始疯狂调用 xStreamBufferSend
    esp_err_t err = drv_icm42688_start_stream(app_basline_imu_data_callback);

    // 5. 本任务进入阻塞，开始大口喝水并计算 Welford
    err = app_baseline_capture(g_imu_stream, 3000, &g_baseline, device_id);

    // 6. 3 秒结束，关水龙头
    drv_icm42688_stop_stream();

    // 7. (极其重要) 基线算完后，拆除水管释放宝贵的内部 RAM！
    vStreamBufferDelete(g_imu_stream);
    g_imu_stream = NULL;
    if (err != ESP_OK)
    {
        LOG_ERROR("Baseline capture failed!");
        return ESP_ERR_INVALID_STATE;
    }
    append_entry(device_id, &g_baseline);
    LOG_DEBUGF(" device baseline value: %f, %f, %f", g_baseline.x.val, g_baseline.y.val, g_baseline.z.val);
    LOG_DEBUGF("device baseline offset: %f, %f, %f", g_baseline.x.offset, g_baseline.y.offset, g_baseline.z.offset);
    return ESP_OK;
}