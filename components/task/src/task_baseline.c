#include "task_baseline.h"
#include "drv_icm_42688_p.h"
#include "daq_icm_42688_p.h"
#include "fs_utils.h"
#include "logger.h"
#include "algo_pdm.h"
#include "config_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include "esp_timer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>


// Define global baseline variable
vib_baseline_t g_baseline = {0};

// 1. 专属的业务数据处理算子
static void baseline_chunk_handler(const imu_raw_data_t *data, size_t count, void *ctx)
{
    vib_welford_3d_t *welford_st = (vib_welford_3d_t *)ctx;
    for (size_t i = 0; i < count; i++)
    {
        float x_g = (int16_t)__builtin_bswap16((uint16_t)data[i].x) * LSB_TO_G;
        float y_g = (int16_t)__builtin_bswap16((uint16_t)data[i].y) * LSB_TO_G;
        float z_g = (int16_t)__builtin_bswap16((uint16_t)data[i].z) * LSB_TO_G;
        vib_welford_3d_update(welford_st, x_g, y_g, z_g);
    }
}

static void print_baseline()
{
    LOG_DEBUGF("baseline: %f, %f, %f, %f, %f, %f",
               g_baseline.x.val,
               g_baseline.y.val,
               g_baseline.z.val,
               g_baseline.x.offset,
               g_baseline.y.offset,
               g_baseline.z.offset);
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

esp_err_t set_device_baseline(uint32_t duration_ms, const char *device_id)
{
    bool existing = load_existing(device_id, &g_baseline);
    if (existing)
    {
        print_baseline();
        return ESP_OK;
    }

    // 3. 配置传感器
    icm_cfg_t cfg = {.odr = ICM_ODR_1KHZ, .fs = ICM_FS_16G};

    vib_welford_3d_t welford_st;
    vib_welford_3d_init(&welford_st);

    // 召唤引擎！把配置、时间和自己的处理函数传进去
    esp_err_t err = daq_icm_42688_p_capture(&cfg, duration_ms, baseline_chunk_handler, &welford_st, 128);
    if (err == ESP_OK)
    {
        // ==============================================================
        // 3. 概念映射：统计学(Stats) -> 业务基线(Baseline)
        // 利用您已有的提取函数，将方差转为 RMS 存入 offset
        // ==============================================================
        g_baseline.x.val = vib_welford_1d_mean(&welford_st.x);
        g_baseline.y.val = vib_welford_1d_mean(&welford_st.y);
        g_baseline.z.val = vib_welford_1d_mean(&welford_st.z);

        // 您的库中提供的是方差(Var)，业务上这里通常记作环境噪声的 RMS (即标准差)
        g_baseline.x.offset = sqrtf(vib_welford_1d_var_sample(&welford_st.x));
        g_baseline.y.offset = sqrtf(vib_welford_1d_var_sample(&welford_st.y));
        g_baseline.z.offset = sqrtf(vib_welford_1d_var_sample(&welford_st.z));
    }

    // 7. (极其重要) 基线算完后，拆除水管释放宝贵的内部 RAM！
    if (err != ESP_OK)
    {
        LOG_ERROR("Baseline capture failed!");
        return ESP_ERR_INVALID_STATE;
    }
    append_entry(device_id, &g_baseline);
    print_baseline();
    return ESP_OK;
}