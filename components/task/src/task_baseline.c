#include "task_baseline.h"
#include "algo_stash.h"
#include "algo_welford.h"
#include "config_manager.h"
#include "drv_iis3dwb.h"
#include "daq_iis3dwb.h"
#include "fs_utils.h"
#include "logger.h"
#include "task_stash.h"
// system
#include "cJSON.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Define global baseline variable
vib_baseline_t g_baseline = {0};
static int64_t baseline_sample_count = 0;

typedef struct
{
    vib_welford_3d_t *welford;
} baseline_cb_ctx_t;

static void print_baseline()
{
    LOG_DEBUGF("x=%f, y=%f, z=%f, offset_x=%f, offset_y=%f, offset_y=%f",
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

static esp_err_t handle_baseline_data(const char *device_id, vib_welford_3d_t *welford_st)
{
    // ==============================================================
    // 3. 概念映射：统计学(Stats) -> 业务基线(Baseline)
    // 利用您已有的提取函数，将方差转为 RMS 存入 offset
    // ==============================================================
    g_baseline.x.val = vib_welford_1d_mean(&welford_st->x);
    g_baseline.y.val = vib_welford_1d_mean(&welford_st->y);
    g_baseline.z.val = vib_welford_1d_mean(&welford_st->z);

    // 您的库中提供的是方差(Var)，业务上这里通常记作环境噪声的 RMS (即标准差)
    g_baseline.x.offset = sqrtf(vib_welford_1d_var_sample(&welford_st->x));
    g_baseline.y.offset = sqrtf(vib_welford_1d_var_sample(&welford_st->y));
    g_baseline.z.offset = sqrtf(vib_welford_1d_var_sample(&welford_st->z));

    LOG_DEBUGF("Baseline sample count: %lld", baseline_sample_count);
    baseline_sample_count = 0; // Reset for next time
    print_baseline();
    // vib_welford_3d_destroy(&welford_st);
    append_entry(device_id, &g_baseline);
    return ESP_OK;
}

// 专属的业务数据处理算子
static void baseline_chunk_handler(const imu_raw_data_t *data, size_t count, void *ctx)
{
    baseline_cb_ctx_t *cb_ctx = (baseline_cb_ctx_t *)ctx;
    if (!cb_ctx || !cb_ctx->welford)
        return;

    for (size_t i = 0; i < count; i++)
    {
        float x_g = (float)data[i].x * LSB_TO_G_2;
        float y_g = (float)data[i].y * LSB_TO_G_2;
        float z_g = (float)data[i].z * LSB_TO_G_2;
        vib_welford_3d_update(cb_ctx->welford, x_g, y_g, z_g);
    }
    baseline_sample_count += 1;
}

static esp_err_t handle_baseline_by_iis3dwb(const char *device_id)
{
    esp_err_t err = ESP_OK;
    err = daq_iis3dwb_init();
    if (err != ESP_OK)
    {
        LOG_WARN("iis3dwb init failed");
        return err;
    }

    DSP_Config_t baseline_dsp_config = IMU_Calculate_DSP_Config(
        &iis3dwb_driver,
        (float)g_user_config.rpm,
        10.0f,   // C
        10.0f,   // H
        1000.0f, // F_env
        1.0f     // delta_f_max
    );

    if (baseline_dsp_config.actual_time <= 0.0f || baseline_dsp_config.actual_odr <= 0.0f)
    {
        LOG_WARN("iis3dwb dsp config failed");
        return ESP_FAIL;
    }

    iis3dwb_cfg_t cfg = {.fs = IIS3DWB_FS_2G};
    vib_welford_3d_t welford_st;
    vib_welford_3d_init(&welford_st);
    baseline_cb_ctx_t cb_ctx = {.welford = &welford_st};

    baseline_sample_count = 0;
    err = daq_iis3dwb_capture(&cfg,
                              baseline_dsp_config.actual_time * 1000.0f,
                              baseline_chunk_handler,
                              &cb_ctx, 64, 0);
    if (err != ESP_OK)
        return err;

    err = handle_baseline_data(device_id, &welford_st);
    if (err != ESP_OK)
    {
        LOG_WARN("failed to set baseline");
    }

    return err;
}

esp_err_t set_device_baseline(const char *device_id)
{
    return handle_baseline_by_iis3dwb(device_id);
}
