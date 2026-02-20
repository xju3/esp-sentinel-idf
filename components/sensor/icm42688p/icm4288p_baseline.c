#include "icm4288p_baseline.h"
#include "fs_utils.h"
#include "logger.h"
#include "cJSON.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

// Baseline sampling: 4 kHz for 1 second -> ~4000 samples
#define SAMPLE_RATE_HZ 4000
#define SAMPLE_WINDOW_MS 1000

icm_freq_profile_t g_icm_baseline = {0};

typedef struct {
    uint32_t total;
    uint32_t count;
    icm4288p_sample_t prev;
    double sum_x, sum_y, sum_z;
    double sum_dx, sum_dy, sum_dz;
    SemaphoreHandle_t done;
    esp_err_t err;
} baseline_ctx_t;

// Timer callback: periodic sampling without busy-wait.
static void icm_baseline_timer_cb(void *arg)
{
    baseline_ctx_t *c = (baseline_ctx_t *)arg;
    if (c->count >= c->total) {
        return;
    }
    icm4288p_sample_t s;
    esp_err_t r = icm4288p_read_accel(&s);
    if (r != ESP_OK) {
        c->err = r;
        c->count = c->total;
    } else {
        c->sum_x += s.x_g;
        c->sum_y += s.y_g;
        c->sum_z += s.z_g;
        c->sum_dx += (s.x_g - c->prev.x_g);
        c->sum_dy += (s.y_g - c->prev.y_g);
        c->sum_dz += (s.z_g - c->prev.z_g);
        c->prev = s;
        c->count++;
    }
    if (c->count >= c->total) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(c->done, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

static bool load_existing(const char *device_id, icm_freq_profile_t *out)
{
    if (!fsu_file_exists(ICM4288P_BASELINE_PATH)) {
        return false;
    }
    size_t len = 0;
    char *json = fsu_read_file_alloc(ICM4288P_BASELINE_PATH, &len);
    if (!json) return false;
    cJSON *root = cJSON_Parse(json);
    free(json);
    if (!root || !cJSON_IsArray(root)) {
        if (root) cJSON_Delete(root);
        return false;
    }
    size_t count = cJSON_GetArraySize(root);
    for (size_t i = 0; i < count; ++i) {
        cJSON *item = cJSON_GetArrayItem(root, i);
        cJSON *did = cJSON_GetObjectItemCaseSensitive(item, "device_id");
        if (cJSON_IsString(did) && strcmp(did->valuestring, device_id) == 0) {
            cJSON *freqs = cJSON_GetObjectItemCaseSensitive(item, "frequencies");
            if (!cJSON_IsObject(freqs)) break;
            cJSON *x = cJSON_GetObjectItemCaseSensitive(freqs, "x");
            cJSON *y = cJSON_GetObjectItemCaseSensitive(freqs, "y");
            cJSON *z = cJSON_GetObjectItemCaseSensitive(freqs, "z");
            if (cJSON_IsObject(x)) {
                out->x.val = (float)cJSON_GetObjectItemCaseSensitive(x, "val")->valuedouble;
                out->x.offset = (float)cJSON_GetObjectItemCaseSensitive(x, "offset")->valuedouble;
            }
            if (cJSON_IsObject(y)) {
                out->y.val = (float)cJSON_GetObjectItemCaseSensitive(y, "val")->valuedouble;
                out->y.offset = (float)cJSON_GetObjectItemCaseSensitive(y, "offset")->valuedouble;
            }
            if (cJSON_IsObject(z)) {
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

static void append_entry(const char *device_id, const icm_freq_profile_t *p)
{
    cJSON *root = NULL;
    if (fsu_file_exists(ICM4288P_BASELINE_PATH)) {
        size_t len = 0;
        char *json = fsu_read_file_alloc(ICM4288P_BASELINE_PATH, &len);
        if (json) {
            root = cJSON_Parse(json);
            free(json);
        }
    }
    if (!root || !cJSON_IsArray(root)) {
        if (root) cJSON_Delete(root);
        root = cJSON_CreateArray();
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
    if (out) {
        fsu_write_file(ICM4288P_BASELINE_PATH, out, strlen(out));
        free(out);
    }
    cJSON_Delete(root);
}

esp_err_t icm4288p_ensure_baseline(const char *device_id, icm_freq_profile_t *out_profile)
{
    if (!device_id || !out_profile) {
        return ESP_ERR_INVALID_ARG;
    }

    if (load_existing(device_id, out_profile)) {
        g_icm_baseline = *out_profile;
        return ESP_OK;
    }

    LOG_INFOF("Baseline missing for device %s, sampling...", device_id);

    esp_err_t err = icm4288p_init();
    if (err != ESP_OK) {
        return err;
    }

    const uint32_t period_us = 1000000UL / SAMPLE_RATE_HZ;
    const uint32_t total_samples = (SAMPLE_WINDOW_MS * SAMPLE_RATE_HZ) / 1000;

    baseline_ctx_t ctx = {
        .total = total_samples,
        .count = 0,
        .prev = {0},
        .sum_x = 0,
        .sum_y = 0,
        .sum_z = 0,
        .sum_dx = 0,
        .sum_dy = 0,
        .sum_dz = 0,
        .done = xSemaphoreCreateBinary(),
        .err = ESP_OK,
    };

    if (!ctx.done) {
        return ESP_ERR_NO_MEM;
    }

    esp_timer_handle_t timer = NULL;

    esp_timer_create_args_t targs = {
        .callback = icm_baseline_timer_cb,
        .arg = &ctx,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "icm_baseline",
    };

    err = esp_timer_create(&targs, &timer);
    if (err != ESP_OK) {
        vSemaphoreDelete(ctx.done);
        return err;
    }
    err = esp_timer_start_periodic(timer, period_us);
    if (err != ESP_OK) {
        esp_timer_delete(timer);
        vSemaphoreDelete(ctx.done);
        return err;
    }

    // Wait until sampling done or timeout (add margin: window + 200 ms)
    TickType_t timeout_ticks = pdMS_TO_TICKS(SAMPLE_WINDOW_MS + 200);
    if (xSemaphoreTake(ctx.done, timeout_ticks) != pdTRUE) {
        err = ESP_ERR_TIMEOUT;
    }

    esp_timer_stop(timer);
    esp_timer_delete(timer);
    vSemaphoreDelete(ctx.done);

    if (err != ESP_OK || ctx.err != ESP_OK) {
        return (err != ESP_OK) ? err : ctx.err;
    }

    float avg_x = (float)(ctx.sum_x / (double)total_samples);
    float avg_y = (float)(ctx.sum_y / (double)total_samples);
    float avg_z = (float)(ctx.sum_z / (double)total_samples);
    float off_x = (float)(ctx.sum_dx / (double)total_samples);
    float off_y = (float)(ctx.sum_dy / (double)total_samples);
    float off_z = (float)(ctx.sum_dz / (double)total_samples);

    out_profile->x.val = avg_x;
    out_profile->y.val = avg_y;
    out_profile->z.val = avg_z;
    out_profile->x.offset = off_x;
    out_profile->y.offset = off_y;
    out_profile->z.offset = off_z;

    g_icm_baseline = *out_profile;

    append_entry(device_id, out_profile);
    LOG_INFO("Baseline sampling finished and stored");
    return ESP_OK;
}
