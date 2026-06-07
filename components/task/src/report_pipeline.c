#include "report_pipeline.h"

#include "algo_fft.h"
#include "config_manager.h"
#include "daq_iis3dwb.h"
#include "drv_t1820b.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "logger.h"
#include "sdkconfig.h"

#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#ifndef REPORT_SN
#define REPORT_SN "UNKNOWN"
#endif

#ifndef POINTS
#define POINTS 4096
#endif

#ifndef RANGE
#define RANGE 2
#endif

#if (POINTS != 4096) && (POINTS != 8192)
#error "POINTS must be 4096 or 8192"
#endif

#if (RANGE != 2) && (RANGE != 4) && (RANGE != 8) && (RANGE != 16)
#error "RANGE must be one of 2, 4, 8, 16"
#endif

#if CONFIG_DSP_MAX_FFT_SIZE < POINTS
#error "CONFIG_DSP_MAX_FFT_SIZE must be >= POINTS"
#endif

#define REPORT_SCHEMA_VERSION 1
#define REPORT_SAMPLE_TYPE "normal"
#define REPORT_FS_HZ 26667.0f
#define REPORT_POINTS ((uint32_t)POINTS)
#define REPORT_REQUESTED_RANGE_G ((uint16_t)RANGE)
#define REPORT_CAPTURE_SKIP_MS 20U
#define REPORT_CAPTURE_GUARD_MS 120U
#define REPORT_DMA_CHUNK_SIZE 512
#define REPORT_CLIP_THRESHOLD_RATIO 0.98f
#define REPORT_FFT_PEAK_COUNT 5
#define REPORT_FFT_PEAK_MIN_HZ 0.0f
#define REPORT_FFT_PEAK_MAX_HZ 5000.0f

static float *s_vib_buffer;
static float *s_fft_scratch;
static float *s_fft_mag;

typedef struct {
    uint32_t count;
    uint32_t raw_count;
    float lsb_to_g;
} capture_ctx_t;

typedef struct {
    float max_abs_g;
    uint32_t clip_count;
    float clip_ratio;
} axis_clip_quality_t;

typedef struct {
    uint16_t range_g;
    bool accepted;
    bool clipped;
    axis_clip_quality_t axes[3];
} capture_attempt_t;

typedef struct {
    float freq_hz;
    float amp_g;
} report_peak_t;

typedef struct {
    float rms_g;
    float peak_g;
    float crest_factor;
    float kurtosis;
} axis_time_features_t;

typedef struct {
    report_peak_t peaks[REPORT_FFT_PEAK_COUNT];
    float spectral_centroid_hz;
    float spectral_entropy;
    float band_ratio[7];
} axis_freq_features_t;

static const struct {
    const char *key;
    float min_hz;
    float max_hz;
} s_bands[] = {
    {"0_100", 0.0f, 100.0f},
    {"100_200", 100.0f, 200.0f},
    {"200_400", 200.0f, 400.0f},
    {"400_800", 400.0f, 800.0f},
    {"800_1600", 800.0f, 1600.0f},
    {"1600_3200", 1600.0f, 3200.0f},
    {"3200_5000", 3200.0f, 5000.0f},
};

static uint64_t current_epoch_ms_or_zero(void)
{
    time_t now = 0;
    time(&now);
    if (now < 1600000000) {
        return 0;
    }

    struct timeval tv = {0};
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
}

static float lsb_to_g_for_range(uint16_t range_g)
{
    return (float)range_g / 32768.0f;
}

static iis3dwb_cfg_t cfg_for_range(uint16_t range_g)
{
    iis3dwb_cfg_t cfg = {.fs = IIS3DWB_FS_2G};
    switch (range_g) {
        case 2: cfg.fs = IIS3DWB_FS_2G; break;
        case 4: cfg.fs = IIS3DWB_FS_4G; break;
        case 8: cfg.fs = IIS3DWB_FS_8G; break;
        case 16:
        default: cfg.fs = IIS3DWB_FS_16G; break;
    }
    return cfg;
}

static uint16_t next_range(uint16_t range_g)
{
    if (range_g <= 2) return 4;
    if (range_g <= 4) return 8;
    return 16;
}

static uint32_t capture_duration_ms(void)
{
    const float capture_ms = ((float)REPORT_POINTS * 1000.0f) / REPORT_FS_HZ;
    return (uint32_t)ceilf(capture_ms) + REPORT_CAPTURE_SKIP_MS + REPORT_CAPTURE_GUARD_MS;
}

static esp_err_t ensure_buffers(void)
{
    if (!s_vib_buffer) {
        s_vib_buffer = heap_caps_calloc(REPORT_POINTS * 3U, sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!s_fft_scratch) {
        s_fft_scratch = heap_caps_malloc(REPORT_POINTS * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!s_fft_mag) {
        s_fft_mag = heap_caps_malloc((REPORT_POINTS / 2U) * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }

    if (!s_vib_buffer || !s_fft_scratch || !s_fft_mag) {
        LOG_ERROR("Report pipeline buffer allocation failed");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void capture_handler(const imu_raw_data_t *data, size_t count, void *user_ctx)
{
    capture_ctx_t *ctx = (capture_ctx_t *)user_ctx;
    if (!ctx || !data || !s_vib_buffer) {
        return;
    }

    for (size_t i = 0; i < count && ctx->count < REPORT_POINTS; ++i) {
        const uint32_t idx = ctx->count++;
        s_vib_buffer[idx] = (float)data[i].x * ctx->lsb_to_g;
        s_vib_buffer[REPORT_POINTS + idx] = (float)data[i].y * ctx->lsb_to_g;
        s_vib_buffer[REPORT_POINTS * 2U + idx] = (float)data[i].z * ctx->lsb_to_g;
    }
    ctx->raw_count += (uint32_t)count;
}

static void evaluate_clip_quality(uint16_t range_g, capture_attempt_t *attempt)
{
    const float threshold = (float)range_g * REPORT_CLIP_THRESHOLD_RATIO;
    attempt->range_g = range_g;
    attempt->clipped = false;
    attempt->accepted = false;

    for (int axis = 0; axis < 3; ++axis) {
        const float *data = s_vib_buffer + REPORT_POINTS * (uint32_t)axis;
        axis_clip_quality_t *q = &attempt->axes[axis];
        memset(q, 0, sizeof(*q));

        for (uint32_t i = 0; i < REPORT_POINTS; ++i) {
            const float abs_v = fabsf(data[i]);
            if (abs_v > q->max_abs_g) {
                q->max_abs_g = abs_v;
            }
            if (abs_v >= threshold) {
                q->clip_count++;
            }
        }

        q->clip_ratio = (float)q->clip_count / (float)REPORT_POINTS;
        if (q->clip_count > 0U) {
            attempt->clipped = true;
        }
    }

    attempt->accepted = !attempt->clipped;
}

static esp_err_t capture_one_attempt(uint16_t range_g, capture_attempt_t *attempt)
{
    if (!attempt) {
        return ESP_ERR_INVALID_ARG;
    }

    capture_ctx_t ctx = {
        .count = 0,
        .raw_count = 0,
        .lsb_to_g = lsb_to_g_for_range(range_g),
    };
    iis3dwb_cfg_t cfg = cfg_for_range(range_g);

    memset(s_vib_buffer, 0, REPORT_POINTS * 3U * sizeof(float));
    esp_err_t err = daq_iis3dwb_capture(&cfg,
                                        capture_duration_ms(),
                                        capture_handler,
                                        &ctx,
                                        REPORT_DMA_CHUNK_SIZE,
                                        REPORT_CAPTURE_SKIP_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (ctx.count < REPORT_POINTS) {
        LOG_ERRORF("Report capture insufficient samples: %lu/%lu",
                   (unsigned long)ctx.count,
                   (unsigned long)REPORT_POINTS);
        return ESP_ERR_INVALID_SIZE;
    }

    evaluate_clip_quality(range_g, attempt);
    return ESP_OK;
}

static esp_err_t capture_with_auto_range(capture_attempt_t *attempts,
                                         size_t max_attempts,
                                         size_t *out_attempt_count,
                                         uint16_t *out_final_range,
                                         bool *out_accepted)
{
    if (!attempts || max_attempts == 0 || !out_attempt_count || !out_final_range || !out_accepted) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t range_g = REPORT_REQUESTED_RANGE_G;
    *out_attempt_count = 0;
    *out_final_range = range_g;
    *out_accepted = false;

    while (*out_attempt_count < max_attempts) {
        capture_attempt_t *attempt = &attempts[*out_attempt_count];
        memset(attempt, 0, sizeof(*attempt));

        LOG_INFOF("Report capture attempt: range=%ug points=%lu",
                  (unsigned)range_g,
                  (unsigned long)REPORT_POINTS);
        esp_err_t err = capture_one_attempt(range_g, attempt);
        (*out_attempt_count)++;
        *out_final_range = range_g;
        if (err != ESP_OK) {
            return err;
        }
        if (attempt->accepted) {
            *out_accepted = true;
            return ESP_OK;
        }
        if (range_g >= 16) {
            return ESP_OK;
        }
        range_g = next_range(range_g);
    }

    return ESP_OK;
}

static axis_time_features_t compute_time_features(const float *data)
{
    axis_time_features_t f = {0};
    double sum = 0.0;
    double sum_sq = 0.0;

    for (uint32_t i = 0; i < REPORT_POINTS; ++i) {
        const double v = data[i];
        const double av = fabs(v);
        sum += v;
        sum_sq += v * v;
        if ((float)av > f.peak_g) {
            f.peak_g = (float)av;
        }
    }

    const double mean = sum / (double)REPORT_POINTS;
    const double rms_sq = sum_sq / (double)REPORT_POINTS;
    f.rms_g = (float)sqrt(rms_sq);
    f.crest_factor = (f.rms_g > 0.0f) ? (f.peak_g / f.rms_g) : 0.0f;

    double m2 = 0.0;
    double m4 = 0.0;
    for (uint32_t i = 0; i < REPORT_POINTS; ++i) {
        const double d = (double)data[i] - mean;
        const double d2 = d * d;
        m2 += d2;
        m4 += d2 * d2;
    }
    m2 /= (double)REPORT_POINTS;
    m4 /= (double)REPORT_POINTS;
    f.kurtosis = (m2 > 0.0) ? (float)(m4 / (m2 * m2)) : 0.0f;
    return f;
}

static void try_insert_peak(report_peak_t *peaks, float freq_hz, float amp_g)
{
    if (amp_g <= 0.0f) {
        return;
    }

    for (int i = 0; i < REPORT_FFT_PEAK_COUNT; ++i) {
        if (amp_g <= peaks[i].amp_g) {
            continue;
        }
        for (int j = REPORT_FFT_PEAK_COUNT - 1; j > i; --j) {
            peaks[j] = peaks[j - 1];
        }
        peaks[i].freq_hz = freq_hz;
        peaks[i].amp_g = amp_g;
        return;
    }
}

static esp_err_t compute_freq_features(const float *data, axis_freq_features_t *out)
{
    if (!data || !out || !s_fft_scratch || !s_fft_mag) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    double mean = 0.0;
    for (uint32_t i = 0; i < REPORT_POINTS; ++i) {
        mean += data[i];
    }
    mean /= (double)REPORT_POINTS;

    for (uint32_t i = 0; i < REPORT_POINTS; ++i) {
        s_fft_scratch[i] = data[i] - (float)mean;
    }

    esp_err_t err = algo_fft_calculate(s_fft_scratch, s_fft_mag, REPORT_POINTS);
    if (err != ESP_OK) {
        return err;
    }

    const uint32_t half = REPORT_POINTS / 2U;
    const float bin_hz = REPORT_FS_HZ / (float)REPORT_POINTS;
    double amp_sum = 0.0;
    double weighted_freq_sum = 0.0;
    double energy_sum = 0.0;
    double band_energy[7] = {0};

    for (uint32_t i = 1; i < half; ++i) {
        const float freq_hz = (float)i * bin_hz;
        if (freq_hz > REPORT_FFT_PEAK_MAX_HZ) {
            break;
        }
        if (freq_hz < REPORT_FFT_PEAK_MIN_HZ) {
            continue;
        }

        const float amp = s_fft_mag[i];
        const double energy = (double)amp * (double)amp;
        amp_sum += amp;
        weighted_freq_sum += (double)freq_hz * (double)amp;
        energy_sum += energy;

        for (size_t b = 0; b < sizeof(s_bands) / sizeof(s_bands[0]); ++b) {
            const bool in_last = (b == (sizeof(s_bands) / sizeof(s_bands[0])) - 1U) &&
                                 freq_hz >= s_bands[b].min_hz &&
                                 freq_hz <= s_bands[b].max_hz;
            if (in_last || (freq_hz >= s_bands[b].min_hz && freq_hz < s_bands[b].max_hz)) {
                band_energy[b] += energy;
                break;
            }
        }

        if (i + 1U < half && s_fft_mag[i] >= s_fft_mag[i - 1U] && s_fft_mag[i] >= s_fft_mag[i + 1U]) {
            try_insert_peak(out->peaks, freq_hz, amp);
        }
    }

    out->spectral_centroid_hz = (amp_sum > 0.0) ? (float)(weighted_freq_sum / amp_sum) : 0.0f;
    if (energy_sum > 0.0) {
        double entropy = 0.0;
        uint32_t bins = 0;
        for (uint32_t i = 1; i < half; ++i) {
            const float freq_hz = (float)i * bin_hz;
            if (freq_hz > REPORT_FFT_PEAK_MAX_HZ) {
                break;
            }
            const double e = (double)s_fft_mag[i] * (double)s_fft_mag[i];
            if (e > 0.0) {
                const double p = e / energy_sum;
                entropy -= p * log(p);
            }
            bins++;
        }
        out->spectral_entropy = (bins > 1U) ? (float)(entropy / log((double)bins)) : 0.0f;
        for (size_t b = 0; b < sizeof(s_bands) / sizeof(s_bands[0]); ++b) {
            out->band_ratio[b] = (float)(band_energy[b] / energy_sum);
        }
    }

    return ESP_OK;
}

static cJSON *add_analysis_config(void)
{
    cJSON *cfg = cJSON_CreateObject();
    if (!cfg) {
        return NULL;
    }
    cJSON_AddNumberToObject(cfg, "fft_peak_count", REPORT_FFT_PEAK_COUNT);
    cJSON_AddNumberToObject(cfg, "fft_peak_min_hz", REPORT_FFT_PEAK_MIN_HZ);
    cJSON_AddNumberToObject(cfg, "fft_peak_max_hz", REPORT_FFT_PEAK_MAX_HZ);

    cJSON *bands = cJSON_AddArrayToObject(cfg, "band_energy_ranges_hz");
    if (!bands) {
        cJSON_Delete(cfg);
        return NULL;
    }
    for (size_t i = 0; i < sizeof(s_bands) / sizeof(s_bands[0]); ++i) {
        cJSON *band = cJSON_CreateObject();
        if (!band) {
            cJSON_Delete(cfg);
            return NULL;
        }
        cJSON_AddStringToObject(band, "key", s_bands[i].key);
        cJSON_AddNumberToObject(band, "min_hz", s_bands[i].min_hz);
        cJSON_AddNumberToObject(band, "max_hz", s_bands[i].max_hz);
        cJSON_AddItemToArray(bands, band);
    }
    return cfg;
}

static cJSON *add_quality(const capture_attempt_t *attempts, size_t attempt_count, bool accepted)
{
    cJSON *quality = cJSON_CreateObject();
    if (!quality) {
        return NULL;
    }
    cJSON_AddStringToObject(quality, "status", accepted ? "ok" : "clipped_at_max_range");
    cJSON_AddBoolToObject(quality, "auto_range", attempt_count > 1U);

    cJSON *arr = cJSON_AddArrayToObject(quality, "attempts");
    if (!arr) {
        cJSON_Delete(quality);
        return NULL;
    }

    static const char *axis_names[3] = {"X", "Y", "Z"};
    for (size_t i = 0; i < attempt_count; ++i) {
        const capture_attempt_t *a = &attempts[i];
        cJSON *item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "range_g", a->range_g);
        cJSON_AddBoolToObject(item, "accepted", a->accepted);
        cJSON_AddStringToObject(item, "reason", a->accepted ? "ok" : "clipped");
        cJSON_AddNumberToObject(item, "clip_threshold_g", (float)a->range_g * REPORT_CLIP_THRESHOLD_RATIO);
        cJSON *axes = cJSON_AddObjectToObject(item, "axes");
        for (int axis = 0; axis < 3; ++axis) {
            cJSON *q = cJSON_CreateObject();
            cJSON_AddNumberToObject(q, "max_abs_g", a->axes[axis].max_abs_g);
            cJSON_AddNumberToObject(q, "clip_count", a->axes[axis].clip_count);
            cJSON_AddNumberToObject(q, "clip_ratio", a->axes[axis].clip_ratio);
            cJSON_AddItemToObject(axes, axis_names[axis], q);
        }
        cJSON_AddItemToArray(arr, item);
    }

    return quality;
}

static void add_time_features(cJSON *axis, const axis_time_features_t *f)
{
    cJSON *time = cJSON_AddObjectToObject(axis, "time");
    cJSON_AddNumberToObject(time, "rms_g", f->rms_g);
    cJSON_AddNumberToObject(time, "peak_g", f->peak_g);
    cJSON_AddNumberToObject(time, "crest_factor", f->crest_factor);
    cJSON_AddNumberToObject(time, "kurtosis", f->kurtosis);
}

static void add_freq_features(cJSON *axis, const axis_freq_features_t *f)
{
    cJSON *freq = cJSON_AddObjectToObject(axis, "freq");
    cJSON *peaks = cJSON_AddArrayToObject(freq, "peaks");
    for (int i = 0; i < REPORT_FFT_PEAK_COUNT; ++i) {
        cJSON *peak = cJSON_CreateObject();
        cJSON_AddNumberToObject(peak, "freq_hz", f->peaks[i].freq_hz);
        cJSON_AddNumberToObject(peak, "amp_g", f->peaks[i].amp_g);
        cJSON_AddItemToArray(peaks, peak);
    }
    cJSON_AddNumberToObject(freq, "spectral_centroid_hz", f->spectral_centroid_hz);
    cJSON_AddNumberToObject(freq, "spectral_entropy", f->spectral_entropy);

    cJSON *bands = cJSON_AddObjectToObject(axis, "band_energy_ratio");
    for (size_t i = 0; i < sizeof(s_bands) / sizeof(s_bands[0]); ++i) {
        cJSON_AddNumberToObject(bands, s_bands[i].key, f->band_ratio[i]);
    }
}

static esp_err_t add_axis_features(cJSON *root)
{
    static const char *axis_names[3] = {"X", "Y", "Z"};
    cJSON *axes = cJSON_AddObjectToObject(root, "axis_features");
    if (!axes) {
        return ESP_ERR_NO_MEM;
    }

    for (int axis = 0; axis < 3; ++axis) {
        const float *data = s_vib_buffer + REPORT_POINTS * (uint32_t)axis;
        axis_time_features_t time_features = compute_time_features(data);
        axis_freq_features_t freq_features = {0};
        esp_err_t err = compute_freq_features(data, &freq_features);
        if (err != ESP_OK) {
            return err;
        }

        cJSON *axis_obj = cJSON_AddObjectToObject(axes, axis_names[axis]);
        add_time_features(axis_obj, &time_features);
        add_freq_features(axis_obj, &freq_features);
    }

    return ESP_OK;
}

static char *build_report_json(uint64_t ts_ms,
                               float temperature_c,
                               bool temperature_valid,
                               uint16_t final_range_g,
                               const capture_attempt_t *attempts,
                               size_t attempt_count,
                               bool accepted,
                               const char *task_id)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return NULL;
    }

    cJSON_AddNumberToObject(root, "schema_version", REPORT_SCHEMA_VERSION);
    cJSON_AddStringToObject(root, "sn", REPORT_SN);
    cJSON_AddNumberToObject(root, "ts_ms", (double)ts_ms);
    if (temperature_valid) {
        cJSON_AddNumberToObject(root, "temperature_c", temperature_c);
    } else {
        cJSON_AddNullToObject(root, "temperature_c");
    }
    cJSON_AddNumberToObject(root, "fs_hz", (int)REPORT_FS_HZ);
    cJSON_AddNumberToObject(root, "requested_range_g", REPORT_REQUESTED_RANGE_G);
    cJSON_AddNumberToObject(root, "range_g", final_range_g);
    cJSON_AddNumberToObject(root, "points", REPORT_POINTS);
    cJSON_AddStringToObject(root, "task_id", task_id ? task_id : "");
    cJSON_AddStringToObject(root, "sample_type", REPORT_SAMPLE_TYPE);

    cJSON *analysis_cfg = add_analysis_config();
    if (!analysis_cfg) {
        cJSON_Delete(root);
        return NULL;
    }
    cJSON_AddItemToObject(root, "analysis_config", analysis_cfg);

    cJSON *quality = add_quality(attempts, attempt_count, accepted);
    if (!quality) {
        cJSON_Delete(root);
        return NULL;
    }
    cJSON_AddItemToObject(root, "quality", quality);

    if (accepted) {
        if (add_axis_features(root) != ESP_OK) {
            cJSON_Delete(root);
            return NULL;
        }
    } else {
        cJSON_AddNullToObject(root, "axis_features");
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

static void log_report_json(const char *json)
{
    if (!json) {
        return;
    }

    const size_t len = strlen(json);
    LOG_INFOF("Report JSON begin, len=%u", (unsigned)len);
    for (size_t i = 0; i < len; i += 256U) {
        const size_t remaining = len - i;
        const size_t chunk = remaining > 256U ? 256U : remaining;
        LOG_INFOF("%.*s", (int)chunk, json + i);
    }
    LOG_INFO("Report JSON end");
}

esp_err_t report_pipeline_run(const char *task_id)
{
    esp_err_t err = ensure_buffers();
    if (err != ESP_OK) {
        return err;
    }

    const uint64_t ts_ms = current_epoch_ms_or_zero();
    float temperature_c = 0.0f;
    const bool temperature_valid = (g_t1820b_initialized &&
                                    drv_t1820b_read_temperature(&temperature_c) == ESP_OK);

    capture_attempt_t attempts[4] = {0};
    size_t attempt_count = 0;
    uint16_t final_range_g = REPORT_REQUESTED_RANGE_G;
    bool accepted = false;
    err = capture_with_auto_range(attempts,
                                  sizeof(attempts) / sizeof(attempts[0]),
                                  &attempt_count,
                                  &final_range_g,
                                  &accepted);
    if (err != ESP_OK) {
        return err;
    }

    char *json = build_report_json(ts_ms,
                                   temperature_c,
                                   temperature_valid,
                                   final_range_g,
                                   attempts,
                                   attempt_count,
                                   accepted,
                                   task_id);
    if (!json) {
        return ESP_ERR_NO_MEM;
    }

    log_report_json(json);

    free(json);
    return ESP_OK;
}
