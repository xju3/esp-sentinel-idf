#include "report_pipeline.h"

#include "algo_fft.h"
#include "config_manager.h"
#include "drv_iis3dwb.h"
#include "drv_ds18b20.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "http_proxy.h"
#include "logger.h"
#include "report_upload_cache.h"
#include "sdkconfig.h"
#include "server_report_task_scheduler.h"
#include "task_http_message.h"

#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define REPORT_SCHEMA_VERSION 3
#define REPORT_SAMPLE_TYPE "normal"
#define REPORT_FS_HZ 26667.0f
#define REPORT_MAX_POINTS MAX_ALLOWED_POINTS
#define REPORT_CAPTURE_SKIP_MS 20U
#define REPORT_CAPTURE_GUARD_MS 250U
#define REPORT_DMA_CHUNK_SIZE 512
#define REPORT_CLIP_THRESHOLD_RATIO 0.98f
#define REPORT_FFT_PEAK_COUNT 5
#define REPORT_FFT_PEAK_MIN_HZ 0.0f
#define REPORT_FFT_PEAK_MAX_HZ 5000.0f
#define REPORT_BAND_COUNT 5
#define REPORT_CAPTURE_ATTEMPTS 4
#define REPORT_BEARING_FAULT_COUNT 4
#define REPORT_BEARING_MAX_HARMONICS 16
#define REPORT_BEARING_HP_HZ 1000.0f
#define REPORT_BEARING_LP_HZ 10000.0f
#define REPORT_ENVELOPE_LP_HZ 1200.0f
#define REPORT_BEARING_MAX_ANALYSIS_HZ 1000.0f
#define REPORT_BEARING_MIN_SNR_DB 3.0f
#define REPORT_BEARING_NOISE_RADIUS_BINS 24U
#define REPORT_VELOCITY_MIN_HZ 10.0f
#define REPORT_VELOCITY_MAX_HZ 1000.0f
#define REPORT_HANN_POWER_CORRECTION (2.0 / 3.0)
#define REPORT_VELOCITY_BAND_COUNT 3

static float *s_vib_buffer;
static float *s_fft_scratch;
static float *s_fft_mag;
static float *s_fft_work_buf;
static uint32_t s_active_points = DEFAULT_CAPTURE_POINTS;
static uint16_t s_active_range_g = 2;

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
    float mean_g;
    float rms_acc_g;
    float peak_acc_g;
    float peak_to_peak_acc_g;
    float rms_vel_mm_s;
    float rms_vel_legacy_mm_s;
    float crest_factor;
    float kurtosis;
} axis_time_features_t;

typedef struct {
    report_peak_t peaks[REPORT_FFT_PEAK_COUNT];
    float spectral_centroid_hz;
    float spectral_entropy;
    float rms_vel_mm_s;
    float rms_vel_legacy_mm_s;
    float velocity_band_rms_mm_s[REPORT_VELOCITY_BAND_COUNT];
    float band_ratio[REPORT_BAND_COUNT];
} axis_freq_features_t;

typedef struct {
    uint8_t harmonic;
    float observed_hz;
    float snr_db;
} bearing_candidate_t;

typedef struct {
    uint8_t count;
    bearing_candidate_t items[REPORT_BEARING_MAX_HARMONICS];
} bearing_fault_candidates_t;

typedef struct {
    uint8_t status; /* 0=complete, 1=insufficient data */
    float envelope_kurtosis;
    bearing_fault_candidates_t faults[REPORT_BEARING_FAULT_COUNT];
} bearing_axis_features_t;

static const char *s_bearing_fault_names[REPORT_BEARING_FAULT_COUNT] = {
    "bpfo", "bpfi", "bsf", "ftf"
};

static const struct {
    const char *key;
    float min_hz;
    float max_hz;
} s_bands[] = {
    {"0_100", 0.0f, 100.0f},
    {"100_500", 100.0f, 500.0f},
    {"500_1000", 500.0f, 1000.0f},
    {"1000_2000", 1000.0f, 2000.0f},
    {"2000_5000", 2000.0f, 5000.0f},
};

static const struct {
    const char *key;
    float min_hz;
    float max_hz;
} s_velocity_bands[] = {
    {"10_100", 10.0f, 100.0f},
    {"100_500", 100.0f, 500.0f},
    {"500_1000", 500.0f, 1000.0f},
};

typedef struct {
    bool accept_server_tasks;
} report_upload_ctx_t;

struct report_payload {
    bool accept_server_tasks;
    char *json;
};

struct report_fft_payload {
    char task_id[64];
    uint8_t *data;
    size_t length;
};

static double round_to_decimals(double value, int decimals)
{
    double scale = 1.0;
    for (int i = 0; i < decimals; ++i) {
        scale *= 10.0;
    }
    return round(value * scale) / scale;
}

static cJSON *add_number_rounded(cJSON *object, const char *key, double value, int decimals)
{
    return cJSON_AddNumberToObject(object, key, round_to_decimals(value, decimals));
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
    const float capture_ms = ((float)s_active_points * 1000.0f) / REPORT_FS_HZ;
    return (uint32_t)ceilf(capture_ms) + REPORT_CAPTURE_SKIP_MS + REPORT_CAPTURE_GUARD_MS;
}

static bool report_points_valid(uint32_t points)
{
    return points > 0 && points <= REPORT_MAX_POINTS;
}

static bool report_range_valid(uint16_t range_g)
{
    return range_g == 2U || range_g == 4U || range_g == 8U || range_g == 16U;
}

static uint32_t configured_report_points(void)
{
    return g_user_config.fft_points > 0
               ? g_user_config.fft_points
               : DEFAULT_CAPTURE_POINTS;
}

static uint16_t configured_report_range_g(void)
{
    return report_range_valid((uint16_t)g_user_config.range_g)
               ? (uint16_t)g_user_config.range_g
               : 2;
}

static esp_err_t apply_report_configuration(void)
{
    const uint32_t points = configured_report_points();
    const uint16_t range_g = configured_report_range_g();

    if (!report_points_valid(points)) {
        LOG_ERRORF("Invalid report points: %lu", (unsigned long)points);
        return ESP_ERR_INVALID_ARG;
    }
    if (!report_range_valid(range_g)) {
        LOG_ERRORF("Invalid report range: %u", (unsigned)range_g);
        return ESP_ERR_INVALID_ARG;
    }

    s_active_points = points;
    s_active_range_g = range_g;
    return ESP_OK;
}

static esp_err_t ensure_buffers(void)
{
    s_vib_buffer = g_user_config.vib_buf;
    s_fft_scratch = g_user_config.fft_scratch;
    s_fft_mag = g_user_config.fft_mag;
    s_fft_work_buf = g_user_config.fft_work_buf;

    if (!s_vib_buffer || !s_fft_scratch || !s_fft_mag || !s_fft_work_buf) {
        LOG_ERROR("Report pipeline buffer not allocated by config_manager");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t ensure_temperature_conversion_started(void)
{
    if (!g_ds18b20_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const ds18b20_state_t state = drv_ds18b20_get_state();
    if (state == DS18B20_STATE_CONVERTING || state == DS18B20_STATE_READY) {
        return ESP_OK;
    }
    return drv_ds18b20_start_conversion();
}

static esp_err_t get_temperature_conversion_progress(int64_t *elapsed_us,
                                                     int64_t *remaining_us)
{
    if (!elapsed_us || !remaining_us) {
        return ESP_ERR_INVALID_ARG;
    }

    return drv_ds18b20_get_conversion_timing(elapsed_us, remaining_us);
}

static esp_err_t wait_for_temperature_conversion_at_read(float *temperature_c)
{
    if (!temperature_c) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t elapsed_us = 0;
    int64_t remaining_us = 0;
    esp_err_t err = get_temperature_conversion_progress(&elapsed_us,
                                                         &remaining_us);
    if (err != ESP_OK) {
        return err;
    }

    LOG_INFOF("DS18B20 conversion elapsed at read: %lld ms",
              (long long)(elapsed_us / 1000LL));
    LOG_INFOF("DS18B20 remaining wait at read: %lld ms",
              (long long)((remaining_us + 999LL) / 1000LL));

    while (remaining_us > 0) {
        const uint32_t remaining_ms =
            (uint32_t)((remaining_us + 999LL) / 1000LL);
        vTaskDelay(pdMS_TO_TICKS(remaining_ms));

        err = get_temperature_conversion_progress(&elapsed_us, &remaining_us);
        if (err != ESP_OK) {
            return err;
        }
    }

    return drv_ds18b20_read_conversion_result(temperature_c);
}

static void capture_handler(const imu_raw_data_t *data, size_t count, void *user_ctx)
{
    capture_ctx_t *ctx = (capture_ctx_t *)user_ctx;
    if (!ctx || !data || !s_vib_buffer) {
        return;
    }

    for (size_t i = 0; i < count && ctx->count < s_active_points; ++i) {
        const uint32_t idx = ctx->count++;
        s_vib_buffer[idx] = (float)data[i].x * ctx->lsb_to_g;
        s_vib_buffer[s_active_points + idx] = (float)data[i].y * ctx->lsb_to_g;
        s_vib_buffer[s_active_points * 2U + idx] = (float)data[i].z * ctx->lsb_to_g;
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
        const float *data = s_vib_buffer + s_active_points * (uint32_t)axis;
        axis_clip_quality_t *q = &attempt->axes[axis];
        memset(q, 0, sizeof(*q));

        for (uint32_t i = 0; i < s_active_points; ++i) {
            const float abs_v = fabsf(data[i]);
            if (abs_v > q->max_abs_g) {
                q->max_abs_g = abs_v;
            }
            if (abs_v >= threshold) {
                q->clip_count++;
            }
        }

        q->clip_ratio = (float)q->clip_count / (float)s_active_points;
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

    memset(s_vib_buffer, 0, s_active_points * 3U * sizeof(float));
    esp_err_t err = drv_iis3dwb_capture(&cfg,
                                        capture_duration_ms(),
                                        capture_handler,
                                        &ctx,
                                        REPORT_DMA_CHUNK_SIZE,
                                        REPORT_CAPTURE_SKIP_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (ctx.count < s_active_points) {
        LOG_ERRORF("Report capture insufficient samples: %lu/%lu",
                   (unsigned long)ctx.count,
                   (unsigned long)s_active_points);
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

    uint16_t range_g = s_active_range_g;
    *out_attempt_count = 0;
    *out_final_range = range_g;
    *out_accepted = false;

    while (*out_attempt_count < max_attempts) {
        capture_attempt_t *attempt = &attempts[*out_attempt_count];
        memset(attempt, 0, sizeof(*attempt));

        LOG_INFOF("Report capture attempt: range=%ug points=%lu",
                  (unsigned)range_g,
                  (unsigned long)s_active_points);
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

static axis_time_features_t compute_time_features(const float *ac_data, float mean_g)
{
    axis_time_features_t f = {0};
    double sum_sq = 0.0;
    float min_v = ac_data[0];
    float max_v = ac_data[0];
    f.mean_g = mean_g;

    for (uint32_t i = 0; i < s_active_points; ++i) {
        const double v = ac_data[i];
        const double av = fabs(v);
        sum_sq += v * v;
        if ((float)av > f.peak_acc_g) {
            f.peak_acc_g = (float)av;
        }
        if (ac_data[i] < min_v) {
            min_v = ac_data[i];
        }
        if (ac_data[i] > max_v) {
            max_v = ac_data[i];
        }
    }

    const double rms_sq = sum_sq / (double)s_active_points;
    f.rms_acc_g = (float)sqrt(rms_sq);
    f.peak_to_peak_acc_g = max_v - min_v;
    f.crest_factor = (f.rms_acc_g > 0.0f) ? (f.peak_acc_g / f.rms_acc_g) : 0.0f;

    double m2 = 0.0;
    double m4 = 0.0;
    for (uint32_t i = 0; i < s_active_points; ++i) {
        const double d = (double)ac_data[i];
        const double d2 = d * d;
        m2 += d2;
        m4 += d2 * d2;
    }
    m2 /= (double)s_active_points;
    m4 /= (double)s_active_points;
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

    if (data != s_fft_scratch) {
        memcpy(s_fft_scratch, data, s_active_points * sizeof(float));
    }

    esp_err_t err = algo_fft_calculate(s_fft_scratch, s_fft_mag, s_fft_work_buf, s_active_points);
    if (err != ESP_OK) {
        return err;
    }

    const uint32_t half = s_active_points / 2U;
    const float bin_hz = REPORT_FS_HZ / (float)s_active_points;
    double amp_sum = 0.0;
    double weighted_freq_sum = 0.0;
    double energy_sum = 0.0;
    double legacy_vel_rms_sq_sum = 0.0;
    double velocity_band_rms_sq[REPORT_VELOCITY_BAND_COUNT] = {0};
    double band_energy[REPORT_BAND_COUNT] = {0};

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
        const double accel_peak_m_s2 = (double)amp * 9.80665;
        const double vel_peak_m_s = accel_peak_m_s2 / (2.0 * M_PI * (double)freq_hz);
        amp_sum += amp;
        weighted_freq_sum += (double)freq_hz * (double)amp;
        energy_sum += energy;
        legacy_vel_rms_sq_sum += (vel_peak_m_s * vel_peak_m_s) * 0.5;

        if (freq_hz >= REPORT_VELOCITY_MIN_HZ &&
            freq_hz <= REPORT_VELOCITY_MAX_HZ) {
            const double corrected_velocity_power =
                (vel_peak_m_s * vel_peak_m_s) * 0.5 *
                REPORT_HANN_POWER_CORRECTION;
            for (size_t b = 0; b < REPORT_VELOCITY_BAND_COUNT; ++b) {
                const bool in_last = b == REPORT_VELOCITY_BAND_COUNT - 1U &&
                                     freq_hz >= s_velocity_bands[b].min_hz &&
                                     freq_hz <= s_velocity_bands[b].max_hz;
                if (in_last ||
                    (freq_hz >= s_velocity_bands[b].min_hz &&
                     freq_hz < s_velocity_bands[b].max_hz)) {
                    velocity_band_rms_sq[b] += corrected_velocity_power;
                    break;
                }
            }
        }

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
    double corrected_velocity_power_sum = 0.0;
    for (size_t b = 0; b < REPORT_VELOCITY_BAND_COUNT; ++b) {
        corrected_velocity_power_sum += velocity_band_rms_sq[b];
        out->velocity_band_rms_mm_s[b] =
            (float)(sqrt(velocity_band_rms_sq[b]) * 1000.0);
    }
    out->rms_vel_mm_s =
        (float)(sqrt(corrected_velocity_power_sum) * 1000.0);
    out->rms_vel_legacy_mm_s =
        (float)(sqrt(legacy_vel_rms_sq_sum) * 1000.0);
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

static void remove_dc_to_buffer(const float *data, float *out, float *out_mean_g)
{
    double sum = 0.0;
    for (uint32_t i = 0; i < s_active_points; ++i) {
        sum += data[i];
    }

    const float mean = (float)(sum / (double)s_active_points);
    for (uint32_t i = 0; i < s_active_points; ++i) {
        out[i] = data[i] - mean;
    }

    if (out_mean_g) {
        *out_mean_g = mean;
    }
}

static float bearing_fault_order(size_t fault_index)
{
    switch (fault_index) {
        case 0: return g_user_config.bearing.bpfo_order;
        case 1: return g_user_config.bearing.bpfi_order;
        case 2: return g_user_config.bearing.bsf_order;
        case 3: return g_user_config.bearing.ftf_order;
        default: return 0.0f;
    }
}

static bool build_envelope_signal(const float *data, float *out_kurtosis)
{
    if (!data || !out_kurtosis || !s_fft_scratch || s_active_points < 8U) {
        return false;
    }

    const float dt = 1.0f / REPORT_FS_HZ;
    const float hp_rc = 1.0f / (2.0f * (float)M_PI * REPORT_BEARING_HP_HZ);
    const float bp_rc = 1.0f / (2.0f * (float)M_PI * REPORT_BEARING_LP_HZ);
    const float env_rc = 1.0f / (2.0f * (float)M_PI * REPORT_ENVELOPE_LP_HZ);
    const float hp_alpha = hp_rc / (hp_rc + dt);
    const float bp_alpha = dt / (bp_rc + dt);
    const float env_alpha = dt / (env_rc + dt);

    float previous_input = data[0];
    float high_pass = 0.0f;
    float band_pass = 0.0f;
    float envelope = 0.0f;
    double envelope_sum = 0.0;
    for (uint32_t i = 0; i < s_active_points; ++i) {
        const float input = data[i];
        high_pass = hp_alpha * (high_pass + input - previous_input);
        previous_input = input;
        band_pass += bp_alpha * (high_pass - band_pass);
        envelope += env_alpha * (fabsf(band_pass) - envelope);
        s_fft_scratch[i] = envelope;
        envelope_sum += envelope;
    }

    const float mean = (float)(envelope_sum / (double)s_active_points);
    double m2 = 0.0;
    double m4 = 0.0;
    for (uint32_t i = 0; i < s_active_points; ++i) {
        const float centered = s_fft_scratch[i] - mean;
        const double squared = (double)centered * (double)centered;
        s_fft_scratch[i] = centered;
        m2 += squared;
        m4 += squared * squared;
    }
    m2 /= (double)s_active_points;
    m4 /= (double)s_active_points;
    if (m2 <= 1e-18 || !isfinite(m2) || !isfinite(m4)) {
        return false;
    }

    *out_kurtosis = (float)(m4 / (m2 * m2));
    return isfinite(*out_kurtosis);
}

static float local_noise_rms(uint32_t peak_bin,
                             uint32_t search_start,
                             uint32_t search_end,
                             uint32_t half)
{
    const uint32_t noise_start =
        peak_bin > REPORT_BEARING_NOISE_RADIUS_BINS
            ? peak_bin - REPORT_BEARING_NOISE_RADIUS_BINS
            : 1U;
    uint32_t noise_end = peak_bin + REPORT_BEARING_NOISE_RADIUS_BINS;
    if (noise_end >= half) {
        noise_end = half - 1U;
    }

    double sum_sq = 0.0;
    uint32_t count = 0;
    for (uint32_t bin = noise_start; bin <= noise_end; ++bin) {
        if (bin >= search_start && bin <= search_end) {
            continue;
        }
        const double amplitude = s_fft_mag[bin];
        sum_sq += amplitude * amplitude;
        count++;
    }
    return count > 0U ? (float)sqrt(sum_sq / (double)count) : 0.0f;
}

static void extract_bearing_candidates(bearing_axis_features_t *out)
{
    const uint32_t half = s_active_points / 2U;
    const float bin_hz = REPORT_FS_HZ / (float)s_active_points;
    const float shaft_hz = g_user_config.bearing.shaft_rpm / 60.0f;
    const float max_hz = fminf(REPORT_BEARING_MAX_ANALYSIS_HZ,
                              REPORT_FS_HZ * 0.5f - bin_hz);

    for (size_t fault = 0; fault < REPORT_BEARING_FAULT_COUNT; ++fault) {
        const float base_hz = shaft_hz * bearing_fault_order(fault);
        bearing_fault_candidates_t *fault_out = &out->faults[fault];
        if (!(base_hz > 0.0f) || !isfinite(base_hz)) {
            continue;
        }

        for (uint8_t harmonic = 1; harmonic <= REPORT_BEARING_MAX_HARMONICS; ++harmonic) {
            const float expected_hz = base_hz * (float)harmonic;
            if (expected_hz > max_hz) {
                break;
            }

            const float tolerance_hz = fmaxf(2.0f * bin_hz, expected_hz * 0.02f);
            const float lower_hz = fmaxf(0.0f, expected_hz - tolerance_hz);
            uint32_t start = (uint32_t)floorf(lower_hz / bin_hz);
            uint32_t end = (uint32_t)ceilf((expected_hz + tolerance_hz) / bin_hz);
            if (start < 1U) start = 1U;
            if (end >= half) end = half - 1U;
            if (start > end) continue;

            uint32_t peak_bin = start;
            for (uint32_t bin = start + 1U; bin <= end; ++bin) {
                if (s_fft_mag[bin] > s_fft_mag[peak_bin]) {
                    peak_bin = bin;
                }
            }

            const float peak_amplitude = s_fft_mag[peak_bin];
            const float noise_rms = local_noise_rms(peak_bin, start, end, half);
            if (!(peak_amplitude > 0.0f) || !(noise_rms > 0.0f)) {
                continue;
            }

            const float snr_db = 20.0f * log10f(peak_amplitude / noise_rms);
            if (!isfinite(snr_db) || snr_db < REPORT_BEARING_MIN_SNR_DB) {
                continue;
            }

            bearing_candidate_t *candidate = &fault_out->items[fault_out->count++];
            candidate->harmonic = harmonic;
            candidate->observed_hz = (float)peak_bin * bin_hz;
            candidate->snr_db = snr_db;
        }
    }
}

static esp_err_t compute_bearing_features(const float *data,
                                          bearing_axis_features_t *out)
{
    if (!data || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    out->status = 1;

    if (!build_envelope_signal(data, &out->envelope_kurtosis)) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = algo_fft_calculate(s_fft_scratch,
                                       s_fft_mag,
                                       s_fft_work_buf,
                                       s_active_points);
    if (err != ESP_OK) {
        return err;
    }

    extract_bearing_candidates(out);
    out->status = 0;
    return ESP_OK;
}

static cJSON *add_quality(const capture_attempt_t *attempts, size_t attempt_count, bool accepted)
{
    cJSON *quality = cJSON_CreateObject();
    if (!quality) {
        return NULL;
    }
    cJSON_AddNumberToObject(quality, "status", accepted ? 0 : 1);
    cJSON_AddBoolToObject(quality, "auto_range", attempt_count > 1U);

    if (attempt_count > 1U || !accepted) {
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
            add_number_rounded(item, "clip_threshold_g", (float)a->range_g * REPORT_CLIP_THRESHOLD_RATIO, 3);
            cJSON *axes = cJSON_AddObjectToObject(item, "axes");
            for (int axis = 0; axis < 3; ++axis) {
                cJSON *q = cJSON_CreateObject();
                add_number_rounded(q, "max_abs_g", a->axes[axis].max_abs_g, 3);
                cJSON_AddNumberToObject(q, "clip_count", a->axes[axis].clip_count);
                add_number_rounded(q, "clip_ratio", a->axes[axis].clip_ratio, 6);
                cJSON_AddItemToObject(axes, axis_names[axis], q);
            }
            cJSON_AddItemToArray(arr, item);
        }
    }

    return quality;
}

static void add_time_features(cJSON *axis, const axis_time_features_t *f)
{
    cJSON *time = cJSON_AddObjectToObject(axis, "time");
    add_number_rounded(time, "mean_g", f->mean_g, 3);
    add_number_rounded(time, "rms_acc_g", f->rms_acc_g, 3);
    add_number_rounded(time, "peak_acc_g", f->peak_acc_g, 3);
    add_number_rounded(time, "peak_to_peak_acc_g", f->peak_to_peak_acc_g, 3);
    add_number_rounded(time, "rms_vel_mm_s", f->rms_vel_mm_s, 2);
    add_number_rounded(time, "rms_vel_legacy_mm_s",
                       f->rms_vel_legacy_mm_s, 2);
    add_number_rounded(time, "crest_factor", f->crest_factor, 2);
    add_number_rounded(time, "kurtosis", f->kurtosis, 2);
}

static void add_freq_features(cJSON *axis, const axis_freq_features_t *f)
{
    cJSON *freq = cJSON_AddObjectToObject(axis, "freq");
    cJSON *peaks = cJSON_AddArrayToObject(freq, "peaks");
    for (int i = 0; i < REPORT_FFT_PEAK_COUNT; ++i) {
        cJSON *peak = cJSON_CreateObject();
        add_number_rounded(peak, "freq_hz", f->peaks[i].freq_hz, 1);
        add_number_rounded(peak, "amp_g", f->peaks[i].amp_g, 4);
        cJSON_AddItemToArray(peaks, peak);
    }
    add_number_rounded(freq, "spectral_centroid_hz", f->spectral_centroid_hz, 1);
    add_number_rounded(freq, "spectral_entropy", f->spectral_entropy, 3);

    cJSON *bands = cJSON_AddObjectToObject(axis, "band_energy_ratio");
    for (size_t i = 0; i < sizeof(s_bands) / sizeof(s_bands[0]); ++i) {
        add_number_rounded(bands, s_bands[i].key, f->band_ratio[i], 3);
    }

    cJSON *velocity_bands =
        cJSON_AddObjectToObject(axis, "band_rms_vel_mm_s");
    for (size_t i = 0; i < REPORT_VELOCITY_BAND_COUNT; ++i) {
        add_number_rounded(velocity_bands,
                           s_velocity_bands[i].key,
                           f->velocity_band_rms_mm_s[i],
                           2);
    }
}

static void add_bearing_axis_features(cJSON *axis,
                                      const bearing_axis_features_t *features)
{
    cJSON_AddNumberToObject(axis, "status", features->status);
    if (features->status == 0U) {
        add_number_rounded(axis, "envelope_kurtosis", features->envelope_kurtosis, 2);
    } else {
        cJSON_AddNullToObject(axis, "envelope_kurtosis");
    }

    cJSON *faults = cJSON_AddObjectToObject(axis, "fault_candidates");
    if (features->status != 0U || !faults) {
        return;
    }
    for (size_t fault = 0; fault < REPORT_BEARING_FAULT_COUNT; ++fault) {
        const bearing_fault_candidates_t *candidates = &features->faults[fault];
        if (candidates->count == 0U) {
            continue;
        }
        cJSON *array = cJSON_AddArrayToObject(faults, s_bearing_fault_names[fault]);
        for (uint8_t i = 0; i < candidates->count; ++i) {
            const bearing_candidate_t *candidate = &candidates->items[i];
            cJSON *item = cJSON_CreateObject();
            cJSON_AddNumberToObject(item, "harmonic", candidate->harmonic);
            add_number_rounded(item, "observed_hz", candidate->observed_hz, 2);
            add_number_rounded(item, "snr_db", candidate->snr_db, 2);
            cJSON_AddItemToArray(array, item);
        }
    }
}

static esp_err_t add_axis_features(cJSON *root)
{
    static const char *axis_names[3] = {"X", "Y", "Z"};
    cJSON *axes = cJSON_AddObjectToObject(root, "axis_features");
    if (!axes) {
        return ESP_ERR_NO_MEM;
    }
    cJSON *bearing_axes = NULL;
    const bool rpm_features_enabled =
        config_manager_get_device_rpm() > 0;
    if (g_user_config.bearing.configured && rpm_features_enabled) {
        bearing_axes = cJSON_AddObjectToObject(root, "bearing_features");
        if (!bearing_axes) {
            return ESP_ERR_NO_MEM;
        }
    } else if (g_user_config.bearing.configured) {
        LOG_WARNF("RPM is %ld; skipping RPM-dependent bearing features",
                  (long)config_manager_get_device_rpm());
    }

    for (int axis = 0; axis < 3; ++axis) {
        const float *raw_data = s_vib_buffer + s_active_points * (uint32_t)axis;
        float mean_g = 0.0f;
        remove_dc_to_buffer(raw_data, s_fft_scratch, &mean_g);

        axis_time_features_t time_features = compute_time_features(s_fft_scratch, mean_g);
        axis_freq_features_t freq_features = {0};
        esp_err_t err = compute_freq_features(s_fft_scratch, &freq_features);
        if (err != ESP_OK) {
            return err;
        }
        time_features.rms_vel_mm_s = freq_features.rms_vel_mm_s;
        time_features.rms_vel_legacy_mm_s =
            freq_features.rms_vel_legacy_mm_s;

        cJSON *axis_obj = cJSON_AddObjectToObject(axes, axis_names[axis]);
        add_time_features(axis_obj, &time_features);
        add_freq_features(axis_obj, &freq_features);

        if (bearing_axes) {
            bearing_axis_features_t bearing_features = {0};
            esp_err_t bearing_err = compute_bearing_features(raw_data, &bearing_features);
            if (bearing_err != ESP_OK) {
                LOG_WARNF("Bearing feature extraction failed for %s: %s",
                          axis_names[axis], esp_err_to_name(bearing_err));
            }
            cJSON *bearing_axis = cJSON_AddObjectToObject(bearing_axes, axis_names[axis]);
            add_bearing_axis_features(bearing_axis, &bearing_features);
        }
    }

    return ESP_OK;
}

static char *build_report_json(float temperature_c,
                               bool temperature_valid,
                               uint16_t final_range_g,
                               const capture_attempt_t *attempts,
                               size_t attempt_count,
                               bool accepted,
                               const char *task_id,
                               uint32_t duration_ms)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return NULL;
    }

    cJSON_AddNumberToObject(root, "schema_version", REPORT_SCHEMA_VERSION);
    cJSON_AddStringToObject(root, "sensor_sn", g_user_config.sn);
    cJSON_AddStringToObject(root, "device_id", g_user_config.device_id);
    cJSON_AddNumberToObject(root, "period", g_user_config.patrol);
    if (temperature_valid) {
        add_number_rounded(root, "temperature_c", temperature_c, 1);
    } else {
        cJSON_AddNullToObject(root, "temperature_c");
    }
    cJSON_AddNumberToObject(root, "fs_hz", (int)REPORT_FS_HZ);
    cJSON_AddNumberToObject(root, "requested_range_g", s_active_range_g);
    cJSON_AddNumberToObject(root, "range_g", final_range_g);
    cJSON_AddNumberToObject(root, "points", s_active_points);
    cJSON_AddStringToObject(root, "task_id", task_id ? task_id : "");
    cJSON_AddStringToObject(root, "sample_type", REPORT_SAMPLE_TYPE);
    cJSON_AddNumberToObject(root, "duration_ms", duration_ms);

    cJSON *velocity_band =
        cJSON_AddObjectToObject(root, "velocity_rms_band_hz");
    if (!velocity_band) {
        cJSON_Delete(root);
        return NULL;
    }
    cJSON_AddNumberToObject(velocity_band, "min", REPORT_VELOCITY_MIN_HZ);
    cJSON_AddNumberToObject(velocity_band, "max", REPORT_VELOCITY_MAX_HZ);
    cJSON_AddStringToObject(velocity_band, "window", "hann");
    add_number_rounded(velocity_band,
                       "power_correction",
                       REPORT_HANN_POWER_CORRECTION,
                       6);

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

static esp_err_t update_report_duration(char **json)
{
    if (!json || !*json) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_Parse(*json);
    if (!root) {
        return ESP_FAIL;
    }

    cJSON *duration = cJSON_GetObjectItemCaseSensitive(root, "duration_ms");
    if (!cJSON_IsNumber(duration)) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    cJSON_SetNumberValue(duration, (double)(esp_timer_get_time() / 1000ULL));
    char *updated = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!updated) {
        return ESP_ERR_NO_MEM;
    }

    free(*json);
    *json = updated;
    return ESP_OK;
}

static esp_err_t update_report_delay(char **json, uint32_t delay_s)
{
    if (!json || !*json) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_Parse(*json);
    if (!cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint32_t total = report_upload_cache_get_count();

    cJSON_DeleteItemFromObjectCaseSensitive(root, "seq");
    cJSON_DeleteItemFromObjectCaseSensitive(root, "delay");
    cJSON_DeleteItemFromObjectCaseSensitive(root, "total");
    if (!cJSON_AddNumberToObject(root, "delay", delay_s)) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    if (!cJSON_AddNumberToObject(root, "total", total)) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    char *updated = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!updated) {
        return ESP_ERR_NO_MEM;
    }

    free(*json);
    *json = updated;
    return ESP_OK;
}

static void handle_upload_response_tasks(const cJSON *data, bool accept_server_tasks)
{
    if (!cJSON_IsArray(data)) {
        LOG_INFO("Report JSON upload completed");
        return;
    }

    const int task_count = cJSON_GetArraySize(data);
    LOG_INFOF("Report JSON upload completed, returned_tasks=%d", task_count);

    cJSON *item = NULL;
    cJSON_ArrayForEach(item, data)
    {
        const cJSON *task_id = cJSON_GetObjectItemCaseSensitive(item, "id");
        const cJSON *action = cJSON_GetObjectItemCaseSensitive(item, "action");
        const cJSON *val = cJSON_GetObjectItemCaseSensitive(item, "val");
        if (cJSON_IsString(task_id) && cJSON_IsNumber(action)) {
            LOG_INFOF("Returned task: id=%s, action=%d, val=%d",
                      task_id->valuestring,
                      action->valueint,
                      cJSON_IsNumber(val) ? val->valueint : 0);
        }
    }

    if (accept_server_tasks) {
        (void)http_message_process_task_array(data);
    }
}

static esp_err_t send_report_json_once(const char *json, void *ctx)
{
    const report_upload_ctx_t *upload_ctx = (const report_upload_ctx_t *)ctx;
    const bool accept_server_tasks = upload_ctx && upload_ctx->accept_server_tasks;

    if (!json) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_user_config.api_host[0] == '\0') {
        LOG_ERROR("Cannot upload report JSON: api_host is empty");
        return ESP_ERR_INVALID_STATE;
    }

    char *url = malloc(256);
    if (!url) {
        return ESP_ERR_NO_MEM;
    }
    snprintf(url, 256, "http://%s/api/v1/sensors/data", g_user_config.api_host);
    // LOG_INFOF("Uploading report JSON to %s", url);

    char *response = NULL;
    esp_err_t err = http_proxy_post_json(url, json, &response);
    free(url);
    if (err != ESP_OK) {
        LOG_WARNF("Report JSON upload failed: %s", esp_err_to_name(err));
        free(response);
        return err;
    }

    if (!response) {
        LOG_WARN("Report JSON upload failed: empty server response");
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(response);
    if (!root) {
        LOG_WARNF("Report JSON upload failed: invalid server response: %s", response);
        free(response);
        return ESP_FAIL;
    }

    const cJSON *code = cJSON_GetObjectItemCaseSensitive(root, "code");
    if (!cJSON_IsNumber(code) || code->valueint != 0) {
        const cJSON *message = cJSON_GetObjectItemCaseSensitive(root, "message");
        LOG_WARNF("Report JSON upload rejected: code=%d, message=%s",
                  cJSON_IsNumber(code) ? code->valueint : -1,
                  cJSON_IsString(message) ? message->valuestring : "");
        cJSON_Delete(root);
        free(response);
        return ESP_FAIL;
    }

    const cJSON *data = cJSON_GetObjectItemCaseSensitive(root, "data");
    handle_upload_response_tasks(data, accept_server_tasks);

    cJSON_Delete(root);
    free(response);
    return ESP_OK;
}

static esp_err_t post_report_json(const char *json, bool accept_server_tasks)
{
    report_upload_ctx_t upload_ctx = {
        .accept_server_tasks = accept_server_tasks,
    };
    return send_report_json_once(json, &upload_ctx);
}

esp_err_t report_pipeline_capture_fft(
    const char *task_id,
    report_sample_complete_fn sample_complete,
    void *sample_complete_ctx,
    report_fft_payload_t **out_payload)
{
    if (!task_id || task_id[0] == '\0' || !out_payload) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_payload = NULL;

    esp_err_t err = apply_report_configuration();
    if (err != ESP_OK) {
        return err;
    }
    err = ensure_buffers();
    if (err != ESP_OK) {
        return err;
    }

    capture_attempt_t attempts[REPORT_CAPTURE_ATTEMPTS] = {0};
    size_t attempt_count = 0;
    uint16_t final_range_g = s_active_range_g;
    bool accepted = false;
    LOG_INFOF("Executing action=99 FFT capture: task_id=%s, points=%lu",
              task_id, (unsigned long)s_active_points);
    err = capture_with_auto_range(attempts,
                                  REPORT_CAPTURE_ATTEMPTS,
                                  &attempt_count,
                                  &final_range_g,
                                  &accepted);
    if (err != ESP_OK) {
        return err;
    }
    if (!accepted) {
        LOG_ERROR("FFT capture remained clipped at maximum range; not uploading spectrum");
        return ESP_ERR_INVALID_STATE;
    }

    if (sample_complete) {
        sample_complete(sample_complete_ctx);
    }

    const uint32_t bins = s_active_points / 2U;
    const size_t axis_bytes = (size_t)bins * sizeof(float);
    const size_t payload_length = 32U + axis_bytes * 3U;
    report_fft_payload_t *payload = calloc(1, sizeof(*payload));
    if (!payload) {
        return ESP_ERR_NO_MEM;
    }
    payload->data = heap_caps_calloc(1, payload_length,
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!payload->data) {
        free(payload);
        return ESP_ERR_NO_MEM;
    }
    payload->length = payload_length;
    snprintf(payload->task_id, sizeof(payload->task_id), "%s", task_id);

    const size_t sn_len = strnlen(g_user_config.sn, 16U);
    memcpy(payload->data, g_user_config.sn, sn_len);
    time_t now = time(NULL);
    uint32_t timestamp_s = now > 0 && (uint64_t)now <= UINT32_MAX
                               ? (uint32_t)now
                               : 0U;
    uint32_t range_g = final_range_g;
    memcpy(payload->data + 16U, &timestamp_s, sizeof(timestamp_s));
    memcpy(payload->data + 20U, &s_active_points, sizeof(s_active_points));
    const float fs_hz = REPORT_FS_HZ;
    memcpy(payload->data + 24U, &fs_hz, sizeof(fs_hz));
    memcpy(payload->data + 28U, &range_g, sizeof(range_g));

    uint8_t *axis_output = payload->data + 32U;
    for (uint32_t axis = 0; axis < 3U; ++axis) {
        const float *raw = s_vib_buffer + s_active_points * axis;
        remove_dc_to_buffer(raw, s_fft_scratch, NULL);
        err = algo_fft_calculate(s_fft_scratch,
                                 s_fft_mag,
                                 s_fft_work_buf,
                                 s_active_points);
        if (err != ESP_OK) {
            report_pipeline_discard_fft(payload);
            return err;
        }
        memcpy(axis_output + axis_bytes * axis, s_fft_mag, axis_bytes);
    }

    *out_payload = payload;
    return ESP_OK;
}

esp_err_t report_pipeline_upload_fft(report_fft_payload_t *payload)
{
    if (!payload || !payload->data || payload->length == 0U ||
        payload->task_id[0] == '\0') {
        report_pipeline_discard_fft(payload);
        return ESP_ERR_INVALID_ARG;
    }

    char url[256];
    snprintf(url, sizeof(url), "http://%s/api/v1/sensors/tasks/%s/fft",
             g_user_config.api_host, payload->task_id);
    char *response = NULL;
    esp_err_t err = http_proxy_post_binary(url,
                                           payload->data,
                                           payload->length,
                                           &response);
    if (err == ESP_OK) {
        cJSON *root = response ? cJSON_Parse(response) : NULL;
        const cJSON *code = root
                                ? cJSON_GetObjectItemCaseSensitive(root, "code")
                                : NULL;
        if (!cJSON_IsNumber(code) || code->valueint != 0) {
            LOG_WARN("FFT binary upload returned an invalid server response");
            err = ESP_FAIL;
        }
        cJSON_Delete(root);
    }
    if (err != ESP_OK) {
        LOG_WARNF("FFT binary upload failed: task_id=%s, err=%s",
                  payload->task_id, esp_err_to_name(err));
    } else {
        LOG_INFOF("FFT binary uploaded: task_id=%s, bytes=%u",
                  payload->task_id, (unsigned)payload->length);
    }
    free(response);
    report_pipeline_discard_fft(payload);
    return err;
}

void report_pipeline_discard_fft(report_fft_payload_t *payload)
{
    if (!payload) {
        return;
    }
    heap_caps_free(payload->data);
    payload->data = NULL;
    free(payload);
}

static esp_err_t report_pipeline_capture_internal(
    const char *task_id,
    bool include_temperature,
    report_sample_complete_fn sample_complete,
    void *sample_complete_ctx,
    report_payload_t **out_payload)
{
    if (!out_payload) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_payload = NULL;

    esp_err_t temperature_err = ESP_ERR_NOT_SUPPORTED;
    if (include_temperature) {
        temperature_err = ensure_temperature_conversion_started();
    }

    esp_err_t err = apply_report_configuration();
    if (err != ESP_OK) {
        return err;
    }

    err = ensure_buffers();
    if (err != ESP_OK) {
        return err;
    }

    int64_t conversion_elapsed_us = 0;
    int64_t conversion_remaining_us = 0;
    if (temperature_err == ESP_OK) {
        temperature_err = get_temperature_conversion_progress(
            &conversion_elapsed_us, &conversion_remaining_us);
        if (temperature_err == ESP_OK) {
            LOG_INFOF("DS18B20 conversion elapsed before IIS3DWB capture: %lld ms",
                      (long long)(conversion_elapsed_us / 1000LL));
        }
    }

    capture_attempt_t attempts[REPORT_CAPTURE_ATTEMPTS] = {0};
    size_t attempt_count = 0;
    uint16_t final_range_g = s_active_range_g;
    bool accepted = false;

    if (temperature_err == ESP_OK && conversion_remaining_us > 0) {
        LOG_INFO("IIS3DWB sampling started while DS18B20 converting");
    } else {
        LOG_INFO("IIS3DWB sampling started");
    }

    err = capture_with_auto_range(attempts,
                                  REPORT_CAPTURE_ATTEMPTS,
                                  &attempt_count,
                                  &final_range_g,
                                  &accepted);
    if (err != ESP_OK) {
        return err;
    }
    LOG_INFO("IIS3DWB sampling completed");

    float temperature_c = 0.0f;
    bool temperature_valid = false;
    if (temperature_err == ESP_OK) {
        temperature_err = wait_for_temperature_conversion_at_read(
            &temperature_c);
        temperature_valid = temperature_err == ESP_OK;
    }
    if (include_temperature && temperature_err != ESP_OK) {
        LOG_WARNF("DS18B20 temperature read failed: %s",
                  esp_err_to_name(temperature_err));
    }

    // All auto-range attempts are complete. From this point onward the raw
    // vibration buffer is only read for feature calculation, and the
    // synchronous DS18B20 result has been collected. External work that must
    // not overlap IIS3DWB sampling can safely begin.
    if (sample_complete) {
        sample_complete(sample_complete_ctx);
    }

    report_payload_t *payload = calloc(1, sizeof(*payload));
    if (!payload) {
        return ESP_ERR_NO_MEM;
    }

    const uint32_t duration_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    payload->json = build_report_json(temperature_c,
                                      temperature_valid,
                                      final_range_g,
                                      attempts,
                                      attempt_count,
                                      accepted,
                                      task_id,
                                      duration_ms);
    if (!payload->json) {
        report_pipeline_discard(payload);
        return ESP_ERR_NO_MEM;
    }

    payload->accept_server_tasks = task_id == NULL || task_id[0] == '\0';
    *out_payload = payload;
    return ESP_OK;
}

esp_err_t report_pipeline_capture_with_sample_complete(
    const char *task_id,
    report_sample_complete_fn sample_complete,
    void *sample_complete_ctx,
    report_payload_t **out_payload)
{
    return report_pipeline_capture_internal(task_id,
                                            true,
                                            sample_complete,
                                            sample_complete_ctx,
                                            out_payload);
}

esp_err_t report_pipeline_capture_vibration_only(
    const char *task_id,
    report_payload_t **out_payload)
{
    return report_pipeline_capture_internal(task_id,
                                            false,
                                            NULL,
                                            NULL,
                                            out_payload);
}

esp_err_t report_pipeline_capture(const char *task_id,
                                  report_payload_t **out_payload)
{
    return report_pipeline_capture_with_sample_complete(task_id,
                                                        NULL,
                                                        NULL,
                                                        out_payload);
}

esp_err_t report_pipeline_upload(report_payload_t *payload)
{
    if (!payload || !payload->json) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = update_report_duration(&payload->json);
    if (err != ESP_OK) {
        report_pipeline_discard(payload);
        return err;
    }

    // The report captured in this wake cycle is the time anchor.
    err = update_report_delay(&payload->json, 0);
    if (err != ESP_OK) {
        report_pipeline_discard(payload);
        return err;
    }

    err = post_report_json(payload->json, payload->accept_server_tasks);
    if (err != ESP_OK) {
        esp_err_t cache_err = report_pipeline_cache(payload);
        if (cache_err != ESP_OK) {
            LOG_ERRORF("Failed to persist report after upload failure: %s",
                       esp_err_to_name(cache_err));
        }
    }
    report_pipeline_discard(payload);
    return err;
}

esp_err_t report_pipeline_cache(report_payload_t *payload)
{
    if (!payload || !payload->json) {
        return ESP_ERR_INVALID_ARG;
    }
    return report_upload_cache_store(payload->json, NULL);
}

const char *report_pipeline_payload_json(const report_payload_t *payload)
{
    return payload ? payload->json : NULL;
}

esp_err_t report_pipeline_flush_cache(void)
{
    const report_upload_ctx_t upload_ctx = {
        .accept_server_tasks = false,
    };
    return report_upload_cache_flush(send_report_json_once,
                                     (void *)&upload_ctx);
}

void report_pipeline_discard(report_payload_t *payload)
{
    if (!payload) {
        return;
    }
    free(payload->json);
    free(payload);
}
