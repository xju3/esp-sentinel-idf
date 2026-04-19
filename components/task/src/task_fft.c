#include "task_fft.h"
#include "algo_fft.h"
#include "logger.h"
#include "config_manager.h"
#include "data_dispatcher.h"
#include "task_diag_fusion.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAX_DAQ_SAMPLES 8192
#define FFT_QUEUE_LEN   5
#define TASK_MEM_CAPS   (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define PATROL_LOG_LOW_FREQ_MAX_HZ 200.0f

QueueHandle_t g_fft_job_queue = NULL;
static volatile bool s_fft_task_busy = false;
static patrol_fft_report_t s_patrol_fft_report;

#define RPM_HINT_MIN_FACTOR       0.70f
#define RPM_HINT_MAX_FACTOR       1.30f
#define ORDER_MATCH_TOLERANCE     0.12f
#define REFINED_ORDER_TOLERANCE   0.08f
#define AXIAL_CANDIDATE_BOOST     1.20f
#define AXIAL_HARMONIC_BOOST      1.10f

typedef struct
{
    float estimated_hz;
    float refined_hz;
    int peak_1x_idx;
    int peak_2x_idx;
    int peak_3x_idx;
} order_analysis_t;

static float peak_score(const patrol_peak_t *peak)
{
    if (!peak)
    {
        return 0.0f;
    }

    float score = peak->amp_x;
    if (peak->amp_y > score)
    {
        score = peak->amp_y;
    }
    if (peak->amp_z > score)
    {
        score = peak->amp_z;
    }
    return score;
}

static char position_code_for_axis(const sensor_position_t *pos, uint8_t axis)
{
    if (!pos)
    {
        return '?';
    }

    switch (axis)
    {
        case 0: return pos->x;
        case 1: return pos->y;
        case 2: return pos->z;
        default: return '?';
    }
}

static int axis_index_for_position(const sensor_position_t *pos, char pos_code)
{
    if (!pos)
    {
        return -1;
    }

    const char target = (char)tolower((unsigned char)pos_code);
    if ((char)tolower((unsigned char)pos->x) == target)
    {
        return 0;
    }
    if ((char)tolower((unsigned char)pos->y) == target)
    {
        return 1;
    }
    if ((char)tolower((unsigned char)pos->z) == target)
    {
        return 2;
    }
    return -1;
}

static float peak_amp_for_axis(const patrol_peak_t *peak, int axis)
{
    if (!peak)
    {
        return 0.0f;
    }

    switch (axis)
    {
        case 0: return peak->amp_x;
        case 1: return peak->amp_y;
        case 2: return peak->amp_z;
        default: return 0.0f;
    }
}

static float axial_amp(const patrol_peak_t *peak, const sensor_position_t *pos)
{
    return peak_amp_for_axis(peak, axis_index_for_position(pos, 'a'));
}

static float radial_amp(const patrol_peak_t *peak, const sensor_position_t *pos)
{
    const float h = peak_amp_for_axis(peak, axis_index_for_position(pos, 'h'));
    const float v = peak_amp_for_axis(peak, axis_index_for_position(pos, 'v'));
    return fmaxf(h, v);
}

static bool is_axial_dominant(const patrol_peak_t *peak, const sensor_position_t *pos)
{
    if (!peak || !pos)
    {
        return false;
    }
    return (char)tolower((unsigned char)position_code_for_axis(pos, peak->dominant_axis)) == 'a';
}

static const char *position_name(char pos_code)
{
    switch ((char)tolower((unsigned char)pos_code))
    {
        case 'h': return "horizontal-radial";
        case 'v': return "vertical-radial";
        case 'a': return "axial";
        default: return "unknown";
    }
}

static const char *axis_name(uint8_t axis)
{
    switch (axis)
    {
        case 0: return "X";
        case 1: return "Y";
        case 2: return "Z";
        default: return "?";
    }
}

static inline bool near_float(float value, float target, float tolerance)
{
    return fabsf(value - target) <= tolerance;
}

typedef struct
{
    float freq_hz;
    float amplitude;
} band_peak_t;

typedef struct
{
    float freq_hz;
    float amp_x;
    float amp_y;
    float amp_z;
    uint8_t dominant_axis;
} band_candidate_peak_t;

typedef struct
{
    float min_hz;
    float max_hz;
    float distance_weight_hz;
} hint_search_stage_t;

static void compute_signal_stats(
    const float *data,
    uint32_t n,
    float *out_mean,
    float *out_rms,
    float *out_min,
    float *out_max)
{
    if (!data || n == 0U)
    {
        return;
    }

    float sum = 0.0f;
    float sum_sq = 0.0f;
    float min_v = data[0];
    float max_v = data[0];

    for (uint32_t i = 0; i < n; ++i)
    {
        const float v = data[i];
        sum += v;
        sum_sq += v * v;
        if (v < min_v)
        {
            min_v = v;
        }
        if (v > max_v)
        {
            max_v = v;
        }
    }

    if (out_mean)
    {
        *out_mean = sum / (float)n;
    }
    if (out_rms)
    {
        *out_rms = sqrtf(sum_sq / (float)n);
    }
    if (out_min)
    {
        *out_min = min_v;
    }
    if (out_max)
    {
        *out_max = max_v;
    }
}

static void remove_dc_inplace(float *data, uint32_t n)
{
    if (!data || n == 0U)
    {
        return;
    }

    float mean = 0.0f;
    for (uint32_t i = 0; i < n; ++i)
    {
        mean += data[i];
    }
    mean /= (float)n;

    for (uint32_t i = 0; i < n; ++i)
    {
        data[i] -= mean;
    }
}

static band_peak_t find_band_peak(
    const float *mag,
    uint32_t mag_len,
    float sample_rate,
    uint32_t fft_len,
    float min_hz,
    float max_hz)
{
    band_peak_t result = {0};
    if (!mag || mag_len == 0U || sample_rate <= 0.0f || fft_len == 0U || max_hz < min_hz)
    {
        return result;
    }

    const float bin_hz = sample_rate / (float)fft_len;
    uint32_t min_bin = (uint32_t)floorf(min_hz / bin_hz);
    uint32_t max_bin = (uint32_t)ceilf(max_hz / bin_hz);

    if (min_bin < 1U)
    {
        min_bin = 1U;
    }
    if (max_bin >= mag_len)
    {
        max_bin = mag_len - 1U;
    }
    if (min_bin > max_bin)
    {
        return result;
    }

    uint32_t best_bin = min_bin;
    float best_amp = mag[min_bin];
    for (uint32_t i = min_bin + 1U; i <= max_bin; ++i)
    {
        if (mag[i] > best_amp)
        {
            best_amp = mag[i];
            best_bin = i;
        }
    }

    result.freq_hz = (float)best_bin * bin_hz;
    result.amplitude = best_amp;
    return result;
}

static float parabolic_refine_peak_hz(
    const float *mag,
    uint32_t mag_len,
    uint32_t bin_idx,
    float bin_hz)
{
    if (!mag || mag_len < 3U || bin_idx == 0U || (bin_idx + 1U) >= mag_len || bin_hz <= 0.0f)
    {
        return (float)bin_idx * bin_hz;
    }

    const float y1 = mag[bin_idx - 1U];
    const float y2 = mag[bin_idx];
    const float y3 = mag[bin_idx + 1U];
    const float denom = (y1 - 2.0f * y2 + y3);
    if (fabsf(denom) < 1e-12f)
    {
        return (float)bin_idx * bin_hz;
    }

    float delta = 0.5f * (y1 - y3) / denom;
    if (delta > 0.5f)
    {
        delta = 0.5f;
    }
    else if (delta < -0.5f)
    {
        delta = -0.5f;
    }

    return ((float)bin_idx + delta) * bin_hz;
}

static bool find_band_candidate_peak(
    const float *x_data,
    const float *y_data,
    const float *z_data,
    uint32_t n,
    float sample_rate,
    float min_hz,
    float max_hz,
    float target_hz,
    float distance_weight_hz,
    band_candidate_peak_t *out_peak)
{
    if (!x_data || !y_data || !z_data || !out_peak || n < 4U || sample_rate <= 0.0f || max_hz < min_hz)
    {
        return false;
    }

    float *scratch = (float *)heap_caps_malloc(n * sizeof(float), MALLOC_CAP_SPIRAM);
    float *mag_x = (float *)heap_caps_malloc((n / 2U) * sizeof(float), MALLOC_CAP_SPIRAM);
    float *mag_y = (float *)heap_caps_malloc((n / 2U) * sizeof(float), MALLOC_CAP_SPIRAM);
    float *mag_z = (float *)heap_caps_malloc((n / 2U) * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!scratch || !mag_x || !mag_y || !mag_z)
    {
        if (scratch) heap_caps_free(scratch);
        if (mag_x) heap_caps_free(mag_x);
        if (mag_y) heap_caps_free(mag_y);
        if (mag_z) heap_caps_free(mag_z);
        return false;
    }

    memcpy(scratch, x_data, n * sizeof(float));
    remove_dc_inplace(scratch, n);
    if (algo_fft_calculate(scratch, mag_x, n) != ESP_OK)
    {
        heap_caps_free(scratch);
        heap_caps_free(mag_x);
        heap_caps_free(mag_y);
        heap_caps_free(mag_z);
        return false;
    }

    memcpy(scratch, y_data, n * sizeof(float));
    remove_dc_inplace(scratch, n);
    if (algo_fft_calculate(scratch, mag_y, n) != ESP_OK)
    {
        heap_caps_free(scratch);
        heap_caps_free(mag_x);
        heap_caps_free(mag_y);
        heap_caps_free(mag_z);
        return false;
    }

    memcpy(scratch, z_data, n * sizeof(float));
    remove_dc_inplace(scratch, n);
    if (algo_fft_calculate(scratch, mag_z, n) != ESP_OK)
    {
        heap_caps_free(scratch);
        heap_caps_free(mag_x);
        heap_caps_free(mag_y);
        heap_caps_free(mag_z);
        return false;
    }

    const float bin_hz = sample_rate / (float)n;
    uint32_t min_bin = (uint32_t)floorf(min_hz / bin_hz);
    uint32_t max_bin = (uint32_t)ceilf(max_hz / bin_hz);
    if (min_bin < 1U)
    {
        min_bin = 1U;
    }
    if (max_bin >= (n / 2U))
    {
        max_bin = (n / 2U) - 1U;
    }
    if (min_bin > max_bin)
    {
        heap_caps_free(scratch);
        heap_caps_free(mag_x);
        heap_caps_free(mag_y);
        heap_caps_free(mag_z);
        return false;
    }

    uint32_t best_bin = 0U;
    float best_score = 0.0f;
    for (uint32_t i = min_bin; i <= max_bin; ++i)
    {
        if (i == 0U || i + 1U >= (n / 2U))
        {
            continue;
        }

        float score = mag_x[i];
        if (mag_y[i] > score)
        {
            score = mag_y[i];
        }
        if (mag_z[i] > score)
        {
            score = mag_z[i];
        }

        float prev_score = mag_x[i - 1U];
        if (mag_y[i - 1U] > prev_score)
        {
            prev_score = mag_y[i - 1U];
        }
        if (mag_z[i - 1U] > prev_score)
        {
            prev_score = mag_z[i - 1U];
        }

        float next_score = mag_x[i + 1U];
        if (mag_y[i + 1U] > next_score)
        {
            next_score = mag_y[i + 1U];
        }
        if (mag_z[i + 1U] > next_score)
        {
            next_score = mag_z[i + 1U];
        }

        if (score < prev_score || score < next_score)
        {
            continue;
        }

        float weighted_score = score;
        if (target_hz > 0.0f && distance_weight_hz > 0.0f)
        {
            weighted_score /= (1.0f + fabsf(((float)i * bin_hz) - target_hz) / distance_weight_hz);
        }

        if (weighted_score > best_score)
        {
            best_score = weighted_score;
            best_bin = i;
        }
    }

    if (best_score <= 0.0f || best_bin == 0U)
    {
        heap_caps_free(scratch);
        heap_caps_free(mag_x);
        heap_caps_free(mag_y);
        heap_caps_free(mag_z);
        return false;
    }

    out_peak->freq_hz = (float)best_bin * bin_hz;
    out_peak->amp_x = mag_x[best_bin];
    out_peak->amp_y = mag_y[best_bin];
    out_peak->amp_z = mag_z[best_bin];
    out_peak->dominant_axis = 0;
    float dominant = out_peak->amp_x;
    if (out_peak->amp_y > dominant)
    {
        dominant = out_peak->amp_y;
        out_peak->dominant_axis = 1;
    }
    if (out_peak->amp_z > dominant)
    {
        out_peak->dominant_axis = 2;
    }

    const float *dominant_mag = mag_x;
    if (out_peak->dominant_axis == 1U)
    {
        dominant_mag = mag_y;
    }
    else if (out_peak->dominant_axis == 2U)
    {
        dominant_mag = mag_z;
    }
    out_peak->freq_hz = parabolic_refine_peak_hz(dominant_mag, n / 2U, best_bin, bin_hz);
    if (out_peak->freq_hz < min_hz)
    {
        out_peak->freq_hz = min_hz;
    }
    else if (out_peak->freq_hz > max_hz)
    {
        out_peak->freq_hz = max_hz;
    }

    heap_caps_free(scratch);
    heap_caps_free(mag_x);
    heap_caps_free(mag_y);
    heap_caps_free(mag_z);
    return true;
}

static void inject_hint_band_peak(
    patrol_fft_report_t *report,
    const float *x_data,
    const float *y_data,
    const float *z_data,
    uint32_t n,
    float sample_rate,
    int32_t rpm_hint)
{
    if (!report || !x_data || !y_data || !z_data || n < 4U || sample_rate <= 0.0f || rpm_hint <= 0)
    {
        return;
    }

    const float hint_hz = (float)rpm_hint / 60.0f;
    const float search_min = hint_hz * RPM_HINT_MIN_FACTOR;
    const float search_max = hint_hz * RPM_HINT_MAX_FACTOR;

    for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
    {
        if (peak_score(&report->peaks[i]) <= 0.0f)
        {
            continue;
        }
        if (report->peaks[i].freq_hz >= search_min && report->peaks[i].freq_hz <= search_max)
        {
            return;
        }
    }

    const hint_search_stage_t stages[] = {
        {.min_hz = fmaxf(hint_hz - 2.0f, 0.5f), .max_hz = hint_hz + 2.0f, .distance_weight_hz = 0.75f},
        {.min_hz = fmaxf(hint_hz * 0.90f, 0.5f), .max_hz = hint_hz * 1.10f, .distance_weight_hz = 1.5f},
        {.min_hz = search_min, .max_hz = search_max, .distance_weight_hz = 3.0f},
    };

    band_candidate_peak_t candidate = {0};
    bool found_candidate = false;
    int used_stage = -1;
    for (int i = 0; i < (int)(sizeof(stages) / sizeof(stages[0])); ++i)
    {
        if (find_band_candidate_peak(
                x_data,
                y_data,
                z_data,
                n,
                sample_rate,
                stages[i].min_hz,
                stages[i].max_hz,
                hint_hz,
                stages[i].distance_weight_hz,
                &candidate))
        {
            found_candidate = true;
            used_stage = i;
            break;
        }
    }
    if (!found_candidate)
    {
        return;
    }

    int replace_idx = 0;
    float weakest_score = peak_score(&report->peaks[0]);
    for (int i = 1; i < PATROL_MAX_PEAKS; ++i)
    {
        const float score = peak_score(&report->peaks[i]);
        if (score < weakest_score)
        {
            weakest_score = score;
            replace_idx = i;
        }
    }

    report->peaks[replace_idx].freq_hz = candidate.freq_hz;
    report->peaks[replace_idx].amp_x = candidate.amp_x;
    report->peaks[replace_idx].amp_y = candidate.amp_y;
    report->peaks[replace_idx].amp_z = candidate.amp_z;
    report->peaks[replace_idx].dominant_axis = candidate.dominant_axis;

    LOG_INFOF("Injected hint-band peak[%d]: %.2fHz, amp[X=%.4f,Y=%.4f,Z=%.4f], stage=%d",
              replace_idx,
              candidate.freq_hz,
              candidate.amp_x,
              candidate.amp_y,
              candidate.amp_z,
              used_stage);
}

static void log_patrol_source_debug(
    const float *x_data,
    const float *y_data,
    const float *z_data,
    uint32_t n,
    float sample_rate,
    int32_t rpm_hint)
{
    if (!x_data || !y_data || !z_data || n < 4U || sample_rate <= 0.0f || rpm_hint <= 0)
    {
        return;
    }

    float *scratch = (float *)heap_caps_malloc(n * sizeof(float), MALLOC_CAP_SPIRAM);
    float *mag = (float *)heap_caps_malloc((n / 2U) * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!scratch || !mag)
    {
        if (scratch) heap_caps_free(scratch);
        if (mag) heap_caps_free(mag);
        LOG_WARN("FFT debug skipped: insufficient memory");
        return;
    }

    const float *axes[3] = {x_data, y_data, z_data};
    const char axis_names[3] = {'X', 'Y', 'Z'};
    const float hint_hz = (float)rpm_hint / 60.0f;
    const float search_min = hint_hz * RPM_HINT_MIN_FACTOR;
    const float search_max = hint_hz * RPM_HINT_MAX_FACTOR;
    const float low_min = 0.5f;
    const float low_max = 20.0f;
    const float narrow_min = fmaxf(hint_hz - 2.0f, 0.5f);
    const float narrow_max = hint_hz + 2.0f;
    const float bin_hz = sample_rate / (float)n;
    const uint32_t hint_bin = (uint32_t)lroundf(hint_hz / bin_hz);

    LOG_INFOF("FFT source debug: sr=%.2fHz, n=%lu, hint=%ldrpm(%.2fHz), search=[%.2f, %.2f]Hz",
              sample_rate, (unsigned long)n, (long)rpm_hint, hint_hz, search_min, search_max);

    for (int axis = 0; axis < 3; ++axis)
    {
        float mean = 0.0f;
        float rms = 0.0f;
        float min_v = 0.0f;
        float max_v = 0.0f;
        compute_signal_stats(axes[axis], n, &mean, &rms, &min_v, &max_v);

        memcpy(scratch, axes[axis], n * sizeof(float));
        remove_dc_inplace(scratch, n);

        const esp_err_t err = algo_fft_calculate(scratch, mag, n);
        if (err != ESP_OK)
        {
            LOG_WARNF("FFT source debug axis %c skipped: %d", axis_names[axis], err);
            continue;
        }

        const band_peak_t low_peak = find_band_peak(mag, n / 2U, sample_rate, n, low_min, low_max);
        const band_peak_t hint_peak = find_band_peak(mag, n / 2U, sample_rate, n, search_min, search_max);
        const band_peak_t narrow_peak = find_band_peak(mag, n / 2U, sample_rate, n, narrow_min, narrow_max);
        const float hint_amp = (hint_bin < (n / 2U)) ? mag[hint_bin] : 0.0f;

        LOG_INFOF("FFT source axis %c: mean=%.4fg, rms=%.4fg, pp=%.4fg, low[%.1f-%.1fHz]=%.2fHz/%.4f, hint[%.1f-%.1fHz]=%.2fHz/%.4f, near1x[%.1f-%.1fHz]=%.2fHz/%.4f, bin@1x=%.4f",
                  axis_names[axis],
                  mean,
                  rms,
                  max_v - min_v,
                  low_min,
                  low_max,
                  low_peak.freq_hz,
                  low_peak.amplitude,
                  search_min,
                  search_max,
                  hint_peak.freq_hz,
                  hint_peak.amplitude,
                  narrow_min,
                  narrow_max,
                  narrow_peak.freq_hz,
                  narrow_peak.amplitude,
                  hint_amp);
    }

    heap_caps_free(scratch);
    heap_caps_free(mag);
}

static float candidate_rotational_score(
    const patrol_fft_report_t *report,
    const sensor_position_t *pos,
    int candidate_idx)
{
    const patrol_peak_t *candidate = &report->peaks[candidate_idx];
    float score = peak_score(candidate);
    if (score <= 0.0f)
    {
        return 0.0f;
    }

    if (is_axial_dominant(candidate, pos))
    {
        score *= AXIAL_CANDIDATE_BOOST;
    }

    for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
    {
        if (i == candidate_idx)
        {
            continue;
        }

        const patrol_peak_t *support = &report->peaks[i];
        const float support_score = peak_score(support);
        if (support_score <= 0.0f)
        {
            continue;
        }

        const float harmonic_ratio = support->freq_hz / candidate->freq_hz;
        float weight = 0.0f;
        if (near_float(harmonic_ratio, 2.0f, ORDER_MATCH_TOLERANCE))
        {
            weight = 0.60f;
        }
        else if (near_float(harmonic_ratio, 3.0f, ORDER_MATCH_TOLERANCE))
        {
            weight = 0.35f;
        }

        if (weight > 0.0f)
        {
            if (is_axial_dominant(support, pos))
            {
                weight *= AXIAL_HARMONIC_BOOST;
            }
            score += support_score * weight;
        }
    }

    return score;
}

static bool estimate_rotational_freq_hz(
    const patrol_fft_report_t *report,
    const sensor_position_t *pos,
    int32_t rpm_hint,
    float *out_freq_hz)
{
    if (!report || !out_freq_hz)
    {
        return false;
    }

    float hint_hz = 0.0f;
    float search_min = 0.0f;
    float search_max = PATROL_MAX_FREQ_HZ;
    if (rpm_hint > 0)
    {
        hint_hz = (float)rpm_hint / 60.0f;
        search_min = hint_hz * RPM_HINT_MIN_FACTOR;
        search_max = hint_hz * RPM_HINT_MAX_FACTOR;
    }

    int best_idx = -1;
    float best_score = 0.0f;

    for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
    {
        const patrol_peak_t *p = &report->peaks[i];
        if (peak_score(p) <= 0.0f)
        {
            continue;
        }

        if (p->freq_hz < search_min || p->freq_hz > search_max)
        {
            continue;
        }

        const float score = candidate_rotational_score(report, pos, i);
        if (score > best_score)
        {
            best_score = score;
            best_idx = i;
        }
    }

    if (best_idx < 0)
    {
        return false;
    }

    const patrol_peak_t *best = &report->peaks[best_idx];
    float refined_sum = best->freq_hz * candidate_rotational_score(report, pos, best_idx);
    float refined_weight = candidate_rotational_score(report, pos, best_idx);

    for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
    {
        if (i == best_idx)
        {
            continue;
        }

        const patrol_peak_t *support = &report->peaks[i];
        const float support_score = peak_score(support);
        if (support_score <= 0.0f)
        {
            continue;
        }

        const float ratio = support->freq_hz / best->freq_hz;
        float weight = 0.0f;
        float implied_freq = 0.0f;
        if (near_float(ratio, 2.0f, ORDER_MATCH_TOLERANCE))
        {
            weight = 0.60f * support_score;
            implied_freq = support->freq_hz / 2.0f;
        }
        else if (near_float(ratio, 3.0f, ORDER_MATCH_TOLERANCE))
        {
            weight = 0.35f * support_score;
            implied_freq = support->freq_hz / 3.0f;
        }

        if (weight > 0.0f)
        {
            refined_sum += implied_freq * weight;
            refined_weight += weight;
        }
    }

    *out_freq_hz = (refined_weight > 0.0f) ? (refined_sum / refined_weight) : best->freq_hz;
    return true;
}

static int find_best_peak_near_order(
    const patrol_fft_report_t *report,
    const sensor_position_t *pos,
    float rotation_hz,
    int order,
    float tolerance,
    bool prefer_axial)
{
    if (!report || rotation_hz <= 0.0f || order <= 0)
    {
        return -1;
    }

    const float target_hz = rotation_hz * (float)order;
    const float min_hz = target_hz * (1.0f - tolerance);
    const float max_hz = target_hz * (1.0f + tolerance);

    int best_idx = -1;
    float best_score = 0.0f;
    for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
    {
        const patrol_peak_t *peak = &report->peaks[i];
        const float base_score = peak_score(peak);
        if (base_score <= 0.0f || peak->freq_hz < min_hz || peak->freq_hz > max_hz)
        {
            continue;
        }

        float score = base_score;
        if (prefer_axial && is_axial_dominant(peak, pos))
        {
            score *= AXIAL_CANDIDATE_BOOST;
        }

        const float target_error = fabsf(peak->freq_hz - target_hz);
        score /= (1.0f + target_error);

        if (score > best_score)
        {
            best_score = score;
            best_idx = i;
        }
    }

    return best_idx;
}

static void analyze_orders(
    const patrol_fft_report_t *report,
    const sensor_position_t *pos,
    int32_t rpm_hint,
    order_analysis_t *analysis)
{
    if (!analysis)
    {
        return;
    }

    memset(analysis, 0, sizeof(*analysis));
    analysis->peak_1x_idx = -1;
    analysis->peak_2x_idx = -1;
    analysis->peak_3x_idx = -1;

    float estimated_hz = 0.0f;
    if (!estimate_rotational_freq_hz(report, pos, rpm_hint, &estimated_hz))
    {
        return;
    }

    analysis->estimated_hz = estimated_hz;
    analysis->peak_1x_idx = find_best_peak_near_order(report, pos, estimated_hz, 1, ORDER_MATCH_TOLERANCE, true);
    analysis->peak_2x_idx = find_best_peak_near_order(report, pos, estimated_hz, 2, ORDER_MATCH_TOLERANCE, false);
    analysis->peak_3x_idx = find_best_peak_near_order(report, pos, estimated_hz, 3, ORDER_MATCH_TOLERANCE, false);

    float refined_sum = 0.0f;
    float refined_weight = 0.0f;
    const int peak_indices[3] = {analysis->peak_1x_idx, analysis->peak_2x_idx, analysis->peak_3x_idx};
    const float divisors[3] = {1.0f, 2.0f, 3.0f};
    const float weights[3] = {1.0f, 0.60f, 0.35f};

    for (int i = 0; i < 3; ++i)
    {
        const int peak_idx = peak_indices[i];
        if (peak_idx < 0)
        {
            continue;
        }

        const patrol_peak_t *peak = &report->peaks[peak_idx];
        const float peak_weight = peak_score(peak) * weights[i];
        if (peak_weight <= 0.0f)
        {
            continue;
        }

        refined_sum += (peak->freq_hz / divisors[i]) * peak_weight;
        refined_weight += peak_weight;
    }

    analysis->refined_hz = (refined_weight > 0.0f) ? (refined_sum / refined_weight) : estimated_hz;

    analysis->peak_1x_idx = find_best_peak_near_order(report, pos, analysis->refined_hz, 1, REFINED_ORDER_TOLERANCE, true);
    analysis->peak_2x_idx = find_best_peak_near_order(report, pos, analysis->refined_hz, 2, REFINED_ORDER_TOLERANCE, false);
    analysis->peak_3x_idx = find_best_peak_near_order(report, pos, analysis->refined_hz, 3, REFINED_ORDER_TOLERANCE, false);
}

static void classify_patrol_fault(patrol_fft_report_t *report)
{
    if (!report)
    {
        return;
    }

    report->fault_code = 0;
    report->fault_desc[0] = '\0';
    report->confidence = 0.0f;

    if (g_user_config.rpm <= 0)
    {
        return;
    }

    order_analysis_t analysis;
    analyze_orders(report, &g_user_config.pos, g_user_config.rpm, &analysis);
    if (analysis.refined_hz <= 0.0f)
    {
        return;
    }

    bool has_grid_100hz = false;
    bool found_integer[16] = {0};
    bool found_fractional = false;
    float one_x_radial_amp = 0.0f;
    float one_x_axial_amp = 0.0f;

    for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
    {
        const patrol_peak_t *p = &report->peaks[i];
        const float score = peak_score(p);
        if (score <= 0.0f)
        {
            continue;
        }

        if (near_float(p->freq_hz, 100.0f, 2.0f))
        {
            has_grid_100hz = true;
        }

        const float ratio = p->freq_hz / analysis.refined_hz;
        if (ratio < 0.5f || ratio > 15.0f)
        {
            continue;
        }

        const float integer_ratio = roundf(ratio);
        const float ratio_err = fabsf(ratio - integer_ratio);
        if (ratio_err <= ORDER_MATCH_TOLERANCE && integer_ratio >= 1.0f && integer_ratio <= 15.0f)
        {
            found_integer[(int)integer_ratio] = true;
        }
        else if (ratio >= 1.0f)
        {
            found_fractional = true;
        }
    }

    bool has_1x = (analysis.peak_1x_idx >= 0);
    bool has_2x = (analysis.peak_2x_idx >= 0);
    bool axial_1x = false;
    bool axial_2x = false;

    if (analysis.peak_1x_idx >= 0)
    {
        const patrol_peak_t *peak_1x = &report->peaks[analysis.peak_1x_idx];
        one_x_radial_amp = radial_amp(peak_1x, &g_user_config.pos);
        one_x_axial_amp = axial_amp(peak_1x, &g_user_config.pos);
        axial_1x = is_axial_dominant(peak_1x, &g_user_config.pos) || (one_x_axial_amp >= one_x_radial_amp);
    }
    if (analysis.peak_2x_idx >= 0)
    {
        const patrol_peak_t *peak_2x = &report->peaks[analysis.peak_2x_idx];
        const float two_x_radial_amp = radial_amp(peak_2x, &g_user_config.pos);
        const float two_x_axial_amp = axial_amp(peak_2x, &g_user_config.pos);
        axial_2x = is_axial_dominant(peak_2x, &g_user_config.pos) || (two_x_axial_amp >= two_x_radial_amp);
    }

    int integer_count = 0;
    for (int i = 1; i <= 15; ++i)
    {
        if (found_integer[i])
        {
            integer_count++;
        }
    }

    const patrol_peak_t *peak_1x = (analysis.peak_1x_idx >= 0) ? &report->peaks[analysis.peak_1x_idx] : NULL;
    const patrol_peak_t *peak_2x = (analysis.peak_2x_idx >= 0) ? &report->peaks[analysis.peak_2x_idx] : NULL;
    const patrol_peak_t *peak_3x = (analysis.peak_3x_idx >= 0) ? &report->peaks[analysis.peak_3x_idx] : NULL;

    LOG_INFOF("Patrol 1X lock: hint=%ldrpm(%.2fHz), lock=%.2fHz/%.1frpm, idx[1x=%d,2x=%d,3x=%d]",
              (long)g_user_config.rpm,
              (float)g_user_config.rpm / 60.0f,
              analysis.refined_hz,
              analysis.refined_hz * 60.0f,
              analysis.peak_1x_idx,
              analysis.peak_2x_idx,
              analysis.peak_3x_idx);

    if (peak_1x)
    {
        const char pos_code = position_code_for_axis(&g_user_config.pos, peak_1x->dominant_axis);
        LOG_INFOF("Patrol order 1X: %.2fHz, dom=%s(%s), amp[X=%.4f,Y=%.4f,Z=%.4f], radial=%.4f, axial=%.4f",
                  peak_1x->freq_hz,
                  axis_name(peak_1x->dominant_axis),
                  position_name(pos_code),
                  peak_1x->amp_x,
                  peak_1x->amp_y,
                  peak_1x->amp_z,
                  radial_amp(peak_1x, &g_user_config.pos),
                  axial_amp(peak_1x, &g_user_config.pos));
    }
    else
    {
        LOG_INFO("Patrol order 1X: not found");
    }

    if (peak_2x)
    {
        const char pos_code = position_code_for_axis(&g_user_config.pos, peak_2x->dominant_axis);
        LOG_INFOF("Patrol order 2X: %.2fHz, dom=%s(%s), amp[X=%.4f,Y=%.4f,Z=%.4f], radial=%.4f, axial=%.4f",
                  peak_2x->freq_hz,
                  axis_name(peak_2x->dominant_axis),
                  position_name(pos_code),
                  peak_2x->amp_x,
                  peak_2x->amp_y,
                  peak_2x->amp_z,
                  radial_amp(peak_2x, &g_user_config.pos),
                  axial_amp(peak_2x, &g_user_config.pos));
    }
    else
    {
        LOG_INFO("Patrol order 2X: not found");
    }

    if (peak_3x)
    {
        const char pos_code = position_code_for_axis(&g_user_config.pos, peak_3x->dominant_axis);
        LOG_INFOF("Patrol order 3X: %.2fHz, dom=%s(%s), amp[X=%.4f,Y=%.4f,Z=%.4f], radial=%.4f, axial=%.4f",
                  peak_3x->freq_hz,
                  axis_name(peak_3x->dominant_axis),
                  position_name(pos_code),
                  peak_3x->amp_x,
                  peak_3x->amp_y,
                  peak_3x->amp_z,
                  radial_amp(peak_3x, &g_user_config.pos),
                  axial_amp(peak_3x, &g_user_config.pos));
    }
    else
    {
        LOG_INFO("Patrol order 3X: not found");
    }

    if (has_1x && !has_2x && one_x_radial_amp > one_x_axial_amp * 2.0f)
    {
        report->fault_code = 1;
        strncpy(report->fault_desc, "Unbalance: clean 1x with radial dominating axial", sizeof(report->fault_desc) - 1);
        report->fault_desc[sizeof(report->fault_desc) - 1] = '\0';
        report->confidence = 0.88f;
        return;
    }

    if (has_2x && (axial_1x || axial_2x))
    {
        report->fault_code = 2;
        strncpy(report->fault_desc, "Misalignment: strong 2x and axial component present", sizeof(report->fault_desc) - 1);
        report->fault_desc[sizeof(report->fault_desc) - 1] = '\0';
        report->confidence = 0.83f;
        return;
    }

    if (integer_count >= 4)
    {
        report->fault_code = 3;
        strncpy(report->fault_desc, "Looseness: multi-harmonic integer spacing in peaks", sizeof(report->fault_desc) - 1);
        report->fault_desc[sizeof(report->fault_desc) - 1] = '\0';
        report->confidence = 0.79f;
        return;
    }

    if (found_fractional)
    {
        report->fault_code = 4;
        strncpy(report->fault_desc, "Bearing defect: non-integer harmonic components detected", sizeof(report->fault_desc) - 1);
        report->fault_desc[sizeof(report->fault_desc) - 1] = '\0';
        report->confidence = 0.72f;
        return;
    }

    if (has_grid_100hz)
    {
        report->fault_code = 5;
        strncpy(report->fault_desc, "Electrical fault: 100Hz grid-synchronous excitation present", sizeof(report->fault_desc) - 1);
        report->fault_desc[sizeof(report->fault_desc) - 1] = '\0';
        report->confidence = 0.66f;
        return;
    }

    report->fault_code = 0;
    strncpy(report->fault_desc, "No patrol fault signature detected", sizeof(report->fault_desc) - 1);
    report->fault_desc[sizeof(report->fault_desc) - 1] = '\0';
    report->confidence = 0.15f;
}

static void send_patrol_fft_report(const vib_job_t *job)
{
    if (!job)
    {
        return;
    }

    s_patrol_fft_report.task_id = (int32_t)job->task_id;
    s_patrol_fft_report.timestamp = (job->timestamp > 0) ? job->timestamp : (int32_t)time(NULL);

    if (job->task_mode == TASK_MODE_PATROLING)
    {
        classify_patrol_fault(&s_patrol_fft_report);
        LOG_INFOF("Patrol fault: code=%d, conf=%.2f, desc=%s", s_patrol_fft_report.fault_code, s_patrol_fft_report.confidence, s_patrol_fft_report.fault_desc);
    }
    else
    {
        s_patrol_fft_report.fault_code = 0;
        strncpy(s_patrol_fft_report.fault_desc, "Not a patrol task - fault analysis skipped", sizeof(s_patrol_fft_report.fault_desc) - 1);
        s_patrol_fft_report.fault_desc[sizeof(s_patrol_fft_report.fault_desc) - 1] = '\0';
        s_patrol_fft_report.confidence = 0.0f;
    }

    bool logged_lowfreq = false;
    for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
    {
        const patrol_peak_t *peak = &s_patrol_fft_report.peaks[i];
        if (peak_score(peak) <= 0.0f)
        {
            continue;
        }

        const char pos_code = position_code_for_axis(&g_user_config.pos, peak->dominant_axis);
        LOG_INFOF("Patrol spectral peak[%d]: %.2fHz, dom=%c(%s), amp[X=%.4f,Y=%.4f,Z=%.4f]",
                  i,
                  peak->freq_hz,
                  "XYZ"[peak->dominant_axis],
                  position_name(pos_code),
                  peak->amp_x,
                  peak->amp_y,
                  peak->amp_z);

        if (job->task_mode == TASK_MODE_PATROLING && peak->freq_hz <= PATROL_LOG_LOW_FREQ_MAX_HZ)
        {
            LOG_INFOF("Patrol lowfreq peak[%d]: %.2fHz, dom=%c(%s), amp[X=%.4f,Y=%.4f,Z=%.4f]",
                      i,
                      peak->freq_hz,
                      "XYZ"[peak->dominant_axis],
                      position_name(pos_code),
                      peak->amp_x,
                      peak->amp_y,
                      peak->amp_z);
            logged_lowfreq = true;
        }
    }

    if (job->task_mode == TASK_MODE_PATROLING && !logged_lowfreq)
    {
        LOG_INFOF("Patrol lowfreq peaks: none below %.1fHz in representative peak set", PATROL_LOG_LOW_FREQ_MAX_HZ);
    }

    if (g_msg_dispatcher_queue == NULL)
    {
        LOG_ERROR("Message dispatcher queue not initialized");
        return;
    }

    // binary_msg_t msg = {
    //     .data = (uint8_t *)&s_patrol_fft_report,
    //     .len = sizeof(s_patrol_fft_report),
    // };
    // strncpy(msg.topic, "device/vibration/fft", sizeof(msg.topic) - 1);
    // msg.topic[sizeof(msg.topic) - 1] = '\0';
    // xQueueSend(g_msg_dispatcher_queue, &msg, 0);
}

static void log_diagnosis_fft_report(const vib_job_t *job)
{
    if (!job)
    {
        return;
    }

    diag_fft_summary_t summary = {
        .timestamp = job->timestamp,
        .length = job->length,
        .sample_rate = job->sample_rate,
    };
    memcpy(summary.peaks, s_patrol_fft_report.peaks, sizeof(summary.peaks));
    if (diag_fusion_submit_fft(&summary) != ESP_OK)
    {
        LOG_WARN("Failed to submit diagnosis FFT summary");
    }

    for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
    {
        const patrol_peak_t *peak = &s_patrol_fft_report.peaks[i];
        if (peak_score(peak) <= 0.0f)
        {
            continue;
        }

        LOG_INFOF("Diagnosis spectral peak[%d]: %.2fHz, dom=%c, amp[X=%.4f,Y=%.4f,Z=%.4f]",
                  i,
                  peak->freq_hz,
                  "XYZ"[peak->dominant_axis],
                  peak->amp_x,
                  peak->amp_y,
                  peak->amp_z);
    }
}

static void fft_task_entry(void *arg)
{
    vib_job_t job;
    while (1)
    {
        if (xQueueReceive(g_fft_job_queue, &job, portMAX_DELAY))
        {
            s_fft_task_busy = true;
            LOG_INFOF("Received job for FFT analysis. Mode: %d, Len: %lu, SR: %.1f",
                      job.task_mode, job.length, job.sample_rate);
            const uint32_t n = job.length;
            if (n < 2 || n > MAX_DAQ_SAMPLES || ((n & (n - 1U)) != 0U))
            {
                LOG_ERRORF("Invalid FFT length: %lu", n);
                s_fft_task_busy = false;
                continue; 
            }

            if (job.raw_data == NULL)
            {
                LOG_ERROR("raw_data is NULL");
                s_fft_task_busy = false;
                continue;
            }

            const float *x_ptr = job.raw_data;
            const float *y_ptr = job.raw_data + MAX_DAQ_SAMPLES;
            const float *z_ptr = job.raw_data + MAX_DAQ_SAMPLES * 2;
            const float *fft_x_ptr = x_ptr;
            const float *fft_y_ptr = y_ptr;
            const float *fft_z_ptr = z_ptr;
            float *demean_buf = NULL;

            LOG_INFO("Performing 3-axis FFT analysis...");
            if (job.task_mode == TASK_MODE_PATROLING)
            {
                log_patrol_source_debug(x_ptr, y_ptr, z_ptr, n, job.sample_rate, g_user_config.rpm);
                demean_buf = (float *)heap_caps_malloc(3U * n * sizeof(float), MALLOC_CAP_SPIRAM);
                if (demean_buf)
                {
                    fft_x_ptr = demean_buf;
                    fft_y_ptr = demean_buf + n;
                    fft_z_ptr = demean_buf + 2U * n;

                    memcpy((void *)fft_x_ptr, x_ptr, n * sizeof(float));
                    memcpy((void *)fft_y_ptr, y_ptr, n * sizeof(float));
                    memcpy((void *)fft_z_ptr, z_ptr, n * sizeof(float));
                    remove_dc_inplace((float *)fft_x_ptr, n);
                    remove_dc_inplace((float *)fft_y_ptr, n);
                    remove_dc_inplace((float *)fft_z_ptr, n);
                }
                else
                {
                    LOG_WARN("Patrol FFT using raw data: failed to allocate de-mean buffer");
                }
            }

            esp_err_t err = algo_fft_calculate_peaks(
                fft_x_ptr,
                fft_y_ptr,
                fft_z_ptr,
                n,
                job.sample_rate,
                &s_patrol_fft_report);
            if (err != ESP_OK)
            {
                if (demean_buf)
                {
                    heap_caps_free(demean_buf);
                }
                LOG_ERRORF("FFT peak analysis failed: %d", err);
                s_fft_task_busy = false;
                continue;
            }

            if (job.task_mode == TASK_MODE_PATROLING)
            {
                inject_hint_band_peak(&s_patrol_fft_report, x_ptr, y_ptr, z_ptr, n, job.sample_rate, g_user_config.rpm);
            }

            if (demean_buf)
            {
                heap_caps_free(demean_buf);
            }

            if (job.task_mode == TASK_MODE_PATROLING)
            {
                send_patrol_fft_report(&job);
            }
            else
            {
                log_diagnosis_fft_report(&job);
            }
            LOG_INFO("FFT analysis complete.");
            s_fft_task_busy = false;
        }
    }
}

bool task_fft_is_idle(void)
{
    if (g_fft_job_queue == NULL)
    {
        return true;
    }

    return (!s_fft_task_busy) && (uxQueueMessagesWaiting(g_fft_job_queue) == 0);
}

esp_err_t start_fft_task(void)
{
    if (g_fft_job_queue == NULL)
    {
        g_fft_job_queue = xQueueCreateWithCaps(FFT_QUEUE_LEN, sizeof(vib_job_t), TASK_MEM_CAPS);
        if (g_fft_job_queue == NULL)
        {
            LOG_ERROR("Failed to create FFT queue");
            return ESP_ERR_NO_MEM;
        }
    }

    if (xTaskCreateWithCaps(fft_task_entry, "fft_task", 4096 * 2, NULL, 3, NULL, TASK_MEM_CAPS) != pdPASS)
    {
        LOG_ERROR("Failed to create FFT task");
    }
    else
    {
        LOG_INFO("FFT analysis task started");
        return ESP_OK;
    }
    return ESP_ERR_INVALID_STATE;
}
