#include "task_fft.h"
#include "algo_fft.h"
#include "logger.h"
#include "config_manager.h"
#include "data_dispatcher.h"
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

QueueHandle_t g_fft_job_queue = NULL;
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

static inline bool near_float(float value, float target, float tolerance)
{
    return fabsf(value - target) <= tolerance;
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

    if (best_idx < 0 && rpm_hint > 0)
    {
        float nearest_delta = 1e9f;
        for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
        {
            const patrol_peak_t *p = &report->peaks[i];
            const float score = peak_score(p);
            if (score <= 0.0f)
            {
                continue;
            }

            const float delta = fabsf(p->freq_hz - hint_hz);
            if (delta < nearest_delta)
            {
                nearest_delta = delta;
                best_idx = i;
            }
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

    LOG_INFOF("Patrol RPM estimate: hint=%ld, est=%.1f rpm, 1x=%d, 2x=%d, 3x=%d",
              (long)g_user_config.rpm,
              analysis.refined_hz * 60.0f,
              analysis.peak_1x_idx,
              analysis.peak_2x_idx,
              analysis.peak_3x_idx);

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

    for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
    {
        const patrol_peak_t *peak = &s_patrol_fft_report.peaks[i];
        if (peak_score(peak) <= 0.0f)
        {
            continue;
        }

        const char pos_code = position_code_for_axis(&g_user_config.pos, peak->dominant_axis);
        LOG_INFOF("Patrol peak[%d]: %.2fHz, dom=%c(%s), amp[X=%.4f,Y=%.4f,Z=%.4f]",
                  i,
                  peak->freq_hz,
                  "XYZ"[peak->dominant_axis],
                  position_name(pos_code),
                  peak->amp_x,
                  peak->amp_y,
                  peak->amp_z);
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

static void fft_task_entry(void *arg)
{
    vib_job_t job;
    while (1)
    {
        if (xQueueReceive(g_fft_job_queue, &job, portMAX_DELAY))
        {
            LOG_INFOF("Received job for FFT analysis. Mode: %d, Len: %lu, SR: %.1f",
                      job.task_mode, job.length, job.sample_rate);
            const uint32_t n = job.length;
            if (n < 2 || n > MAX_DAQ_SAMPLES || ((n & (n - 1U)) != 0U))
            {
                LOG_ERRORF("Invalid FFT length: %lu", n);
                continue; 
            }

            if (job.raw_data == NULL)
            {
                LOG_ERROR("raw_data is NULL");
                continue;
            }

            const float *x_ptr = job.raw_data;
            const float *y_ptr = job.raw_data + MAX_DAQ_SAMPLES;
            const float *z_ptr = job.raw_data + MAX_DAQ_SAMPLES * 2;

            LOG_INFO("Performing 3-axis FFT analysis...");

            esp_err_t err = algo_fft_calculate_peaks(
                x_ptr,
                y_ptr,
                z_ptr,
                n,
                job.sample_rate,
                &s_patrol_fft_report);
            if (err != ESP_OK)
            {
                LOG_ERRORF("FFT peak analysis failed: %d", err);
                continue;
            }

            send_patrol_fft_report(&job);
            LOG_INFO("FFT analysis complete.");
        }
    }
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
