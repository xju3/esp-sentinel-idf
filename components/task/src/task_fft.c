#include "task_fft.h"
#include "algo_fft.h"
#include "logger.h"
#include "data_dispatcher.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAX_DAQ_SAMPLES 8192
#define FFT_QUEUE_LEN   5
#define TASK_MEM_CAPS   (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

QueueHandle_t g_fft_job_queue = NULL;
static patrol_fft_report_t s_patrol_fft_report;

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

static const char *position_name(char pos_code)
{
    switch (pos_code)
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

    const float rpm_hz = (float)g_user_config.rpm / 60.0f;
    if (rpm_hz <= 0.0f)
    {
        return;
    }

    bool has_1x = false;
    bool has_2x = false;
    bool has_3x = false;
    bool has_grid_100hz = false;
    bool axial_1x = false;
    bool axial_2x = false;

    bool found_integer[16] = {0};
    bool found_fractional = false;
    float one_x_radial_amp = 0.0f;
    float one_x_axial_amp = 0.0f;

    for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
    {
        const patrol_peak_t *p = &report->peaks[i];
        float score = peak_score(p);
        if (score <= 0.0f)
        {
            continue;
        }

        if (near_float(p->freq_hz, 100.0f, 2.0f))
        {
            has_grid_100hz = true;
        }

        float ratio = p->freq_hz / rpm_hz;
        if (ratio < 0.5f || ratio > 15.0f)
        {
            continue;
        }

        float integer_ratio = roundf(ratio);
        float ratio_err = fabsf(ratio - integer_ratio);
        bool is_int = (ratio_err <= 0.12f);

        if (is_int && integer_ratio >= 1.0f && integer_ratio <= 15.0f)
        {
            found_integer[(int)integer_ratio] = true;
            if (integer_ratio == 1.0f)
            {
                has_1x = true;
                float radial = fmaxf(p->amp_x, p->amp_y);
                one_x_radial_amp = fmaxf(one_x_radial_amp, radial);
                one_x_axial_amp = fmaxf(one_x_axial_amp, p->amp_z);
                if (p->dominant_axis == 2)
                {
                    axial_1x = true;
                }
            }
            else if (integer_ratio == 2.0f)
            {
                has_2x = true;
                if (p->dominant_axis == 2)
                {
                    axial_2x = true;
                }
            }
            else if (integer_ratio == 3.0f)
            {
                has_3x = true;
            }
        }
        else
        {
            if (ratio >= 1.0f)
            {
                found_fractional = true;
            }
        }
    }

    int integer_count = 0;
    for (int i = 1; i <= 15; ++i)
    {
        if (found_integer[i])
        {
            integer_count++;
        }
    }

    // 1. 不平衡 (Unbalance)
    if (has_1x && !has_2x && one_x_radial_amp > one_x_axial_amp * 2.0f)
    {
        report->fault_code = 1;
        strncpy(report->fault_desc, "Unbalance: clean 1x with radial dominating axial", sizeof(report->fault_desc) - 1);
        report->fault_desc[sizeof(report->fault_desc) - 1] = '\0';
        report->confidence = 0.88f;
        return;
    }

    // 2. 不对中 (Misalignment)
    if (has_2x && (axial_1x || axial_2x))
    {
        report->fault_code = 2;
        strncpy(report->fault_desc, "Misalignment: strong 2x and axial component present", sizeof(report->fault_desc) - 1);
        report->fault_desc[sizeof(report->fault_desc) - 1] = '\0';
        report->confidence = 0.83f;
        return;
    }

    // 3. 机械松动 (Mechanical Looseness)
    if (integer_count >= 4)
    {
        report->fault_code = 3;
        strncpy(report->fault_desc, "Looseness: multi-harmonic integer spacing in peaks", sizeof(report->fault_desc) - 1);
        report->fault_desc[sizeof(report->fault_desc) - 1] = '\0';
        report->confidence = 0.79f;
        return;
    }

    // 4. 轴承中后期故障 (Bearing Defects - Late Stage)
    if (found_fractional)
    {
        report->fault_code = 4;
        strncpy(report->fault_desc, "Bearing defect: non-integer harmonic components detected", sizeof(report->fault_desc) - 1);
        report->fault_desc[sizeof(report->fault_desc) - 1] = '\0';
        report->confidence = 0.72f;
        return;
    }

    // 5. 电气故障 (Electrical Faults)
    if (has_grid_100hz)
    {
        report->fault_code = 5;
        strncpy(report->fault_desc, "Electrical fault: 100Hz grid-synchronous excitation present", sizeof(report->fault_desc) - 1);
        report->fault_desc[sizeof(report->fault_desc) - 1] = '\0';
        report->confidence = 0.66f;
        return;
    }

    // 未识别
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

    binary_msg_t msg = {
        .data = (uint8_t *)&s_patrol_fft_report,
        .len = sizeof(s_patrol_fft_report),
    };
    strncpy(msg.topic, "device/vibration/fft", sizeof(msg.topic) - 1);
    msg.topic[sizeof(msg.topic) - 1] = '\0';
    xQueueSend(g_msg_dispatcher_queue, &msg, 0);
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
