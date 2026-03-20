#include "task_fft.h"
#include "algo_fft.h"
#include "logger.h"
#include "data_dispatcher.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"

#define MAX_DAQ_SAMPLES 8192
#define FFT_QUEUE_LEN   5
#define PATROL_MAX_PEAKS 5
#define PATROL_MAX_FREQ_HZ 1000.0f
#define TASK_MEM_CAPS   (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

QueueHandle_t g_fft_job_queue = NULL;
EXT_RAM_BSS_ATTR static float s_fft_mag_x[MAX_DAQ_SAMPLES / 2];
EXT_RAM_BSS_ATTR static float s_fft_mag_y[MAX_DAQ_SAMPLES / 2];
EXT_RAM_BSS_ATTR static float s_fft_mag_z[MAX_DAQ_SAMPLES / 2];
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

static uint8_t dominant_axis_from_mag(float x, float y, float z)
{
    if (y >= x && y >= z)
    {
        return 1;
    }
    if (z >= x && z >= y)
    {
        return 2;
    }
    return 0;
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

static void try_insert_peak(
    patrol_peak_t *peaks,
    const patrol_peak_t *candidate)
{
    if (!peaks || !candidate)
    {
        return;
    }

    const float candidate_score = peak_score(candidate);
    if (candidate_score <= 0.0f)
    {
        return;
    }

    for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
    {
        if (candidate_score <= peak_score(&peaks[i]))
        {
            continue;
        }

        for (int j = PATROL_MAX_PEAKS - 1; j > i; --j)
        {
            peaks[j] = peaks[j - 1];
        }
        peaks[i] = *candidate;
        return;
    }
}

static void extract_patrol_peaks(
    patrol_fft_report_t *report,
    float sample_rate,
    uint32_t fft_len)
{
    if (!report || fft_len < 4 || sample_rate <= 0.0f)
    {
        return;
    }

    memset(report->peaks, 0, sizeof(report->peaks));

    const uint32_t half = fft_len >> 1;
    const float bin_hz = sample_rate / (float)fft_len;
    uint32_t max_bin = (uint32_t)(PATROL_MAX_FREQ_HZ / bin_hz);
    if (max_bin >= (half - 1U))
    {
        max_bin = half - 2U;
    }

    for (uint32_t i = 1; i <= max_bin; ++i)
    {
        const float prev_x = s_fft_mag_x[i - 1];
        const float prev_y = s_fft_mag_y[i - 1];
        const float prev_z = s_fft_mag_z[i - 1];
        const float cur_x = s_fft_mag_x[i];
        const float cur_y = s_fft_mag_y[i];
        const float cur_z = s_fft_mag_z[i];
        const float next_x = s_fft_mag_x[i + 1];
        const float next_y = s_fft_mag_y[i + 1];
        const float next_z = s_fft_mag_z[i + 1];

        float prev_score = prev_x;
        if (prev_y > prev_score) prev_score = prev_y;
        if (prev_z > prev_score) prev_score = prev_z;

        float cur_score = cur_x;
        if (cur_y > cur_score) cur_score = cur_y;
        if (cur_z > cur_score) cur_score = cur_z;

        float next_score = next_x;
        if (next_y > next_score) next_score = next_y;
        if (next_z > next_score) next_score = next_z;

        if (cur_score < prev_score || cur_score < next_score)
        {
            continue;
        }

        patrol_peak_t candidate = {
            .freq_hz = i * bin_hz,
            .amp_x = cur_x,
            .amp_y = cur_y,
            .amp_z = cur_z,
            .dominant_axis = dominant_axis_from_mag(cur_x, cur_y, cur_z),
        };
        try_insert_peak(report->peaks, &candidate);
    }
}

static void send_patrol_fft_report(const vib_job_t *job)
{
    if (!job)
    {
        return;
    }

    s_patrol_fft_report.task_id = (int32_t)job->task_id;
    s_patrol_fft_report.timestamp = (int32_t)time(NULL);
    s_patrol_fft_report.sample_rate = job->sample_rate;
    s_patrol_fft_report.pos = g_user_config.pos;

    extract_patrol_peaks(&s_patrol_fft_report, job->sample_rate, job->length);

    for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
    {
        const patrol_peak_t *peak = &s_patrol_fft_report.peaks[i];
        if (peak_score(peak) <= 0.0f)
        {
            continue;
        }

        const char pos_code = position_code_for_axis(&s_patrol_fft_report.pos, peak->dominant_axis);
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

            esp_err_t err = algo_fft_calculate(x_ptr, s_fft_mag_x, n);
            if (err != ESP_OK)
            {
                LOG_ERRORF("FFT failed on X axis: %d", err);
                continue;
            }
            
            err = algo_fft_calculate(y_ptr, s_fft_mag_y, n);
            if (err != ESP_OK)
            {
                LOG_ERRORF("FFT failed on Y axis: %d", err);
                continue;
           }
            
            err = algo_fft_calculate(z_ptr, s_fft_mag_z, n);
            if (err != ESP_OK)
            {
                LOG_ERRORF("FFT failed on Z axis: %d", err);
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
