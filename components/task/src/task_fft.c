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
#define TASK_MEM_CAPS   (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

QueueHandle_t g_fft_job_queue = NULL;
EXT_RAM_BSS_ATTR static float s_fft_mag_x[MAX_DAQ_SAMPLES / 2];
EXT_RAM_BSS_ATTR static float s_fft_mag_y[MAX_DAQ_SAMPLES / 2];
EXT_RAM_BSS_ATTR static float s_fft_mag_z[MAX_DAQ_SAMPLES / 2];

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

            const uint32_t half = n >> 1;
            uint32_t peak_idx_x = 0;
            uint32_t peak_idx_y = 0;
            uint32_t peak_idx_z = 0;
            float peak_val_x = 0.0f;
            float peak_val_y = 0.0f;
            float peak_val_z = 0.0f;
            for (uint32_t i = 1; i < half; ++i)
            {
                float vx = s_fft_mag_x[i];
                float vy = s_fft_mag_y[i];
                float vz = s_fft_mag_z[i];
                if (vx > peak_val_x)
                {
                    peak_val_x = vx;
                    peak_idx_x = i;
                }
                if (vy > peak_val_y)
                {
                    peak_val_y = vy;
                    peak_idx_y = i;
                }
                if (vz > peak_val_z)
                {
                    peak_val_z = vz;
                    peak_idx_z = i;
                }
            }
            const float bin_hz = job.sample_rate / (float)n;
            LOG_INFOF("Peaks: X=%.2fHz(%.4f), Y=%.2fHz(%.4f), Z=%.2fHz(%.4f)",
                      peak_idx_x * bin_hz, peak_val_x,
                      peak_idx_y * bin_hz, peak_val_y,
                      peak_idx_z * bin_hz, peak_val_z);

            LOG_INFO("FFT analysis complete.");

            // TODO: 将FFT结果发送到 data_dispatcher
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
