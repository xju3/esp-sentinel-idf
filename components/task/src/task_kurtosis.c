#include "task_kurtosis.h"
#include "task_stash.h" // for vib_job_t
#include "algo_kurtosis.h"
#include "task_diag_fusion.h"
#include "logger.h"
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"


#define KURTOSIS_QUEUE_LEN 5
#define MAX_DAQ_SAMPLES 8192
#define TASK_MEM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define KURTOSIS_ALARM_THRESHOLD 3.5f

QueueHandle_t g_kurtosis_job_queue = NULL;

static void kurtosis_task_entry(void *arg)
{
    vib_job_t job;
    while(1) {
        if (xQueueReceive(g_kurtosis_job_queue, &job, portMAX_DELAY))
        {
            LOG_INFOF("Kurtosis Analysis Start. Len: %lu, SR: %.1f", job.length, job.sample_rate);

            // 1. Parse planar layout pointers (X | Y | Z)
            const float *x_ptr = job.raw_data;
            const float *y_ptr = job.raw_data + MAX_DAQ_SAMPLES;
            const float *z_ptr = job.raw_data + MAX_DAQ_SAMPLES * 2;

            // 2. Perform Kurtosis calculation using ESP-DSP
            vib_kurtosis_t result = algo_kurtosis_calculate(x_ptr, y_ptr, z_ptr, job.length);

            // 3. Log results
            // Normal distribution kurtosis is ~3.0. High values indicate impulsive faults (e.g. bearing clicks).
            LOG_INFOF("Kurtosis Result: X=%.2f, Y=%.2f, Z=%.2f", 
                      result.x, result.y, result.z);
            
            // 4. Log diagnostic severity independently from the envelope path.
            float max_kurt = (result.x > result.y) ? result.x : result.y;
            max_kurt = (max_kurt > result.z) ? max_kurt : result.z;
            diag_kurtosis_summary_t summary = {
                .timestamp = job.timestamp,
                .length = job.length,
                .sample_rate = job.sample_rate,
                .kurtosis = result,
                .max_kurtosis = max_kurt,
                .threshold_exceeded = (max_kurt > KURTOSIS_ALARM_THRESHOLD),
            };
            if (diag_fusion_submit_kurtosis(&summary) != ESP_OK)
            {
                LOG_WARN("Failed to submit diagnosis kurtosis summary");
            }
            if (max_kurt > KURTOSIS_ALARM_THRESHOLD) {  
                LOG_WARNF("High Kurtosis Detected! Max=%.2f", max_kurt);
            }
        }
    }
}

esp_err_t start_kurtosis_task(void)
{
    if (g_kurtosis_job_queue == NULL)
    {
        g_kurtosis_job_queue = xQueueCreateWithCaps(KURTOSIS_QUEUE_LEN, sizeof(vib_job_t), TASK_MEM_CAPS);
        if (g_kurtosis_job_queue == NULL)
        {
            LOG_ERROR("Failed to create KURTOSIS queue");
            return ESP_ERR_NO_MEM;
        }
    }

    // 创建处理任务
    if (xTaskCreateWithCaps(kurtosis_task_entry, "kurtosis_task", 4096, NULL, 4, NULL, TASK_MEM_CAPS) != pdPASS)
    {
        LOG_ERROR("Failed to create KURTOSIS task");
        return ESP_ERR_INVALID_STATE;
    }
    LOG_DEBUG("KURTOSIS analysis task started");
    return ESP_OK;
}
