#include "task_envelope.h"
#include "task_stash.h"
#include "algo_envelope.h"
#include "config_manager.h"
#include "logger.h"
#include "task_diag_fusion.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include "freertos/idf_additions.h"
#include <string.h>

#define ENVELOPE_QUEUE_LEN 5
#define MAX_DAQ_SAMPLES 8192
#define TASK_MEM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

QueueHandle_t g_envelope_job_queue = NULL;
EXT_RAM_BSS_ATTR static float s_envelope_work_buf[MAX_DAQ_SAMPLES];

static void log_envelope_report(const char *axis, const envelope_report_t *report)
{
    LOG_INFOF("Envelope Report [%s]: Peak=%.2f Hz (Mag=%.4f)", axis, report->peak_freq_hz, report->peak_mag);
    
    if (report->bpfo_stat.detected) {
        LOG_WARNF("[%s] BPFO Detected! Freq=%.2f Hz, Conf=%.2f", axis, report->bpfo_stat.frequency_hz, report->bpfo_stat.confidence);
    }
    if (report->bpfi_stat.detected) {
        LOG_WARNF("[%s] BPFI Detected! Freq=%.2f Hz, Conf=%.2f", axis, report->bpfi_stat.frequency_hz, report->bpfi_stat.confidence);
    }
    if (report->bs_stat.detected) {
        LOG_WARNF("[%s] BS Detected! Freq=%.2f Hz, Conf=%.2f", axis, report->bs_stat.frequency_hz, report->bs_stat.confidence);
    }
    if (report->ftf_stat.detected) {
        LOG_WARNF("[%s] FTF Detected! Freq=%.2f Hz, Conf=%.2f", axis, report->ftf_stat.frequency_hz, report->ftf_stat.confidence);
    }
}

static void envelope_task_entry(void *arg)
{
    vib_job_t job;
    envelope_report_t report;
    diag_envelope_summary_t summary;

    // Initialize algorithm resources (filters, etc.)
    algo_envelope_init();

    while (1)
    {
        if (xQueueReceive(g_envelope_job_queue, &job, portMAX_DELAY))
        {
            LOG_INFOF("Envelope Analysis Start. Len: %lu, SR: %.1f", job.length, job.sample_rate);
            memset(&summary, 0, sizeof(summary));
            summary.timestamp = job.timestamp;
            summary.length = job.length;
            summary.sample_rate = job.sample_rate;

            if (job.length == 0 || job.length > MAX_DAQ_SAMPLES)
            {
                LOG_ERRORF("Invalid envelope length: %lu", job.length);
                continue;
            }

            if (job.raw_data == NULL)
            {
                LOG_ERROR("Envelope raw_data is NULL");
                continue;
            }

            // Get bearing configuration
            // bearing_orders_t bearing = g_user_config.bearing;
            bearing_orders_t bearing = {0};
            float rpm = (float)g_user_config.rpm;

            // Pointers to raw data (Planar layout: X | Y | Z). Keep raw_data read-only and
            // stage each axis into the task-local work buffer for in-place envelope processing.
            const float *x_ptr = job.raw_data;
            const float *y_ptr = job.raw_data + MAX_DAQ_SAMPLES;
            const float *z_ptr = job.raw_data + MAX_DAQ_SAMPLES * 2;

            // Process X Axis
            memset(&report, 0, sizeof(report));
            memcpy(s_envelope_work_buf, x_ptr, job.length * sizeof(float));
            if (algo_envelope_execute(s_envelope_work_buf, job.length, job.sample_rate, rpm, &bearing, &report) == ESP_OK)
            {
                summary.x_axis = report;
                log_envelope_report("X", &report);
            }

            // Process Y Axis
            memset(&report, 0, sizeof(report));
            memcpy(s_envelope_work_buf, y_ptr, job.length * sizeof(float));
            if (algo_envelope_execute(s_envelope_work_buf, job.length, job.sample_rate, rpm, &bearing, &report) == ESP_OK)
            {
                summary.y_axis = report;
                log_envelope_report("Y", &report);
            }

            // Process Z Axis
            memset(&report, 0, sizeof(report));
            memcpy(s_envelope_work_buf, z_ptr, job.length * sizeof(float));
            if (algo_envelope_execute(s_envelope_work_buf, job.length, job.sample_rate, rpm, &bearing, &report) == ESP_OK)
            {
                summary.z_axis = report;
                log_envelope_report("Z", &report);
            }

            if (diag_fusion_submit_envelope(&summary) != ESP_OK)
            {
                LOG_WARN("Failed to submit diagnosis envelope summary");
            }

            LOG_INFO("Envelope Analysis Complete");
        }
    }
}

esp_err_t start_envelope_task(void)
{
    if (g_envelope_job_queue == NULL)
    {
        g_envelope_job_queue = xQueueCreateWithCaps(ENVELOPE_QUEUE_LEN, sizeof(vib_job_t), TASK_MEM_CAPS);
        if (g_envelope_job_queue == NULL)
        {
            LOG_ERROR("Failed to create ENVELOPE queue");
            return ESP_ERR_NO_MEM;
        }
    }

    if (xTaskCreateWithCaps(envelope_task_entry, "envelope_task", 4096, NULL, 3, NULL, TASK_MEM_CAPS) != pdPASS)
    {
        LOG_ERROR("Failed to create ENVELOPE task");
        return ESP_ERR_INVALID_STATE;
    }
    LOG_DEBUG("ENVELOPE analysis task started");
    return ESP_OK;
}
