#include "task_kurtosis.h"
#include "logger.h"

#define KURTOSIS_QUEUE_LEN 5

QueueHandle_t g_kurtosis_job_queue = NULL;

static void kurtosis_task_entry(void *arg)
{
    while(1) {
        
    }
}

esp_err_t start_kurtosis_task(void)
{
    if (g_kurtosis_job_queue == NULL)
    {
        g_kurtosis_job_queue = xQueueCreate(KURTOSIS_QUEUE_LEN, sizeof(vib_job_t));
    }

    // 创建处理任务
    if (xTaskCreate(kurtosis_task_entry, "kurtosis_task", 4096, NULL, 4, NULL) != pdPASS)
    {
        LOG_ERROR("Failed to create KURTOSIS task");
        return ESP_ERR_INVALID_STATE;
    }
    LOG_DEBUG("KURTOSIS analysis task started");
    return ESP_OK;
}