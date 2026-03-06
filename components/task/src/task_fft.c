#include "task_fft.h"
#include "algo_fft.h"
#include "logger.h"
#include "data_dispatcher.h"
#include "esp_timer.h"

#define MAX_DAQ_SAMPLES 8192
#define FFT_QUEUE_LEN   5

QueueHandle_t g_fft_job_queue = NULL;

static void fft_task_entry(void *arg)
{
    vib_job_t job;
    while (1)
    {
        if (xQueueReceive(g_fft_job_queue, &job, portMAX_DELAY))
        {
            LOG_INFOF("[TASK_FFT] Received job for FFT analysis. Mode: %d, Len: %lu, SR: %.1f",
                      job.task_mode, job.length, job.sample_rate);

            // 1. 根据平面化布局 (Planar Layout) 获取各轴指针
            // const float *x_ptr = job.raw_data;
            // const float *y_ptr = job.raw_data + MAX_DAQ_SAMPLES;
            // const float *z_ptr = job.raw_data + MAX_DAQ_SAMPLES * 2;
            
            // TODO: 调用 algo_fft_calculate 并处理结果
            LOG_INFO("[TASK_FFT] Performing 3-axis FFT analysis...");

            // 示例: 对X轴进行FFT
            // algo_fft_calculate(x_ptr, job.length, job.sample_rate, ...);

            LOG_INFO("[TASK_FFT] FFT analysis complete.");

            // TODO: 将FFT结果发送到 data_dispatcher
        }
    }
}

void start_fft_task(void)
{
    if (g_fft_job_queue == NULL)
    {
        g_fft_job_queue = xQueueCreate(FFT_QUEUE_LEN, sizeof(vib_job_t));
    }

    if (xTaskCreate(fft_task_entry, "fft_task", 4096 * 2, NULL, 3, NULL) != pdPASS)
    {
        LOG_ERROR("[TASK_FFT] Failed to create FFT task");
    }
    else
    {
        LOG_INFO("[TASK_FFT] FFT analysis task started");
    }
}
