#ifndef TASK_FFT_H
#define TASK_FFT_H

#include "esp_err.h"
#include "task_rms.h" // for vib_job_t

#ifdef __cplusplus
extern "C"
{
#endif

extern QueueHandle_t g_fft_job_queue;

esp_err_t start_fft_task(void);

#ifdef __cplusplus
}
#endif // TASK_FFT_H
#endif // TASK_FFT_H
