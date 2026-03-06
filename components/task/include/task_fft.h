#ifndef TASK_FFT_H
#define TASK_FFT_H

#include "task_rms.h" // for vib_job_t

#ifdef __cplusplus
extern "C"
{
#endif

extern QueueHandle_t g_fft_job_queue;

void start_fft_task(void);

#ifdef __cplusplus
}
#endif // TASK_FFT_H
#endif // TASK_FFT_H
