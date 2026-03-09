#ifndef TASK_KURTOSIS_H
#define TASK_KURTOSIS_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "task_stash.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

extern QueueHandle_t g_kurtosis_job_queue;

esp_err_t start_kurtosis_task(void);

#ifdef __cplusplus
}
#endif // TASK_KURTOSIS_H
#endif // TASK_KURTOSIS_H