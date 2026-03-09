#ifndef TASK_ENVELOPE_H
#define TASK_ENVELOPE_H

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

extern QueueHandle_t g_envelope_job_queue;

esp_err_t start_envelope_task(void);

#ifdef __cplusplus
}
#endif // TASK_ENVELOPE_H
#endif // TASK_ENVELOPE_H
