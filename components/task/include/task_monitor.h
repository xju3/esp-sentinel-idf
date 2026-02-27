#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "data_dispatcher.h"

#ifdef __cplusplus
extern "C" {
#endif


// Global Queue Handle (created by task_monitor_start)
extern QueueHandle_t g_monitor_message_queue;

// Start the monitor task
esp_err_t task_monitor_start(void);

#ifdef __cplusplus
}
#endif

#endif // TASK_MONITOR_H
