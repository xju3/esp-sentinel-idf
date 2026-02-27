#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "data_dispatcher.h"

#ifdef __cplusplus
extern "C" {
#endif

// Monitor operating modes
typedef enum {
    MONITOR_MODE_NORMAL = 0,        // Normal periodic monitoring
    MONITOR_MODE_FFT_DIAGNOSIS = 1  // FFT diagnosis in progress
} monitor_mode_t;

// Global Queue Handle (created by task_monitor_start)
extern QueueHandle_t g_monitor_message_queue;

// Wakeup semaphore for monitor task
extern SemaphoreHandle_t g_wakeup_sem;

// Current monitor mode
extern monitor_mode_t g_monitor_mode;

// Start the monitor task
esp_err_t task_monitor_start(void);

// Control functions for FFT coordination
void task_monitor_pause_for_fft(void);
void task_monitor_resume_after_fft(void);

#ifdef __cplusplus
}
#endif

#endif // TASK_MONITOR_H