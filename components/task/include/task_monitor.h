#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_err.h"
#include <time.h>
#include <string.h>
#include <math.h>

#include "config_manager.h"
#include "daq_icm_42688_p.h"
#include "data_dispatcher.h"
#include "drv_icm_42688_p.h"
#include "logger.h"

#endif

#ifdef __cplusplus
extern "C"
{
#endif

    // Monitor operating modes
    typedef enum
    {
        MONITOR_MODE_PATROLLING = 0, // Normal periodic monitoring
        MONITOR_MODE_DIAGNOSIS = 1   // FFT diagnosis in progress
    } monitor_mode_t;

    typedef struct
    {
        icm_cfg_t *cfg;
        imu_rms_data_t rms;
        esp_timer_handle_t s_timer;
    } task_monitor_params_t;

    // Global Queue Handle (created by task_monitor_start)
    extern QueueHandle_t g_monitor_message_queue;

    // Wakeup semaphore for monitor task
    extern SemaphoreHandle_t g_wakeup_sem;

    // Current monitor mode
    extern monitor_mode_t g_monitor_mode;

    // Start the monitor task
    esp_err_t task_monitor_start(void);

    // Control functions for FFT coordination
    void task_monitor_pause(void);
    void task_monitor_resume(void);

#ifdef __cplusplus
}
#endif // TASK_MONITOR_H