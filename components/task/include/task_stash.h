#ifndef TASK_STASH_H
#define TASK_STASH_H

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
    // daq operating modes
    typedef enum
    {
        TASK_MODE_PATROLING = 0, // Normal periodic daqing
        TASK_MODE_DIAGNOSIS = 1  // FFT diagnosis in progress
    } task_mode_t;

#ifdef __cplusplus
}
#endif // TASK_daq_H