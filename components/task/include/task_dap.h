#ifndef TASK_DAQ_H
#define TASK_DAQ_H

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
#include "drv_icm_42688_p.h"
#include "imu_config.h"
#include "logger.h"
#include "task_stash.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

void start_task_daq(void);

#ifdef __cplusplus
}
#endif // TASK_daq_H