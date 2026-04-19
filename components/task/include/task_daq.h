#ifndef TASK_DAQ_H
#define TASK_DAQ_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

esp_err_t start_task_daq(void);
esp_err_t task_daq_pause_periodic(void);
esp_err_t task_daq_resume_periodic(bool trigger_patrol_now);
esp_err_t task_daq_trigger_patrol_now(void);
bool task_daq_periodic_enabled(void);

#ifdef __cplusplus
}
#endif // TASK_daq_H
#endif // TASK_DAQ_H
