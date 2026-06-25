#ifndef TASK_DAQ_H
#define TASK_DAQ_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief 执行基于RTC时间单次任务调度决策与采集 */
esp_err_t daq_scheduler_execute(void);

/** @brief 获取距离下一次任务唤醒所需的时间差 (微秒) */
uint64_t daq_scheduler_get_sleep_time_us(void);

esp_err_t start_task_daq(void);
esp_err_t task_daq_pause_periodic(void);
esp_err_t task_daq_resume_periodic(bool trigger_patrol_now);
esp_err_t task_daq_trigger_patrol_now(void);
esp_err_t task_daq_trigger_wom_patrol(void);
bool task_daq_periodic_enabled(void);
bool task_daq_is_idle(void);

#ifdef __cplusplus
}
#endif // TASK_daq_H
#endif // TASK_DAQ_H
