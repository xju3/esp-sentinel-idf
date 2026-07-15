#ifndef TASK_DAQ_H
#define TASK_DAQ_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef esp_err_t (*daq_before_upload_fn)(void *ctx);

/** @brief Evaluate the RTC schedule without starting capture or networking. */
esp_err_t daq_scheduler_prepare(bool *out_has_report_work);

/**
 * @brief Execute prepared reports, starting prepare_upload after the final raw
 * sample while report calculation continues, then join before upload.
 */
esp_err_t daq_scheduler_execute(daq_before_upload_fn prepare_upload, void *ctx);

/** @brief 获取距离下一次任务唤醒所需的时间差 (微秒) */
uint64_t daq_scheduler_get_sleep_time_us(void);

esp_err_t task_daq_trigger_patrol_now(void);
esp_err_t task_daq_trigger_wom_patrol(void);


#ifdef __cplusplus
}
#endif // TASK_daq_H
#endif // TASK_DAQ_H
