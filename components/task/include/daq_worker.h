#ifndef DAQ_WORKER_H
#define DAQ_WORKER_H

#include <stdint.h>
#include <stdbool.h>
#include "task_stash.h"

#ifdef __cplusplus
extern "C" {
#endif

// 任务上下文结构体
typedef struct {
    int32_t rpm;
    task_mode_t task_mode;
} daq_worker_param_t;


esp_err_t start_daq_worker(daq_worker_param_t *param);

/**
 * @brief 启动巡逻工作
 * 
 * @param rpm 设备转速 (RPM)
 * @return true 成功启动
 * @return false 启动失败
 */
bool start_patrolling_work(int32_t rpm);

/**
 * @brief 启动诊断工作
 * 
 * @param rpm 设备转速 (RPM)
 * @return true 成功启动
 * @return false 启动失败
 */
bool start_diagnosing_work(int32_t rpm);

#ifdef __cplusplus
}
#endif

#endif // DAQ_WORKER_H