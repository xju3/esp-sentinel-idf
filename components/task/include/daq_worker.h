#ifndef DAQ_WORKER_H
#define DAQ_WORKER_H

#include <stdint.h>
#include <stdbool.h>
#include "task_stash.h"

#ifdef __cplusplus
extern "C" {
#endif
#define LSB_TO_G_2G  (2.0f / 32768.0f)
#define LSB_TO_G_4G  (4.0f / 32768.0f)
#define LSB_TO_G_8G  (8.0f / 32768.0f)
#define LSB_TO_G_16G (16.0f / 32768.0f)
// 任务上下文结构体
typedef struct {
    int32_t rpm;
    task_mode_t task_mode;
} daq_worker_param_t;


esp_err_t start_daq_worker(daq_worker_param_t *param);


#ifdef __cplusplus
}
#endif

#endif // DAQ_WORKER_H