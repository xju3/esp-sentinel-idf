#ifndef TASK_WOM_H
#define TASK_WOM_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#ifdef __cplusplus
extern "C"
{
#endif

    void imu_interrupt_init(void);
#ifdef __cplusplus
}
#endif

#endif // TASK_WOM_H