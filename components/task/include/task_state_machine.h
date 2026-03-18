#ifndef TASK_STATE_MACHINE_H
#define TASK_STATE_MACHINE_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// This semaphore is given by the LIS2DH12TR ISR to trigger the state check handler.
extern SemaphoreHandle_t g_state_check_semaphore;

/**
 * @brief Creates the high-priority state check handler task.
 *
 * This task waits for a signal from the motion-detecting ISR and then
 * runs the state determination process.
 */
void create_state_check_handler_task(void);

#endif // TASK_STATE_MACHINE_H
