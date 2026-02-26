#ifndef DATA_DISPATCHER_H
#define DATA_DISPATCHER_H

#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Queue is created/owned by task_monitor, dispatcher only consumes
extern QueueHandle_t g_monitor_message_queue;
esp_err_t data_dispatcher_start(void);

#ifdef __cplusplus
}
#endif

#endif // DATA_DISPATCHER_H