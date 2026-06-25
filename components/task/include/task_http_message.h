#ifndef TASK_HTTP_MESSAGE_H
#define TASK_HTTP_MESSAGE_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*http_task_action_handler_t)(int action, const char *ts);

typedef enum
{
    HTTP_PENDING_TASKS_NONE = 0,
    HTTP_PENDING_TASKS_KEEP_4G,
    HTTP_PENDING_TASKS_CAN_SHUTDOWN_4G,
} http_pending_tasks_result_t;

http_pending_tasks_result_t http_message_process_task_array(const cJSON *task_array);

esp_err_t start_http_message_task(void);
esp_err_t http_message_task_submit(const uint8_t *data, size_t len);
void http_message_task_register_action_handler(http_task_action_handler_t handler);

#ifdef __cplusplus
}
#endif

#endif // TASK_HTTP_MESSAGE_H
