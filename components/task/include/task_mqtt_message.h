#ifndef TASK_MQTT_MESSAGE_H
#define TASK_MQTT_MESSAGE_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*mqtt_action_handler_t)(int action, const char *ts);

void mqtt_message_process_pending_tasks(void);

esp_err_t start_mqtt_message_task(void);
esp_err_t mqtt_message_task_submit(const char *topic, const uint8_t *data, size_t len);
void mqtt_message_task_register_action_handler(mqtt_action_handler_t handler);

#ifdef __cplusplus
}
#endif

#endif // TASK_MQTT_MESSAGE_H
