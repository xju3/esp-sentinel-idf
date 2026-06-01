#include "task_mqtt_message.h"
#include "cJSON.h"
#include "logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef SN
#define SN 0
#endif

#define MQTT_MESSAGE_QUEUE_LEN 5
#define MQTT_MESSAGE_TASK_STACK_SIZE 4096
#define MQTT_MESSAGE_MAX_JSON_LEN 1024

typedef struct {
    char *topic;
    char *json;
    size_t len;
} mqtt_message_item_t;

static QueueHandle_t s_mqtt_message_queue = NULL;
static TaskHandle_t s_mqtt_message_task = NULL;
static mqtt_action_handler_t s_action_handler = NULL;

static void mqtt_message_free_item(mqtt_message_item_t *item)
{
    if (item == NULL) {
        return;
    }
    free(item->topic);
    free(item->json);
    item->topic = NULL;
    item->json = NULL;
    item->len = 0;
}

static bool mqtt_message_sn_matches(const cJSON *sn)
{
    if (cJSON_IsNumber(sn)) {
        return (uint32_t)sn->valuedouble == (uint32_t)SN;
    }
    if (cJSON_IsString(sn) && sn->valuestring != NULL) {
        char expected[16];
        snprintf(expected, sizeof(expected), "%" PRIu32, (uint32_t)SN);
        return strcmp(sn->valuestring, expected) == 0;
    }
    return false;
}

static void mqtt_message_handle_action(int action, const char *ts)
{
    if (s_action_handler != NULL) {
        esp_err_t err = s_action_handler(action, ts);
        if (err != ESP_OK) {
            LOG_WARNF("MQTT action handler failed: action=%d err=%s",
                      action,
                      esp_err_to_name(err));
        }
        return;
    }

    LOG_INFOF("MQTT action received: action=%d ts=%s", action, ts != NULL ? ts : "");
}

static void mqtt_message_process_json(const char *topic, const char *json)
{
    cJSON *root = cJSON_Parse(json);
    if (!cJSON_IsObject(root)) {
        LOG_WARN("MQTT message ignored: invalid JSON object");
        cJSON_Delete(root);
        return;
    }

    const cJSON *sn = cJSON_GetObjectItemCaseSensitive(root, "sn");
    if (!mqtt_message_sn_matches(sn)) {
        LOG_DEBUGF("MQTT message ignored: SN mismatch topic=%s", topic != NULL ? topic : "");
        cJSON_Delete(root);
        return;
    }

    const cJSON *actions = cJSON_GetObjectItemCaseSensitive(root, "actions");
    if (!cJSON_IsArray(actions)) {
        LOG_WARN("MQTT message ignored: actions is not an array");
        cJSON_Delete(root);
        return;
    }

    const cJSON *ts = cJSON_GetObjectItemCaseSensitive(root, "ts");
    const char *ts_value = cJSON_IsString(ts) ? ts->valuestring : NULL;

    const cJSON *action = NULL;
    cJSON_ArrayForEach(action, actions) {
        if (!cJSON_IsNumber(action)) {
            LOG_WARN("MQTT action ignored: non-number action");
            continue;
        }
        mqtt_message_handle_action(action->valueint, ts_value);
    }

    cJSON_Delete(root);
}

static void mqtt_message_task_entry(void *arg)
{
    (void)arg;
    mqtt_message_item_t item = {0};

    while (true) {
        if (xQueueReceive(s_mqtt_message_queue, &item, portMAX_DELAY) == pdTRUE) {
            mqtt_message_process_json(item.topic, item.json);
            mqtt_message_free_item(&item);
        }
    }
}

esp_err_t start_mqtt_message_task(void)
{
    if (s_mqtt_message_queue == NULL) {
        s_mqtt_message_queue = xQueueCreate(MQTT_MESSAGE_QUEUE_LEN, sizeof(mqtt_message_item_t));
        if (s_mqtt_message_queue == NULL) {
            LOG_ERROR("Failed to create MQTT message queue");
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_mqtt_message_task != NULL) {
        return ESP_OK;
    }

    if (xTaskCreate(mqtt_message_task_entry,
                    "mqtt_message",
                    MQTT_MESSAGE_TASK_STACK_SIZE,
                    NULL,
                    4,
                    &s_mqtt_message_task) != pdPASS) {
        LOG_ERROR("Failed to create MQTT message task");
        return ESP_ERR_INVALID_STATE;
    }

    LOG_INFO("MQTT message task started");
    return ESP_OK;
}

esp_err_t mqtt_message_task_submit(const char *topic, const uint8_t *data, size_t len)
{
    if (s_mqtt_message_queue == NULL || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > MQTT_MESSAGE_MAX_JSON_LEN) {
        LOG_WARNF("MQTT message ignored: JSON too large len=%u", (unsigned)len);
        return ESP_ERR_INVALID_SIZE;
    }

    mqtt_message_item_t item = {0};
    item.json = calloc(1, len + 1);
    if (item.json == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(item.json, data, len);
    item.len = len;

    if (topic != NULL && topic[0] != '\0') {
        size_t topic_len = strlen(topic);
        item.topic = malloc(topic_len + 1);
        if (item.topic == NULL) {
            mqtt_message_free_item(&item);
            return ESP_ERR_NO_MEM;
        }
        memcpy(item.topic, topic, topic_len + 1);
    }

    if (xQueueSend(s_mqtt_message_queue, &item, 0) != pdTRUE) {
        mqtt_message_free_item(&item);
        LOG_WARN("MQTT message queue full, dropping message");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

void mqtt_message_task_register_action_handler(mqtt_action_handler_t handler)
{
    s_action_handler = handler;
}
