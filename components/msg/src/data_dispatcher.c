#include "data_dispatcher.h"
#include "bsp_4g.h"
#include "bsp_wifi.h"
#include "config_manager.h"
#include "fs_utils.h"
#include "logger.h"
#include "mqtt_client.h"
#include "mqtt_proxy.h"
#include "msg_sentinel.pb-c.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

QueueHandle_t g_msg_dispatcher_queue = NULL;

extern esp_mqtt_client_handle_t g_mqtt_client;

#define DISPATCHER_QUEUE_LEN 20
#define DISPATCHER_MAX_PENDING_MSGS 64
#define DISPATCHER_RETRY_INTERVAL_MS 5000
#define EVENT_TYPE_RMS_REPORT 1U

typedef enum
{
    DISPATCHER_ITEM_PAYLOAD = 0,
    DISPATCHER_ITEM_FLUSH = 1,
    DISPATCHER_ITEM_MQTT_READY = 2,
    DISPATCHER_ITEM_MQTT_PUBLISHED = 3,
    DISPATCHER_ITEM_MQTT_FAILED = 4,
} dispatcher_item_type_t;

typedef struct
{
    dispatcher_item_type_t type;
    MsgPayload payload;
    int32_t msg_id;
    SemaphoreHandle_t done_sem;
    bool *flush_result_out;
} dispatcher_queue_item_t;

typedef struct
{
    uint8_t *data;
    size_t len;
} dispatcher_persisted_msg_t;

static void dispatcher_enqueue_internal_event(dispatcher_item_type_t type, int32_t msg_id);

static bool is_mqtt_connected(void)
{
    return g_mqtt_client != NULL;
}

static uint16_t dispatcher_batch_target(void)
{
    if (g_user_config.report <= 0)
    {
        return 1U;
    }
    if (g_user_config.report > DISPATCHER_MAX_PENDING_MSGS)
    {
        return DISPATCHER_MAX_PENDING_MSGS;
    }
    return (uint16_t)g_user_config.report;
}

static bool dispatcher_should_flush_payload(const MsgPayload *msg)
{
    if (!msg)
    {
        return false;
    }

    return (msg->et == EVENT_TYPE_RMS_REPORT &&
            g_user_config.patrol >= CONFIG_SENTINEL_DIRECT_REPORT_PATROL_THRESHOLD_MIN);
}

static void free_payload_data(MsgPayload *msg)
{
    if (!msg)
    {
        return;
    }
    free(msg->data.data);
    msg->data.data = NULL;
    msg->data.len = 0;
}

static void dispatcher_free_persisted_msgs(dispatcher_persisted_msg_t *msgs, size_t count)
{
    if (!msgs)
    {
        return;
    }

    for (size_t i = 0; i < count; ++i)
    {
        free(msgs[i].data);
    }
    free(msgs);
}

static esp_err_t dispatcher_ensure_user_storage(void)
{
    if (fsu_is_user_mounted())
    {
        return ESP_OK;
    }
    return fsu_mount_user(false);
}

static esp_err_t dispatcher_pack_payload(const MsgPayload *msg, dispatcher_persisted_msg_t *out)
{
    if (!msg || !out)
    {
        return ESP_ERR_INVALID_ARG;
    }

    MsgPayload normalized = *msg;
    if (normalized.sn == 0)
    {
        normalized.sn = SN;
    }

    size_t packed_size = msg_payload__get_packed_size(&normalized);
    uint8_t *buffer = malloc(packed_size);
    if (buffer == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    msg_payload__pack(&normalized, buffer);
    out->data = buffer;
    out->len = packed_size;
    return ESP_OK;
}

static esp_err_t dispatcher_write_cache(const dispatcher_persisted_msg_t *msgs, size_t count)
{
    esp_err_t err = dispatcher_ensure_user_storage();
    if (err != ESP_OK)
    {
        LOG_ERRORF("User storage unavailable for dispatcher cache: %s", esp_err_to_name(err));
        return err;
    }

    if (count == 0U)
    {
        remove(FILE_PATH_DISPATCH_CACHE);
        return ESP_OK;
    }

    FILE *f = fopen(FILE_PATH_DISPATCH_CACHE, "wb");
    if (!f)
    {
        LOG_ERRORF("Failed to open %s for dispatcher cache write", FILE_PATH_DISPATCH_CACHE);
        return ESP_FAIL;
    }

    for (size_t i = 0; i < count; ++i)
    {
        uint32_t len = (uint32_t)msgs[i].len;
        if (fwrite(&len, sizeof(len), 1, f) != 1 ||
            fwrite(msgs[i].data, 1, msgs[i].len, f) != msgs[i].len)
        {
            fclose(f);
            LOG_ERROR("Failed to write dispatcher cache");
            return ESP_FAIL;
        }
    }

    fclose(f);
    return ESP_OK;
}

static esp_err_t dispatcher_load_cache(dispatcher_persisted_msg_t **out_msgs, size_t *out_count)
{
    if (!out_msgs || !out_count)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_msgs = NULL;
    *out_count = 0U;

    if (!fsu_file_exists(FILE_PATH_DISPATCH_CACHE))
    {
        return ESP_ERR_NOT_FOUND;
    }

    FILE *f = fopen(FILE_PATH_DISPATCH_CACHE, "rb");
    if (!f)
    {
        LOG_ERRORF("Failed to open %s for dispatcher cache read", FILE_PATH_DISPATCH_CACHE);
        return ESP_FAIL;
    }

    dispatcher_persisted_msg_t *msgs = NULL;
    size_t count = 0U;
    while (1)
    {
        uint32_t len = 0U;
        size_t read_len = fread(&len, sizeof(len), 1, f);
        if (read_len != 1)
        {
            break;
        }

        dispatcher_persisted_msg_t *grown =
            realloc(msgs, (count + 1U) * sizeof(dispatcher_persisted_msg_t));
        if (!grown)
        {
            fclose(f);
            dispatcher_free_persisted_msgs(msgs, count);
            return ESP_ERR_NO_MEM;
        }
        msgs = grown;
        msgs[count].data = malloc(len);
        msgs[count].len = len;
        if (!msgs[count].data)
        {
            fclose(f);
            dispatcher_free_persisted_msgs(msgs, count);
            return ESP_ERR_NO_MEM;
        }
        if (fread(msgs[count].data, 1, len, f) != len)
        {
            fclose(f);
            dispatcher_free_persisted_msgs(msgs, count + 1U);
            return ESP_FAIL;
        }
        count++;
    }

    fclose(f);
    *out_msgs = msgs;
    *out_count = count;
    return (count > 0U) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static void dispatcher_network_channel_established(void)
{
    esp_err_t err = init_mqtt_client();
    if (err != ESP_OK)
    {
        LOG_ERRORF("MQTT initialization failed after network bring-up: %s", esp_err_to_name(err));
        dispatcher_enqueue_internal_event(DISPATCHER_ITEM_MQTT_FAILED, -1);
    }
}

static void dispatcher_enqueue_internal_event(dispatcher_item_type_t type, int32_t msg_id)
{
    if (g_msg_dispatcher_queue == NULL)
    {
        return;
    }

    dispatcher_queue_item_t item = {
        .type = type,
        .msg_id = msg_id,
    };
    if (xQueueSend(g_msg_dispatcher_queue, &item, 0) != pdTRUE)
    {
        LOG_WARN("Failed to enqueue dispatcher internal event");
    }
}

static void dispatcher_mqtt_event_handler(mqtt_proxy_event_t event, int32_t msg_id, void *user_ctx)
{
    (void)user_ctx;
    switch (event)
    {
        case MQTT_PROXY_EVENT_READY:
            dispatcher_enqueue_internal_event(DISPATCHER_ITEM_MQTT_READY, msg_id);
            break;
        case MQTT_PROXY_EVENT_PUBLISHED:
            dispatcher_enqueue_internal_event(DISPATCHER_ITEM_MQTT_PUBLISHED, msg_id);
            break;
        case MQTT_PROXY_EVENT_DISCONNECTED:
        case MQTT_PROXY_EVENT_ERROR:
            dispatcher_enqueue_internal_event(DISPATCHER_ITEM_MQTT_FAILED, msg_id);
            break;
        default:
            break;
    }
}

static esp_err_t dispatcher_request_transport(void)
{
    if (g_user_config.network == 1)
    {
        LOG_INFO("Dispatcher bringing up 4G transport before send");
        return init_ppp_4g(dispatcher_network_channel_established);
    }

    LOG_INFO("Dispatcher bringing up WiFi STA transport before send");
    return wifi_init_sta(g_user_config.wifi.ssid,
                         g_user_config.wifi.pass,
                         dispatcher_network_channel_established);
}

static void dispatcher_shutdown_transport(void)
{
    (void)mqtt_client_stop();

    if (g_user_config.network == 1)
    {
        (void)shutdown_ppp_4g();
    }
    else
    {
        (void)wifi_stop_sta();
    }
}

static void dispatcher_finish_batch(bool success,
                                    bool *flush_requested,
                                    bool *immediate_flush_pending,
                                    bool *mqtt_ready,
                                    bool *transport_requested,
                                    bool *send_in_progress,
                                    SemaphoreHandle_t *flush_done_sem,
                                    bool **flush_result_out)
{
    if (flush_result_out && *flush_result_out)
    {
        **flush_result_out = success;
    }
    if (flush_done_sem && *flush_done_sem)
    {
        xSemaphoreGive(*flush_done_sem);
    }
    if (flush_done_sem)
    {
        *flush_done_sem = NULL;
    }
    if (flush_result_out)
    {
        *flush_result_out = NULL;
    }
    if (flush_requested)
    {
        *flush_requested = false;
    }
    if (immediate_flush_pending)
    {
        *immediate_flush_pending = false;
    }
    if (send_in_progress)
    {
        *send_in_progress = false;
    }
    if (mqtt_ready)
    {
        *mqtt_ready = false;
    }
    if (transport_requested && *transport_requested)
    {
        dispatcher_shutdown_transport();
        *transport_requested = false;
    }
}

static void dispatcher_handle_transport_failure(size_t pending_count,
                                                bool *flush_requested,
                                                bool *immediate_flush_pending,
                                                bool *mqtt_ready,
                                                bool *transport_requested,
                                                bool *send_in_progress,
                                                SemaphoreHandle_t *flush_done_sem,
                                                bool **flush_result_out)
{
    LOG_WARNF("Dispatcher transport/session failed, pending=%u", (unsigned)pending_count);
    dispatcher_finish_batch(false,
                            flush_requested,
                            immediate_flush_pending,
                            mqtt_ready,
                            transport_requested,
                            send_in_progress,
                            flush_done_sem,
                            flush_result_out);
}

static bool dispatcher_publish_one(const dispatcher_persisted_msg_t *msg, int32_t *out_msg_id)
{
    if (!msg || !out_msg_id || !is_mqtt_connected())
    {
        return false;
    }

    int msg_id = esp_mqtt_client_publish(g_mqtt_client,
                                         "sentinel",
                                         (const char *)msg->data,
                                         msg->len,
                                         1,
                                         0);
    if (msg_id == -1)
    {
        LOG_WARN("MQTT publish request failed");
        return false;
    }

    *out_msg_id = msg_id;
    return true;
}

static esp_err_t dispatcher_build_active_batch(MsgPayload *pending_msgs,
                                               size_t *pending_count,
                                               dispatcher_persisted_msg_t **active_msgs,
                                               size_t *active_count)
{
    if (!pending_msgs || !pending_count || !active_msgs || !active_count)
    {
        return ESP_ERR_INVALID_ARG;
    }

    dispatcher_persisted_msg_t *cached_msgs = NULL;
    size_t cached_count = 0U;
    esp_err_t load_err = dispatcher_load_cache(&cached_msgs, &cached_count);
    if (load_err != ESP_OK && load_err != ESP_ERR_NOT_FOUND)
    {
        return load_err;
    }

    const size_t total_count = cached_count + *pending_count;
    if (total_count == 0U)
    {
        free(cached_msgs);
        *active_msgs = NULL;
        *active_count = 0U;
        return ESP_OK;
    }

    dispatcher_persisted_msg_t *combined =
        calloc(total_count, sizeof(dispatcher_persisted_msg_t));
    if (!combined)
    {
        dispatcher_free_persisted_msgs(cached_msgs, cached_count);
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i < cached_count; ++i)
    {
        combined[i] = cached_msgs[i];
    }
    free(cached_msgs);

    for (size_t i = 0; i < *pending_count; ++i)
    {
        esp_err_t err = dispatcher_pack_payload(&pending_msgs[i], &combined[cached_count + i]);
        if (err != ESP_OK)
        {
            dispatcher_free_persisted_msgs(combined, cached_count + i);
            return err;
        }
    }

    esp_err_t write_err = dispatcher_write_cache(combined, total_count);
    if (write_err != ESP_OK)
    {
        dispatcher_free_persisted_msgs(combined, total_count);
        return write_err;
    }

    for (size_t i = 0; i < *pending_count; ++i)
    {
        free_payload_data(&pending_msgs[i]);
    }
    *pending_count = 0U;

    *active_msgs = combined;
    *active_count = total_count;
    return ESP_OK;
}

static esp_err_t dispatcher_persist_failure_state(dispatcher_persisted_msg_t *active_msgs,
                                                  size_t active_count,
                                                  MsgPayload *pending_msgs,
                                                  size_t *pending_count)
{
    if (!pending_count)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t extra_count = *pending_count;
    const size_t total_count = active_count + extra_count;
    dispatcher_persisted_msg_t *combined =
        calloc((total_count > 0U) ? total_count : 1U, sizeof(dispatcher_persisted_msg_t));
    if (!combined)
    {
        return ESP_ERR_NO_MEM;
    }

    size_t copied = 0U;
    for (size_t i = 0; i < active_count; ++i)
    {
        dispatcher_persisted_msg_t *slot = &combined[copied++];
        slot->data = malloc(active_msgs[i].len);
        slot->len = active_msgs[i].len;
        if (!slot->data)
        {
            dispatcher_free_persisted_msgs(combined, copied);
            return ESP_ERR_NO_MEM;
        }
        memcpy(slot->data, active_msgs[i].data, active_msgs[i].len);
    }

    for (size_t i = 0; i < extra_count; ++i)
    {
        esp_err_t err = dispatcher_pack_payload(&pending_msgs[i], &combined[copied]);
        if (err != ESP_OK)
        {
            dispatcher_free_persisted_msgs(combined, copied);
            return err;
        }
        copied++;
    }

    esp_err_t write_err = dispatcher_write_cache(combined, total_count);
    if (write_err == ESP_OK)
    {
        for (size_t i = 0; i < extra_count; ++i)
        {
            free_payload_data(&pending_msgs[i]);
        }
        *pending_count = 0U;
    }

    dispatcher_free_persisted_msgs(combined, total_count);
    return write_err;
}

static void dispatcher_compact_active_batch(dispatcher_persisted_msg_t *active_msgs,
                                            size_t *active_count,
                                            size_t consumed_count)
{
    if (!active_msgs || !active_count || consumed_count == 0U || consumed_count > *active_count)
    {
        return;
    }

    for (size_t i = 0; i < consumed_count; ++i)
    {
        free(active_msgs[i].data);
        active_msgs[i].data = NULL;
        active_msgs[i].len = 0U;
    }

    const size_t remaining = *active_count - consumed_count;
    if (remaining > 0U)
    {
        memmove(active_msgs, active_msgs + consumed_count, remaining * sizeof(dispatcher_persisted_msg_t));
    }
    *active_count = remaining;
}

static void dispatcher_maybe_progress(size_t *pending_count,
                                      MsgPayload *pending_msgs,
                                      bool *flush_requested,
                                      bool *immediate_flush_pending,
                                      bool *mqtt_ready,
                                      bool *transport_requested,
                                      bool *send_in_progress,
                                      int32_t *inflight_msg_id,
                                      int64_t *transport_deadline_us,
                                      dispatcher_persisted_msg_t **active_msgs,
                                      size_t *active_count,
                                      SemaphoreHandle_t *flush_done_sem,
                                      bool **flush_result_out)
{
    if (!pending_count || !pending_msgs || !flush_requested || !immediate_flush_pending ||
        !mqtt_ready || !transport_requested || !send_in_progress || !inflight_msg_id ||
        !transport_deadline_us || !active_msgs || !active_count)
    {
        return;
    }

    if (!*flush_requested)
    {
        return;
    }

    if (*active_count == 0U)
    {
        if (*active_msgs != NULL)
        {
            dispatcher_free_persisted_msgs(*active_msgs, *active_count);
            *active_msgs = NULL;
        }

        esp_err_t build_err = dispatcher_build_active_batch(pending_msgs,
                                                            pending_count,
                                                            active_msgs,
                                                            active_count);
        if (build_err != ESP_OK)
        {
            LOG_ERRORF("Failed to build dispatcher active batch: %s", esp_err_to_name(build_err));
            dispatcher_finish_batch(false,
                                    flush_requested,
                                    immediate_flush_pending,
                                    mqtt_ready,
                                    transport_requested,
                                    send_in_progress,
                                    flush_done_sem,
                                    flush_result_out);
            return;
        }
        *immediate_flush_pending = false;
    }

    if (*active_count == 0U)
    {
        dispatcher_finish_batch(true,
                                flush_requested,
                                immediate_flush_pending,
                                mqtt_ready,
                                transport_requested,
                                send_in_progress,
                                flush_done_sem,
                                flush_result_out);
        return;
    }

    if (*send_in_progress)
    {
        return;
    }

    if (!*mqtt_ready || !is_mqtt_connected())
    {
        if (!*transport_requested)
        {
            esp_err_t err = dispatcher_request_transport();
            if (err != ESP_OK)
            {
                LOG_ERRORF("Transport bring-up start failed: %s", esp_err_to_name(err));
                dispatcher_handle_transport_failure(*pending_count,
                                                    flush_requested,
                                                    immediate_flush_pending,
                                                    mqtt_ready,
                                                    transport_requested,
                                                    send_in_progress,
                                                    flush_done_sem,
                                                    flush_result_out);
                return;
            }
            *transport_requested = true;
            *transport_deadline_us = esp_timer_get_time() +
                                     ((int64_t)CONFIG_SENTINEL_NETWORK_BRINGUP_TIMEOUT_SEC * 1000000LL);
        }
        else if (*transport_deadline_us > 0 &&
                 esp_timer_get_time() > *transport_deadline_us)
        {
            LOG_WARN("Timed out waiting for network/MQTT bring-up");
            dispatcher_handle_transport_failure(*pending_count,
                                                flush_requested,
                                                immediate_flush_pending,
                                                mqtt_ready,
                                                transport_requested,
                                                send_in_progress,
                                                flush_done_sem,
                                                flush_result_out);
            *transport_deadline_us = 0;
        }
        return;
    }

    int32_t msg_id = -1;
    if (!dispatcher_publish_one(&(*active_msgs)[0], &msg_id))
    {
        dispatcher_handle_transport_failure(*pending_count,
                                            flush_requested,
                                            immediate_flush_pending,
                                            mqtt_ready,
                                            transport_requested,
                                            send_in_progress,
                                            flush_done_sem,
                                            flush_result_out);
        return;
    }

    *send_in_progress = true;
    *inflight_msg_id = msg_id;
}

static void dispatcher_task(void *arg)
{
    dispatcher_queue_item_t item;
    MsgPayload pending_msgs[DISPATCHER_MAX_PENDING_MSGS] = {0};
    size_t pending_count = 0U;
    bool flush_requested = false;
    bool immediate_flush_pending = false;
    bool mqtt_ready = false;
    bool transport_requested = false;
    bool send_in_progress = false;
    int32_t inflight_msg_id = -1;
    int64_t transport_deadline_us = 0;
    dispatcher_persisted_msg_t *active_msgs = NULL;
    size_t active_count = 0U;
    SemaphoreHandle_t flush_done_sem = NULL;
    bool *flush_result_out = NULL;

    LOG_INFO("Data dispatcher started. Ready to batch MsgPayload messages for MQTT");
    while (1)
    {
        if (g_msg_dispatcher_queue == NULL)
        {
            return;
        }

        if (xQueueReceive(g_msg_dispatcher_queue, &item, pdMS_TO_TICKS(DISPATCHER_RETRY_INTERVAL_MS)) == pdTRUE)
        {
            if (item.type == DISPATCHER_ITEM_PAYLOAD)
            {
                MsgPayload *msg = &item.payload;
                if (pending_count >= DISPATCHER_MAX_PENDING_MSGS)
                {
                    LOG_WARN("Pending dispatcher buffer full, dropping newest message");
                    free_payload_data(msg);
                }
                else
                {
                    pending_msgs[pending_count++] = *msg;
                    LOG_DEBUGF("Queued dispatcher message, pending=%u/%u, batch_target=%u",
                               (unsigned)pending_count,
                               (unsigned)DISPATCHER_MAX_PENDING_MSGS,
                               (unsigned)dispatcher_batch_target());
                    if (dispatcher_should_flush_payload(msg))
                    {
                        immediate_flush_pending = true;
                    }
                }
            }
            else if (item.type == DISPATCHER_ITEM_FLUSH)
            {
                flush_requested = true;
                if (flush_done_sem == NULL)
                {
                    flush_done_sem = item.done_sem;
                    flush_result_out = item.flush_result_out;
                }
                else
                {
                    if (item.flush_result_out)
                    {
                        *item.flush_result_out = false;
                    }
                    if (item.done_sem)
                    {
                        xSemaphoreGive(item.done_sem);
                    }
                }
            }
            else if (item.type == DISPATCHER_ITEM_MQTT_READY)
            {
                mqtt_ready = true;
                transport_deadline_us = 0;
            }
            else if (item.type == DISPATCHER_ITEM_MQTT_PUBLISHED)
            {
                if (send_in_progress && item.msg_id == inflight_msg_id && active_count > 0U)
                {
                    dispatcher_compact_active_batch(active_msgs, &active_count, 1U);
                    inflight_msg_id = -1;
                    send_in_progress = false;
                    esp_err_t cache_err = dispatcher_write_cache(active_msgs, active_count);
                    if (cache_err != ESP_OK)
                    {
                        LOG_ERRORF("Failed to update dispatcher cache after publish ack: %s",
                                   esp_err_to_name(cache_err));
                    }
                    LOG_INFOF("Dispatcher publish acknowledged, remaining=%u",
                              (unsigned)active_count);
                }
            }
            else if (item.type == DISPATCHER_ITEM_MQTT_FAILED)
            {
                if (transport_requested || send_in_progress || flush_requested)
                {
                    esp_err_t persist_err = dispatcher_persist_failure_state(active_msgs,
                                                                             active_count,
                                                                             pending_msgs,
                                                                             &pending_count);
                    if (persist_err == ESP_OK)
                    {
                        dispatcher_free_persisted_msgs(active_msgs, active_count);
                        active_msgs = NULL;
                        active_count = 0U;
                    }
                    else
                    {
                        LOG_ERRORF("Failed to persist dispatcher failure state: %s", esp_err_to_name(persist_err));
                    }

                    dispatcher_handle_transport_failure(pending_count,
                                                        &flush_requested,
                                                        &immediate_flush_pending,
                                                        &mqtt_ready,
                                                        &transport_requested,
                                                        &send_in_progress,
                                                        &flush_done_sem,
                                                        &flush_result_out);
                }
            }
        }

        if (pending_count >= dispatcher_batch_target())
        {
            flush_requested = true;
        }
        if (immediate_flush_pending)
        {
            flush_requested = true;
        }

        dispatcher_maybe_progress(&pending_count,
                                  pending_msgs,
                                  &flush_requested,
                                  &immediate_flush_pending,
                                  &mqtt_ready,
                                  &transport_requested,
                                  &send_in_progress,
                                  &inflight_msg_id,
                                  &transport_deadline_us,
                                  &active_msgs,
                                  &active_count,
                                  &flush_done_sem,
                                  &flush_result_out);
    }
}

esp_err_t send_protobuf_message(uint32_t event_type, const ProtobufCMessage *message)
{
    if (g_msg_dispatcher_queue == NULL || message == NULL)
    {
        LOG_ERROR("Message dispatcher queue not initialized or invalid message");
        return ESP_ERR_INVALID_STATE;
    }

    size_t packed_size = protobuf_c_message_get_packed_size(message);
    uint8_t *buffer = malloc(packed_size);
    if (buffer == NULL)
    {
        LOG_ERROR("Failed to allocate buffer for message packing");
        return ESP_ERR_NO_MEM;
    }
    protobuf_c_message_pack(message, buffer);

    dispatcher_queue_item_t item = {0};
    item.type = DISPATCHER_ITEM_PAYLOAD;
    item.payload = (MsgPayload)MSG_PAYLOAD__INIT;
    item.payload.et = event_type;
    item.payload.data.len = packed_size;
    item.payload.data.data = buffer;

    BaseType_t result = xQueueSend(g_msg_dispatcher_queue, &item, 0);
    if (result != pdTRUE)
    {
        free(buffer);
        LOG_WARN("Failed to send message to dispatcher queue");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t data_dispatcher_flush_all(TickType_t timeout_ticks)
{
    if (g_msg_dispatcher_queue == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
    if (done_sem == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    bool flush_success = false;
    dispatcher_queue_item_t item = {
        .type = DISPATCHER_ITEM_FLUSH,
        .done_sem = done_sem,
        .flush_result_out = &flush_success,
    };

    BaseType_t queued = xQueueSend(g_msg_dispatcher_queue, &item, timeout_ticks);
    if (queued != pdTRUE)
    {
        vSemaphoreDelete(done_sem);
        LOG_WARN("Failed to enqueue dispatcher flush request");
        return ESP_ERR_TIMEOUT;
    }

    BaseType_t completed = xSemaphoreTake(done_sem, timeout_ticks);
    vSemaphoreDelete(done_sem);
    if (completed != pdTRUE)
    {
        LOG_WARN("Timed out waiting for dispatcher flush completion");
        return ESP_ERR_TIMEOUT;
    }

    if (!flush_success)
    {
        LOG_WARN("Dispatcher flush completed with pending messages remaining");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t data_dispatcher_start(void)
{
    if (!g_msg_dispatcher_queue)
    {
        g_msg_dispatcher_queue = xQueueCreate(DISPATCHER_QUEUE_LEN, sizeof(dispatcher_queue_item_t));
        if (!g_msg_dispatcher_queue)
        {
            LOG_ERROR("Failed to create message dispatcher queue!");
            return ESP_FAIL;
        }
        LOG_INFO("Created message dispatcher queue");
    }

    mqtt_proxy_set_event_callback(dispatcher_mqtt_event_handler, NULL);

    xTaskCreate(dispatcher_task, "data_dispatcher", 4096, NULL, 5, NULL);
    LOG_INFO("Data dispatcher task created");
    return ESP_OK;
}
