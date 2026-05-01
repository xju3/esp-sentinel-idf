#ifndef DATA_DISPATCHER_H
#define DATA_DISPATCHER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "msg_sentinel.pb-c.h"

#ifndef SN
#define SN 0 // Default SN value if not defined in config
#endif
#ifdef __cplusplus
extern "C"
{
#endif
    // Binary message: dispatcher 不关心数据内容和结构
    typedef struct
    {
        uint8_t *data;      // 二进制数据指针
        size_t len;         // 数据长度
        char topic[128];    // MQTT topic
    } binary_msg_t;

    // Queue is created/owned by producer, dispatcher only consumes
    // Queue items are of type binary_msg_t
    extern QueueHandle_t g_msg_dispatcher_queue;
    esp_err_t data_dispatcher_start(void);

    // Unified message sending function for protobuf messages
    esp_err_t send_protobuf_message(uint32_t event_type, const ProtobufCMessage *message);
    esp_err_t data_dispatcher_flush_all(TickType_t timeout_ticks);
    bool data_dispatcher_is_busy(void);
    esp_err_t data_dispatcher_wait_idle(TickType_t timeout_ticks);

#ifdef __cplusplus
}
#endif

#endif // DATA_DISPATCHER_H
