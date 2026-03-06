#ifndef DATA_DISPATCHER_H
#define DATA_DISPATCHER_H

#include "esp_err.h"
#include "drv_icm_42688_p.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif
// Message Types
#define MSG_TYPE_RMS 1
#define MSG_TYPE_FFT 2

// Queue Message Wrapper
typedef struct {
    uint8_t type;
    union {
        imu_rms_data_t rms;
        // future: monitor_fft_data_t fft;
    } payload;
} dispatch_msg_t;

// Queue is created/owned by task_monitor, dispatcher only consumes
extern QueueHandle_t g_monitor_message_queue;
esp_err_t data_dispatcher_start(void);

#ifdef __cplusplus
}
#endif

#endif // DATA_DISPATCHER_H