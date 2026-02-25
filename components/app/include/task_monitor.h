#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Message Types
#define MONITOR_MSG_TYPE_RMS 1
#define MONITOR_MSG_TYPE_FFT 2

// RMS Data Structure
typedef struct {
    int64_t timestamp;
    float rms_x;
    float rms_y;
    float rms_z;
} monitor_rms_data_t;

// Queue Message Wrapper
typedef struct {
    uint8_t type;
    union {
        monitor_rms_data_t rms;
        // future: monitor_fft_data_t fft;
    } payload;
} monitor_rms_msg_t;

// Global Queue Handle
extern QueueHandle_t g_monitor_rms_queue;

// Start the monitor task
esp_err_t task_monitor_start(void);

#ifdef __cplusplus
}
#endif

#endif // TASK_MONITOR_H