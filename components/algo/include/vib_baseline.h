#ifndef VIB_BASELINE_H
#define VIB_BASELINE_H

#include <stdbool.h>
#include <stddef.h>

#include "cJSON.h"
#include "esp_err.h"
#include "vib_welford_feature.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float val;     // baseline mean（例如：静态零偏均值）
    float offset;  // baseline offset（例如：环境噪声RMS偏置）
} vib_axis_baseline_t;

typedef struct {
    vib_axis_baseline_t x;
    vib_axis_baseline_t y;
    vib_axis_baseline_t z;
} vib_baseline_t;

// 全局 StreamBuffer 句柄 (水管)
// 注意：不要在头文件里定义变量（否则会导致 multiple definition 链接错误）。
extern StreamBufferHandle_t g_imu_stream;
#define IMU_STREAM_SIZE 4096
extern vib_baseline_t g_baseline;
#ifdef __cplusplus
}
#endif

#endif // VIB_BASELINE_H
