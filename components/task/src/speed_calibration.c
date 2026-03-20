#include "speed_calibration.h"
#include "daq_icm_42688_p.h"
#include "algo_rpm_calibration.h"
#include "logger.h"
#include "task_stash.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include <string.h>

// === 配置参数 ===
#define CALIB_ODR_HZ        200.0f      // 采样率 200Hz
#define CALIB_FFT_SIZE      2048        // FFT 点数

// 静态 Buffer 索引
static uint32_t s_calib_write_idx = 0;

// 使用 SPIRAM 分配大块内存，避免占用宝贵的内部 RAM
// 内存布局: [X0...Xn | Y0...Yn | Z0...Zn]
static float *s_calib_raw_buf = NULL;

/**
 * @brief 数据采集回调 (由 DAQ 任务调用)
 */
static void calib_data_handler(const imu_raw_data_t *data, size_t count, void *ctx)
{
    if (!s_calib_raw_buf) return;

    for (size_t i = 0; i < count; i++) {
        if (s_calib_write_idx >= CALIB_FFT_SIZE) break;

        // 转换为物理量 (g)
        float x = (int16_t)__builtin_bswap16((uint16_t)data[i].x) * LSB_TO_G_16G;
        float y = (int16_t)__builtin_bswap16((uint16_t)data[i].y) * LSB_TO_G_16G;
        float z = (int16_t)__builtin_bswap16((uint16_t)data[i].z) * LSB_TO_G_16G;

        // 存入平面化 Buffer
        s_calib_raw_buf[s_calib_write_idx] = x;
        s_calib_raw_buf[CALIB_FFT_SIZE + s_calib_write_idx] = y;
        s_calib_raw_buf[CALIB_FFT_SIZE * 2 + s_calib_write_idx] = z;

        s_calib_write_idx++;
    }
}

esp_err_t speed_calibration_start(int32_t expected_rpm, int32_t *out_rpm)
{
    if (!out_rpm) return ESP_ERR_INVALID_ARG;
    
    esp_err_t ret = ESP_OK;
    s_calib_write_idx = 0;

    // 1. 内存分配 (Raw Data: 3轴 * 2048 * 4字节 = 24KB)
    s_calib_raw_buf = (float *)heap_caps_malloc(3 * CALIB_FFT_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!s_calib_raw_buf) {
        LOG_ERROR("Failed to allocate memory for speed calibration");
        return ESP_ERR_NO_MEM;
    }

    // 2. 启动采集
    // 计算需要的时长 (ms) = (2048 / 200) * 1000 = 10240 ms
    uint32_t duration_ms = (uint32_t)((CALIB_FFT_SIZE / CALIB_ODR_HZ) * 1000.0f) + 200; // +200ms 冗余
    
    LOG_INFOF("Starting RPM calibration... Duration: %lu ms", duration_ms);

    // 配置 IMU: 16G 量程 (低频关注大振幅), 禁用 WoM
    icm_cfg_t cfg = {.fs = ICM_FS_16G, .enable_wom = false, .wom_thr_mg = 0};
    
    // 调用 DAQ 模块进行采集 (阻塞式)
    // 注意：这里 chunk_size 设为 128，skip_ms 设为 100 以跳过启动瞬态
    ret = daq_icm_42688_p_capture(&cfg, duration_ms, calib_data_handler, NULL, 128, 100);

    if (ret != ESP_OK) {
        LOG_ERRORF("Calibration capture failed: %d", ret);
        goto cleanup;
    }

    if (s_calib_write_idx < CALIB_FFT_SIZE) {
        LOG_WARNF("Insufficient samples captured: %lu/%d", s_calib_write_idx, CALIB_FFT_SIZE);
        ret = ESP_FAIL;
        goto cleanup;
    }

    // 3. 调用算法模块计算转速
    // 提取三轴数据指针
    const float *x_data = s_calib_raw_buf;
    const float *y_data = s_calib_raw_buf + CALIB_FFT_SIZE;
    const float *z_data = s_calib_raw_buf + CALIB_FFT_SIZE * 2;

    // 创建配置（使用默认值）
    rpm_calib_config_t algo_cfg = algo_rpm_calib_get_default_config();

    // 调用算法计算转速
    ret = algo_rpm_calibration_auto_axis(x_data, y_data, z_data, expected_rpm, &algo_cfg, out_rpm);

cleanup:
    if (s_calib_raw_buf) heap_caps_free(s_calib_raw_buf);
    s_calib_raw_buf = NULL;

    return ret;
}