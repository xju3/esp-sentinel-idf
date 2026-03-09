#include "speed_calibration.h"
#include "daq_icm_42688_p.h"
#include "algo_fft.h"
#include "logger.h"
#include "task_stash.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include <math.h>
#include <string.h>

// === 配置参数 ===
#define CALIB_ODR_HZ        200.0f      // 采样率 200Hz (覆盖 0-100Hz, 即 0-6000 RPM)
#define CALIB_FFT_SIZE      2048        // FFT 点数 (分辨率 ~0.1Hz -> ~6 RPM)
#define CALIB_MIN_RPM       600.0f      // 最小关注转速 (10 Hz)
#define CALIB_MAX_RPM       6000.0f     // 最大关注转速 (100 Hz)
#define CALIB_MIN_MAG_THR   0.005f      // 最小幅值阈值 (g)，低于此值认为设备未运行

// 静态 Buffer 索引
static uint32_t s_calib_write_idx = 0;

// 使用 SPIRAM 分配大块内存，避免占用宝贵的内部 RAM
// 内存布局: [X0...Xn | Y0...Yn | Z0...Zn]
static float *s_calib_raw_buf = NULL;
static float *s_calib_fft_mag = NULL;

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

esp_err_t speed_calibration_start(int32_t *out_rpm)
{
    if (!out_rpm) return ESP_ERR_INVALID_ARG;
    
    esp_err_t ret = ESP_OK;
    s_calib_write_idx = 0;

    // 1. 内存分配 (Raw Data: 3轴 * 2048 * 4字节 = 24KB)
    s_calib_raw_buf = (float *)heap_caps_malloc(3 * CALIB_FFT_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    // FFT Magnitude Buffer (复用一个 buffer 轮流计算三轴)
    s_calib_fft_mag = (float *)heap_caps_malloc((CALIB_FFT_SIZE / 2) * sizeof(float), MALLOC_CAP_SPIRAM);

    if (!s_calib_raw_buf || !s_calib_fft_mag) {
        LOG_ERROR("Failed to allocate memory for speed calibration");
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
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
        // 如果数据不够，可以补零或报错，这里选择报错以保证精度
        ret = ESP_FAIL;
        goto cleanup;
    }

    // 3. 信号处理与 FFT
    // 我们需要找到三轴中能量最大的那个频率点。
    // 策略：分别计算 X, Y, Z 的 FFT 幅值谱，然后将它们叠加，在叠加谱上寻峰。
    
    // 分配叠加谱 Buffer (Spectrum Sum)
    float *spectrum_sum = (float *)heap_caps_calloc((CALIB_FFT_SIZE / 2), sizeof(float), MALLOC_CAP_SPIRAM);
    if (!spectrum_sum) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    const float *ptrs[3] = {
        s_calib_raw_buf,
        s_calib_raw_buf + CALIB_FFT_SIZE,
        s_calib_raw_buf + CALIB_FFT_SIZE * 2
    };

    for (int axis = 0; axis < 3; axis++) {
        // 去直流 (Detrend/Mean Removal) - 简单的去均值
        float mean = 0.0f;
        for (int i = 0; i < CALIB_FFT_SIZE; i++) mean += ptrs[axis][i];
        mean /= CALIB_FFT_SIZE;
        
        // 原地去均值 (注意：algo_fft 内部会加窗，所以这里只需去直流)
        // 由于 algo_fft_calculate 输入是 const，我们需要拷贝到临时 buffer 或者修改 algo_fft 接口
        // 这里 algo_fft_calculate 会申请自己的 scratch，但输入必须是纯净的。
        // 为了简单，我们直接修改 s_calib_raw_buf 中的数据（虽然指针是 const，但内存是我们分配的）
        float *mutable_ptr = (float *)ptrs[axis];
        for (int i = 0; i < CALIB_FFT_SIZE; i++) mutable_ptr[i] -= mean;

        // 计算 FFT
        ret = algo_fft_calculate(ptrs[axis], s_calib_fft_mag, CALIB_FFT_SIZE);
        if (ret != ESP_OK) {
            LOG_ERRORF("FFT failed on axis %d", axis);
            heap_caps_free(spectrum_sum);
            goto cleanup;
        }

        // 累加到总谱
        for (int i = 0; i < CALIB_FFT_SIZE / 2; i++) {
            spectrum_sum[i] += s_calib_fft_mag[i];
        }
    }

    // 4. 寻峰 (Peak Picking)
    float bin_res = CALIB_ODR_HZ / (float)CALIB_FFT_SIZE;
    int min_bin = (int)(CALIB_MIN_RPM / 60.0f / bin_res);
    int max_bin = (int)(CALIB_MAX_RPM / 60.0f / bin_res);
    
    if (max_bin >= CALIB_FFT_SIZE / 2) max_bin = CALIB_FFT_SIZE / 2 - 1;

    float max_val = 0.0f;
    int peak_idx = 0;

    for (int i = min_bin; i <= max_bin; i++) {
        if (spectrum_sum[i] > max_val) {
            max_val = spectrum_sum[i];
            peak_idx = i;
        }
    }

    heap_caps_free(spectrum_sum);

    // 5. 结果验证
    // 检查幅值是否足够大 (避免在静止时标定出噪声频率)
    // spectrum_sum 是三轴叠加，所以阈值要适当放宽或除以3
    if (max_val < (CALIB_MIN_MAG_THR * 3.0f)) {
        LOG_WARNF("Vibration too low for calibration (Max: %.4fg)", max_val);
        ret = ESP_FAIL;
    } else {
        float freq = peak_idx * bin_res;
        *out_rpm = (int32_t)(freq * 60.0f);
        LOG_INFOF("Calibration Success: Peak=%.2f Hz, RPM=%ld, Mag=%.4fg", freq, *out_rpm, max_val);
    }

cleanup:
    if (s_calib_raw_buf) heap_caps_free(s_calib_raw_buf);
    if (s_calib_fft_mag) heap_caps_free(s_calib_fft_mag);
    s_calib_raw_buf = NULL;
    s_calib_fft_mag = NULL;

    return ret;
}