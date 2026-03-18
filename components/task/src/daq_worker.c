#include "daq_worker.h"
#include "drv_icm_42688_p.h"
#include "daq_icm_42688_p.h"
#include "imu_config.h"
#include "logger.h"
#include "esp_err.h"
#include "task_rms.h"
#include "task_kurtosis.h"
#include <math.h>
#include "esp_attr.h"
#include "machine_state.h"

static DSP_Config_t patrol_config = {0};
static DSP_Config_t diagnosis_config = {0};

static int32_t current_rpm = 0; // 当前设备转速 (RPM)，由外部配置更新
// 初始化配置
void init_config(daq_worker_param_t *param);
esp_err_t start_patrolling_work();
esp_err_t start_diagnosing_work();

#define MAX_DAQ_SAMPLES 8192
#define DMA_CHUNK_SIZE 128
EXT_RAM_BSS_ATTR static float s_vib_buffer[MAX_DAQ_SAMPLES * 3];
static uint32_t s_buffer_write_idx = 0;

/**
 * @brief 内联辅助函数：将物理量存入平面化 Buffer (Planar Buffer)
 * 内存布局: [X0...Xn | Y0...Yn | Z0...Zn]
 */
static inline void daq_push_sample(float x, float y, float z)
{
    if (s_buffer_write_idx >= MAX_DAQ_SAMPLES) return;
    
    s_vib_buffer[s_buffer_write_idx] = x;
    s_vib_buffer[MAX_DAQ_SAMPLES + s_buffer_write_idx] = y;
    s_vib_buffer[MAX_DAQ_SAMPLES * 2 + s_buffer_write_idx] = z;
    
    s_buffer_write_idx++;
}

// ICM-42688-P 专用处理函数 (Big-Endian)
// 注意：这是一个临时方案，用于 PoV 阶段。
// 未来替换为 ST IIS3DWB 后，将与 LIS2DH12 一样使用 Little-Endian 处理逻辑，
// 届时可以考虑合并 Handler 或使用统一的 ST 处理模板。
static void daq_buffer_handler_icm(const imu_raw_data_t *data, size_t count, void *ctx)
{
    for (size_t i = 0; i < count; i++)
    {
        if (s_buffer_write_idx >= MAX_DAQ_SAMPLES)
        {
            break;
        }
        const float x_g = (int16_t)__builtin_bswap16((uint16_t)data[i].x) * LSB_TO_G_16G;
        const float y_g = (int16_t)__builtin_bswap16((uint16_t)data[i].y) * LSB_TO_G_16G;
        const float z_g = (int16_t)__builtin_bswap16((uint16_t)data[i].z) * LSB_TO_G_16G;
        daq_push_sample(x_g, y_g, z_g);
    }
}


/**
 * @brief 启动巡逻工作
 *
 * 巡逻工作只需要低频率数据
 * 参数配置：C=10, H=10, F_env=1000, delta_f_max=1
 *
 * @param rpm 设备转速 (RPM)
 * @return true 成功启动
 * @return false 启动失败
 */
esp_err_t start_patrolling_work()
{
    if (patrol_config.actual_odr <= 0.0f)
    {
        LOG_ERROR(" Patrol config is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    s_buffer_write_idx = 0; // 重置写入索引

    // 计算采集时长 (ms)
    uint32_t duration_ms = (uint32_t)(patrol_config.actual_time * 1000.0f);
    if (duration_ms == 0)
        duration_ms = 1000;

    icm_cfg_t capture_cfg = {.fs = ICM_FS_16G, .enable_wom = false, .wom_thr_mg = 0};
    LOG_DEBUGF("Sampling duration: %lu ms (Patrol)", duration_ms);
    esp_err_t ret = daq_icm_42688_p_capture(
        &capture_cfg, duration_ms, daq_buffer_handler_icm, NULL, DMA_CHUNK_SIZE, 20);

    if (ret != ESP_OK)
    {
        LOG_ERRORF("Patrol capture failed: %d", ret);
        return ret;
    }
    LOG_DEBUGF("Samples: %lu", s_buffer_write_idx * DMA_CHUNK_SIZE);

    // 巡逻工作完成后，将数据发送给 RMS 任务进行分析
    vib_job_t job = {
        .raw_data = s_vib_buffer,
        .length = s_buffer_write_idx,
        .sample_rate = patrol_config.actual_odr,
        .task_mode = TASK_MODE_PATROLING};
    if (g_rms_job_queue)
    {
        xQueueSend(g_rms_job_queue, &job, 0);
    }

    return ESP_OK;
}

/**
 * @brief 启动诊断工作
 *
 * 诊断需要高频数据
 * 参数配置：C=10, H=40, F_env=2000, delta_f_max=1
 *
 * @param rpm 设备转速 (RPM)
 * @return true 成功启动
 * @return false 启动失败
 */
esp_err_t start_diagnosing_work()
{
    if (diagnosis_config.actual_odr <= 0.0f)
    {
        LOG_ERROR(" Diagnosis config is not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    s_buffer_write_idx = 0; // 重置写入索引
    icm_cfg_t capture_cfg = {.fs = ICM_FS_16G, .enable_wom = false, .wom_thr_mg = 0};


    // 计算采集时长 (ms)
    uint32_t duration_ms = (uint32_t)(diagnosis_config.actual_time * 1000.0f);
    if (duration_ms == 0)
        duration_ms = 100;

    esp_err_t ret = daq_icm_42688_p_capture(
        &capture_cfg, duration_ms, daq_buffer_handler_icm, NULL, 512, 20);

    if (ret != ESP_OK)
    {
        LOG_ERRORF(" Diagnosis capture failed: %d", ret);
        return ret;
    }
    LOG_INFOF("Samples: %lu", s_buffer_write_idx);

    vib_job_t job = {
        .raw_data = s_vib_buffer,
        .length = s_buffer_write_idx,
        .sample_rate = diagnosis_config.actual_odr,
        .task_mode = TASK_MODE_DIAGNOSIS};
    if (g_rms_job_queue)
    {
        xQueueSend(g_kurtosis_job_queue, &job, 0);
    }

    return ESP_OK;
}

void init_config(daq_worker_param_t *param)
{
    if (param->task_mode == TASK_MODE_PATROLING)
    {
        if (current_rpm ==  param -> rpm)
        {
            LOG_INFO("Patrol config already initialized");
            return;
        } else {
            LOG_WARNF("RPM changed from %d to %d." , current_rpm, param->rpm);
        }

        current_rpm = param->rpm;
        patrol_config = IMU_Calculate_DSP_Config(
            &icm42688_driver,
            (float)param->rpm,
            10.0f,   // C
            10.0f,   // H
            1000.0f, // F_env
            1.0f     // delta_f_max
        );
        LOG_DEBUGF("config: ODR=%.2f Hz, Time=%.2f s, FFT Points=%u",
                  patrol_config.actual_odr, patrol_config.actual_time, patrol_config.fft_points);
        return;
    }
    else if (param->task_mode == TASK_MODE_DIAGNOSIS)
    {
        if (diagnosis_config.actual_odr > 0.0f)
        {
            LOG_INFO(" Diagnosis config already initialized");
            return;
        }

        diagnosis_config = IMU_Calculate_DSP_Config(
            &icm42688_driver,
            (float)param->rpm,
            10.0f,   // C
            40.0f,   // H
            2000.0f, // F_env
            1.0f     // delta_f_max
        );
        LOG_INFOF(" Diagnosis config initialized: ODR=%.2f Hz, Time=%.2f s, FFT Points=%u",
                  diagnosis_config.actual_odr, diagnosis_config.actual_time, diagnosis_config.fft_points);
    }
}


esp_err_t start_daq_worker(daq_worker_param_t *param)
{
    // 1. Check if the machine is in a stable state. This is a mandatory precondition.
    if (get_machine_state() != STATE_STABLE) {
        LOG_WARN("Cannot start DAQ worker: machine state is not STABLE.");
        return ESP_ERR_INVALID_STATE;
    }

    // 2. Lock the system task mutex to ensure exclusive operation during this DAQ work.
    // This will block the state check handler from running concurrently.
    lock_system_task();

    esp_err_t ret;

    if (param == NULL)
    {
        ret = ESP_ERR_INVALID_ARG;
    } else {
        // The original function body is now protected by the state check and mutex.
        init_config(param);

        // 根据任务模式启动相应的工作
        if (param->task_mode == TASK_MODE_PATROLING)
        {
            LOG_DEBUG("Starting patrol work...");
            ret = start_patrolling_work();
        }
        else if (param->task_mode == TASK_MODE_DIAGNOSIS)
        {
            LOG_DEBUG("Starting diagnosis work...");
            ret = start_diagnosing_work();
        }
        else
        {
            LOG_ERROR("Invalid task mode");
            ret = ESP_ERR_INVALID_ARG;
        }
    }

    // 3. Unlock the mutex now that the blocking DAQ work is complete.
    unlock_system_task();

    return ret;
}

// /**
//  * @brief 获取最近一次采集的振动数据 Buffer (用于测试/调试)
//  * @param length 输出参数，返回有效数据点的数量
//  * @return float* 指向 Buffer 的指针
//  */
// float *daq_get_vib_buffer(size_t *length)
// {
//     if (length)
//         *length = s_buffer_write_idx;
//     return s_vib_buffer;
// }
