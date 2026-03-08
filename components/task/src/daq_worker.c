#include "daq_worker.h"
#include "drv_icm_42688_p.h"
#include "daq_icm_42688_p.h"
#include "imu_config.h"
#include "logger.h"
#include "esp_err.h"
#include "task_rms.h"
#include <math.h>
#include "esp_attr.h"

static DSP_Config_t patrol_config = {0};
static DSP_Config_t diagnosis_config = {0};

// 初始化配置
void init_config(daq_worker_param_t *param);
esp_err_t start_patrolling_work();
esp_err_t start_diagnosing_work();

#define MAX_DAQ_SAMPLES 8192
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

// 通用数据处理回调：将数据解交错并存入静态 Buffer
static void daq_buffer_handler_icm(const imu_raw_data_t *data, size_t count, void *ctx);

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
        LOG_ERROR("[DAQ_WORKER] Patrol config is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    s_buffer_write_idx = 0; // 重置写入索引

    // 计算采集时长 (ms)
    uint32_t duration_ms = (uint32_t)(patrol_config.actual_time * 1000.0f);
    if (duration_ms == 0)
        duration_ms = 1000;

    // 方案调整：LIS2DH12 仅用于报警，数据采集统一使用 ICM-42688-P
    icm_cfg_t capture_cfg = {.fs = ICM_FS_16G, .enable_wom = false, .wom_thr_mg = 0};
    float lsb_to_g = LSB_TO_G_16G;

    LOG_INFOF("[DAQ_WORKER] Starting ICM-42688-P capture for %lu ms (Patrol)", duration_ms);
    esp_err_t ret = daq_icm_42688_p_capture(
        &capture_cfg, duration_ms, daq_buffer_handler_icm, &lsb_to_g, 512, 20);

    if (ret != ESP_OK)
    {
        LOG_ERRORF("[DAQ_WORKER] Patrol capture failed: %d", ret);
        return ret;
    }
    LOG_INFOF("[DAQ_WORKER] Patrol complete. Samples: %lu", s_buffer_write_idx);

    // TODO: Add job to queue for processing patrol data

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
        LOG_ERROR("[DAQ_WORKER] Diagnosis config is not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    s_buffer_write_idx = 0; // 重置写入索引
    icm_cfg_t capture_cfg = {.fs = ICM_FS_16G, .enable_wom = false, .wom_thr_mg = 0};

    // 根据配置的量程选择对应的 LSB 转换系数
    float lsb_to_g = LSB_TO_G_16G;
    switch (capture_cfg.fs) {
        case ICM_FS_2G:  lsb_to_g = LSB_TO_G_2G; break;
        case ICM_FS_4G:  lsb_to_g = LSB_TO_G_4G; break;
        case ICM_FS_8G:  lsb_to_g = LSB_TO_G_8G; break;
        case ICM_FS_16G: lsb_to_g = LSB_TO_G_16G; break;
    }

    // 计算采集时长 (ms)
    uint32_t duration_ms = (uint32_t)(diagnosis_config.actual_time * 1000.0f);
    if (duration_ms == 0)
        duration_ms = 100;

    esp_err_t ret = daq_icm_42688_p_capture(
        &capture_cfg, duration_ms, daq_buffer_handler_icm, &lsb_to_g, 512, 20);

    if (ret != ESP_OK)
    {
        LOG_ERRORF("[DAQ_WORKER] Diagnosis capture failed: %d", ret);
        return ret;
    }
    LOG_INFOF("[DAQ_WORKER] Diagnosis complete. Samples: %lu", s_buffer_write_idx);

    vib_job_t job = {
        .raw_data = s_vib_buffer,
        .length = s_buffer_write_idx,
        .sample_rate = diagnosis_config.actual_odr,
        .task_mode = TASK_MODE_DIAGNOSIS};
    if (g_rms_job_queue)
    {
        xQueueSend(g_rms_job_queue, &job, 0);
    }

    return ESP_OK;
}

void init_config(daq_worker_param_t *param)
{
    if (param->task_mode == TASK_MODE_PATROLING)
    {
        // 这意味着如果在一次系统运行中，
        //  先用 1500 RPM 跑了一次测试，
        // 然后想改用 3000 RPM 再跑一次测试，配置不会更新，
        // 仍然会沿用 1500 RPM 的参数。在编写单元测试时，
        // 如果涉及多次不同参数的调用，可能需要增加一个 "重置配置" 的接口。
        if (patrol_config.actual_odr > 0.0f)
        {
            LOG_INFO("[DAQ_WORKER] Patrol config already initialized");
            return;
        }
        // Use ICM driver for patrolling (LIS2DH12 is alarm only)
        patrol_config = IMU_Calculate_DSP_Config(
            &icm42688_driver,
            (float)param->rpm,
            10.0f,   // C
            10.0f,   // H
            1000.0f, // F_env
            1.0f     // delta_f_max
        );
        LOG_INFOF("[DAQ_WORKER] Patrol config initialized: ODR=%.2f Hz, Time=%.2f s, FFT Points=%u",
                  patrol_config.actual_odr, patrol_config.actual_time, patrol_config.fft_points);
        return;
    }
    else if (param->task_mode == TASK_MODE_DIAGNOSIS)
    {
        if (diagnosis_config.actual_odr > 0.0f)
        {
            LOG_INFO("[DAQ_WORKER] Diagnosis config already initialized");
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
        LOG_INFOF("[DAQ_WORKER] Diagnosis config initialized: ODR=%.2f Hz, Time=%.2f s, FFT Points=%u",
                  diagnosis_config.actual_odr, diagnosis_config.actual_time, diagnosis_config.fft_points);
    }
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

esp_err_t start_daq_worker(daq_worker_param_t *param)
{
    if (param == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    // 初始化配置
    init_config(param);

    // 根据任务模式启动相应的工作
    if (param->task_mode == TASK_MODE_PATROLING)
    {
        LOG_INFO("[DAQ_WORKER] Starting patrol work...");
        return start_patrolling_work();
    }
    else if (param->task_mode == TASK_MODE_DIAGNOSIS)
    {
        LOG_INFO("[DAQ_WORKER] Starting diagnosis work...");
        return start_diagnosing_work();
    }
    else
    {
        LOG_ERROR("[DAQ_WORKER] Invalid task mode");
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief 获取最近一次采集的振动数据 Buffer (用于测试/调试)
 * @param length 输出参数，返回有效数据点的数量
 * @return float* 指向 Buffer 的指针
 */
float *daq_get_vib_buffer(size_t *length)
{
    if (length)
        *length = s_buffer_write_idx;
    return s_vib_buffer;
}
