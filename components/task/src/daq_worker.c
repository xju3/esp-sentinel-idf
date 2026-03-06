#include "daq_worker.h"
#include "drv_icm_42688_p.h"
#include "daq_icm_42688_p.h"
#include "imu_config.h"
#include "logger.h"
#include "esp_err.h"
#include "task_rms.h"
#include <math.h>
#include "esp_attr.h"

#define LSB_TO_G (1.0f / 2048.0f) // Assuming 16G FS: 32768/16 = 2048

static DSP_Config_t patrol_config = {0};    
static DSP_Config_t diagnosis_config = {0};    

// 初始化配置
void init_config(daq_worker_param_t *param);    
esp_err_t start_patrolling_work();
esp_err_t start_diagnosing_work();

#define MAX_DAQ_SAMPLES 8192
EXT_RAM_ATTR static float s_vib_buffer[MAX_DAQ_SAMPLES * 3];
static uint32_t s_buffer_write_idx = 0;

// 通用数据处理回调：将数据解交错并存入静态 Buffer
static void daq_buffer_handler(const imu_raw_data_t *data, size_t count, void *ctx);

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
    icm_cfg_t capture_cfg = { .fs = ICM_FS_16G, .enable_wom = false, .wom_thr_mg = 0 };
    
    // 计算采集时长 (ms)
    uint32_t duration_ms = (uint32_t)(patrol_config.actual_time * 1000.0f);
    if (duration_ms == 0) duration_ms = 1000;

    esp_err_t ret = daq_icm_42688_p_capture(
        &capture_cfg, duration_ms, daq_buffer_handler, NULL, 256, 20
    );

    if (ret != ESP_OK) {
        LOG_ERRORF("[DAQ_WORKER] Patrol capture failed: %d", ret);
        return ret;
    }
    LOG_INFOF("[DAQ_WORKER] Patrol complete. Samples: %lu", s_buffer_write_idx);
    
    vib_job_t job = {
        .raw_data = s_vib_buffer,
        .length = s_buffer_write_idx,
        .sample_rate = patrol_config.actual_odr,
        .task_mode = TASK_MODE_PATROLING
    };
    if (g_rms_job_queue) {
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
        LOG_ERROR("[DAQ_WORKER] Diagnosis config is not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    s_buffer_write_idx = 0; // 重置写入索引
    icm_cfg_t capture_cfg = { .fs = ICM_FS_16G, .enable_wom = false, .wom_thr_mg = 0 };

    // 计算采集时长 (ms)
    uint32_t duration_ms = (uint32_t)(diagnosis_config.actual_time * 1000.0f);
    if (duration_ms == 0) duration_ms = 100;

    esp_err_t ret = daq_icm_42688_p_capture(
        &capture_cfg, duration_ms, daq_buffer_handler, NULL, 512, 20
    );

    if (ret != ESP_OK) {
        LOG_ERRORF("[DAQ_WORKER] Diagnosis capture failed: %d", ret);
        return ret;
    }
    LOG_INFOF("[DAQ_WORKER] Diagnosis complete. Samples: %lu", s_buffer_write_idx);
    
    vib_job_t job = {
        .raw_data = s_vib_buffer,
        .length = s_buffer_write_idx,
        .sample_rate = diagnosis_config.actual_odr,
        .task_mode = TASK_MODE_DIAGNOSIS
    };
    if (g_rms_job_queue) {
        xQueueSend(g_rms_job_queue, &job, 0);
    }
    
    return ESP_OK;
}


void init_config(daq_worker_param_t *param)
{
    if (param->task_mode == TASK_MODE_PATROLING)
    {
        if (patrol_config.actual_odr > 0.0f) {
            LOG_INFO("[DAQ_WORKER] Patrol config already initialized");
            return;
        }
        patrol_config = IMU_Calculate_DSP_Config(
            &icm42688_driver,
            (float)param->rpm,
            10.0f,   // C
            10.0f,   // H
            1000.0f, // F_env
            1.0f     // delta_f_max
        );
        return;
    }
    else if (param->task_mode == TASK_MODE_DIAGNOSIS)
    {
        if (diagnosis_config.actual_odr > 0.0f) {
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
    }
}

static void daq_buffer_handler(const imu_raw_data_t *data, size_t count, void *ctx)
{
    float *x_plane = &s_vib_buffer[0];
    float *y_plane = &s_vib_buffer[MAX_DAQ_SAMPLES];
    float *z_plane = &s_vib_buffer[MAX_DAQ_SAMPLES * 2];

    for (size_t i = 0; i < count; i++)
    {
        if (s_buffer_write_idx >= MAX_DAQ_SAMPLES)
        {
            break;
        }

        const float x_g = (int16_t)__builtin_bswap16((uint16_t)data[i].x) * LSB_TO_G;
        const float y_g = (int16_t)__builtin_bswap16((uint16_t)data[i].y) * LSB_TO_G;
        const float z_g = (int16_t)__builtin_bswap16((uint16_t)data[i].z) * LSB_TO_G;
        
        x_plane[s_buffer_write_idx] = x_g;
        y_plane[s_buffer_write_idx] = y_g;
        z_plane[s_buffer_write_idx] = z_g;
        
        s_buffer_write_idx++;
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
