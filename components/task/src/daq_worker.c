#include "daq_worker.h"
#include "drv_icm_42688_p.h"
#include "daq_icm_42688_p.h"
#include "drv_iis3dwb.h"
#include "daq_iis3dwb.h"
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

// 初始化配置
void config_icm_42688_p(daq_worker_param_t *param);
void config_iis3dwb(daq_worker_param_t *param);
esp_err_t start_patrolling_work();
esp_err_t start_diagnosing_work();

#define MAX_DAQ_SAMPLES 8192
#define ICM4_DMA_CHUNK_SIZE 128
#define IIS3_DMA_CHUNK_SIZE 512
#define DAQ_SKIP_MS 20U
#define DAQ_CAPTURE_GUARD_MS 100U

#define PATROL_FIXED_ODR_HZ 4000.0f
#define PATROL_FIXED_FFT_POINTS 4096U
#define PATROL_MAX_FREQ_HZ 1000.0f

#define DIAGNOSIS_FIXED_ODR_HZ 8000.0f
#define DIAGNOSIS_FIXED_FFT_POINTS 4096U
#define DIAGNOSIS_MIN_FREQ_HZ 2000.0f

static float s_vib_buffer[MAX_DAQ_SAMPLES * 3];

static uint32_t s_buffer_write_idx = 0;

static uint32_t get_capture_duration_ms(const DSP_Config_t *cfg)
{
    if (cfg == NULL || cfg->actual_odr <= 0.0f || cfg->fft_points == 0U)
    {
        return 0;
    }

    uint32_t duration_ms = (uint32_t)ceilf(cfg->actual_time * 1000.0f);
    duration_ms += DAQ_SKIP_MS;
    duration_ms += DAQ_CAPTURE_GUARD_MS;
    return duration_ms;
}

/**
 * @brief 内联辅助函数：将物理量存入平面化 Buffer (Planar Buffer)
 * 内存布局: [X0...Xn | Y0...Yn | Z0...Zn]
 */
static inline void daq_push_sample(float x, float y, float z)
{
    if (s_buffer_write_idx >= MAX_DAQ_SAMPLES)
        return;

    s_vib_buffer[s_buffer_write_idx] = x;
    s_vib_buffer[MAX_DAQ_SAMPLES + s_buffer_write_idx] = y;
    s_vib_buffer[MAX_DAQ_SAMPLES * 2 + s_buffer_write_idx] = z;

    s_buffer_write_idx++;
}

// ICM-42688-P 专用处理函数
static void daq_buffer_handler(const imu_raw_data_t *data, size_t count, void *ctx)
{
    (void)ctx;
    for (size_t i = 0; i < count; i++)
    {
        if (s_buffer_write_idx >= MAX_DAQ_SAMPLES)
        {
            break;
        }
        const float x_g = (float)data[i].x * LSB_TO_G_16;
        const float y_g = (float)data[i].y * LSB_TO_G_16;
        const float z_g = (float)data[i].z * LSB_TO_G_16;
        daq_push_sample(x_g, y_g, z_g);
    }
}
esp_err_t data_acquire_by_icm42688p(int8_t task_mode)
{
    uint32_t duration_ms = get_capture_duration_ms(&patrol_config);
    esp_err_t err = ESP_OK;
    if (task_mode == TASK_MODE_PATROLING)
    {
        duration_ms = get_capture_duration_ms(&patrol_config);
    }
    else
    {
        duration_ms = get_capture_duration_ms(&diagnosis_config);
    }
    if (duration_ms == 0)
        duration_ms = 1200;
     LOG_DEBUGF("duration(icm42688p): %lu ms, %s ", duration_ms, task_mode == TASK_MODE_PATROLING ? "Patrol" : "Diagnosis");
    err = daq_icm_42688_p_capture(
        &icm42688p_accel_fs_cfg_16, duration_ms, daq_buffer_handler, NULL, ICM4_DMA_CHUNK_SIZE, DAQ_SKIP_MS);
    return err;
}

esp_err_t data_acquire_by_iis3dwb(int8_t task_mode)
{
    uint32_t duration_ms = 0;
    if (task_mode == TASK_MODE_PATROLING)
    {
        duration_ms = get_capture_duration_ms(&patrol_config);
    }
    else
    {
        duration_ms = get_capture_duration_ms(&diagnosis_config);
    }

    if (duration_ms == 0)
        duration_ms = 1200;

    LOG_DEBUGF("duration(iis3dwb): %lu ms, %s ", duration_ms, task_mode == TASK_MODE_PATROLING ? "Patrol" : "Diagnosis");
    return daq_iis3dwb_capture(
        &iis3dwb_accel_fs_cfg_16, duration_ms, daq_buffer_handler, NULL, IIS3_DMA_CHUNK_SIZE, DAQ_SKIP_MS);
}

/**
 * @brief 启动巡逻工作
 *
 * 巡逻工作使用固定低频频谱模板
 * 模板配置：ODR=4000Hz, FFT点数=4096, 关注频段<=1000Hz
 *
 * @param rpm 设备转速 (RPM)
 * @return true 成功启动
 * @return false 启动失败
 */
esp_err_t start_patrolling_work()
{
    if (patrol_config.actual_odr <= 0.0f)
    {
        LOG_ERROR("Patrol config is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    s_buffer_write_idx = 0;
    esp_err_t ret = ESP_OK;

#if IMU == 1
    ret = data_acquire_by_iis3dwb(TASK_MODE_PATROLING);
#else
    ret = data_acquire_by_icm42688p(TASK_MODE_PATROLING);
#endif
    if (ret != ESP_OK)
    {
        LOG_ERRORF("Patrol capture failed: %d", ret);
        return ret;
    }
    if (s_buffer_write_idx < patrol_config.fft_points)
    {
        LOG_ERRORF("Patrol samples insufficient: %lu/%lu", s_buffer_write_idx, patrol_config.fft_points);
        return ESP_ERR_INVALID_SIZE;
    }
    LOG_DEBUGF("Patrol samples captured: %lu, using fixed FFT window=%lu",
               s_buffer_write_idx, patrol_config.fft_points);

    // 巡逻工作完成后，将数据发送给 RMS 任务进行分析
    vib_job_t job = {
        .raw_data = s_vib_buffer,
        .length = patrol_config.fft_points,
        .sample_rate = patrol_config.actual_odr,
        .task_mode = TASK_MODE_PATROLING};
    if (g_rms_job_queue)
    {
        xQueueSend(g_rms_job_queue, &job, 0);
    }

    s_buffer_write_idx = 0;
    return ESP_OK;
}

/**
 * @brief 启动诊断工作
 *
 * 诊断工作使用固定高频频谱模板
 * 模板配置：ODR=8000Hz, FFT点数=4096, 关注频段>=2000Hz
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
    esp_err_t ret = ESP_OK;
#if IMU == 1
    ret = data_acquire_by_iis3dwb(TASK_MODE_DIAGNOSIS);
#else
    ret = data_acquire_by_icm42688p(TASK_MODE_DIAGNOSIS);
#endif

    if (ret != ESP_OK)
    {
        LOG_ERRORF(" Diagnosis capture failed: %d", ret);
        return ret;
    }
    if (s_buffer_write_idx < diagnosis_config.fft_points)
    {
        LOG_ERRORF("Diagnosis samples insufficient: %lu/%lu", s_buffer_write_idx, diagnosis_config.fft_points);
        return ESP_ERR_INVALID_SIZE;
    }
    LOG_INFOF("Diagnosis samples captured: %lu, using fixed FFT window=%lu",
              s_buffer_write_idx, diagnosis_config.fft_points);

    vib_job_t job = {
        .raw_data = s_vib_buffer,
        .length = diagnosis_config.fft_points,
        .sample_rate = diagnosis_config.actual_odr,
        .task_mode = TASK_MODE_DIAGNOSIS};
    if (g_kurtosis_job_queue)
    {
        xQueueSend(g_kurtosis_job_queue, &job, 0);
    }
    return ESP_OK;
}

void config_icm_42688_p(daq_worker_param_t *param)
{
    if (param->task_mode == TASK_MODE_PATROLING)
    {
        if (patrol_config.actual_odr > 0.0f)
        {
            // LOG_INFO("Patrol config already initialized");
            return;
        }

        patrol_config = IMU_Get_Fixed_DSP_Config(
            &icm42688_driver,
            PATROL_FIXED_ODR_HZ,
            PATROL_FIXED_FFT_POINTS,
            PATROL_MAX_FREQ_HZ);
        // LOG_DEBUGF("Patrol fixed config initialized: ODR=%.2f Hz, Time=%.3f s, FFT Points=%u, Fmax=%.1f Hz",
        //            patrol_config.actual_odr, patrol_config.actual_time,
        //            patrol_config.fft_points, patrol_config.f_max_interest);
        return;
    }
    else if (param->task_mode == TASK_MODE_DIAGNOSIS)
    {
        if (diagnosis_config.actual_odr > 0.0f)
        {
            LOG_INFO(" Diagnosis config already initialized");
            return;
        }

        diagnosis_config = IMU_Get_Fixed_DSP_Config(
            &icm42688_driver,
            DIAGNOSIS_FIXED_ODR_HZ,
            DIAGNOSIS_FIXED_FFT_POINTS,
            DIAGNOSIS_MIN_FREQ_HZ);
        // LOG_DEBUGF("Diagnosis fixed config initialized: ODR=%.2f Hz, Time=%.3f s, FFT Points=%u, Fmax=%.1f Hz",
        //            diagnosis_config.actual_odr, diagnosis_config.actual_time,
        //            diagnosis_config.fft_points, diagnosis_config.f_max_interest);
    }
}

void config_iis3dwb(daq_worker_param_t *param)
{
    if (param->task_mode == TASK_MODE_PATROLING)
    {

        if (patrol_config.actual_odr > 0.0f)
        {
            // LOG_INFO("Patrol config already initialized");
            return;
        }
        patrol_config = IMU_Get_Fixed_DSP_Config(
            &iis3dwb_driver,
            PATROL_FIXED_ODR_HZ,
            PATROL_FIXED_FFT_POINTS,
            PATROL_MAX_FREQ_HZ);
    }
    else
    {
        if (diagnosis_config.actual_odr > 0.0f)
        {
            LOG_INFO(" Diagnosis config already initialized");
            return;
        }
        diagnosis_config = IMU_Get_Fixed_DSP_Config(
            &iis3dwb_driver,
            DIAGNOSIS_FIXED_ODR_HZ,
            DIAGNOSIS_FIXED_FFT_POINTS,
            DIAGNOSIS_MIN_FREQ_HZ);
    }
}

esp_err_t start_daq_worker(daq_worker_param_t *param)
{
    // 1. Check if the machine is in a stable state. This is a mandatory precondition.
    // if (get_machine_state() != STATE_STABLE) {
    //     LOG_WARN("Cannot start DAQ worker: machine state is not STABLE.");
    //     return ESP_ERR_INVALID_STATE;
    // }

    // 2. Lock the system task mutex to ensure exclusive operation during this DAQ work.
    // This will block the state check handler from running concurrently.
    lock_system_task();

    esp_err_t ret;

    if (param == NULL)
    {
        ret = ESP_ERR_INVALID_ARG;
    }
    else
    {

    // using the appropriate imu sensor
#if IMU == 1
        config_iis3dwb(param);
#else
        config_icm_42688_p(param);
#endif

        // 根据任务模式启动相应的工作
        if (param->task_mode == TASK_MODE_PATROLING)
        {
            // LOG_DEBUG("Starting patrol work...");
            ret = start_patrolling_work();
        }
        else if (param->task_mode == TASK_MODE_DIAGNOSIS)
        {
            // LOG_DEBUG("Starting diagnosis work...");
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
