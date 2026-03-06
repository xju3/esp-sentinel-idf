#include "daq_worker.h"
#include "imu_config.h"
#include "logger.h"
#include <math.h>

// 假设的传感器驱动实例（实际项目中需要从其他地方获取）
static SensorDriver_t* s_sensor_driver = NULL;

/**
 * @brief 设置传感器驱动实例
 * 
 * @param sensor 传感器驱动指针
 */
void daq_worker_set_sensor_driver(SensorDriver_t* sensor)
{
    s_sensor_driver = sensor;
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
bool start_patrolling_work(int32_t rpm)
{
    if (s_sensor_driver == NULL)
    {
        LOG_ERROR("[DAQ_WORKER] Sensor driver not set");
        return false;
    }
    
    if (rpm <= 0)
    {
        LOG_ERRORF("[DAQ_WORKER] Invalid RPM for patrolling: %d", rpm);
        return false;
    }
    
    LOG_INFOF("[DAQ_WORKER] Starting patrolling work with RPM: %d", rpm);
    
    // 巡逻工作参数配置
    float C = 10.0f;           // 最小覆盖周期 (转数)
    float H = 10.0f;           // 最大谐波阶数
    float F_env = 1000.0f;     // 包络最小高频载波起点 (Hz)
    float delta_f_max = 1.0f;  // 最大允许的频率分辨率 (Hz)
    
    // 计算DSP配置
    DSP_Config_t config = IMU_Calculate_DSP_Config(
        s_sensor_driver,
        (float)rpm,
        C,
        H,
        F_env,
        delta_f_max
    );
    
    // 检查配置是否有效
    if (config.actual_odr <= 0.0f || config.fft_points == 0)
    {
        LOG_ERROR("[DAQ_WORKER] Failed to calculate DSP config for patrolling");
        return false;
    }
    
    LOG_INFOF("[DAQ_WORKER] Patrolling DSP config: ODR=%.1f Hz, FFT points=%u, Time=%.3f s, F_max=%.1f Hz",
              config.actual_odr, config.fft_points, config.actual_time, config.f_max_interest);
    
    // 这里可以添加实际的巡逻工作逻辑，例如：
    // 1. 配置传感器采样率
    // 2. 启动数据采集
    // 3. 执行Welford算法和RMS检查
    // 4. 检查机械状态（转子不平衡，固定螺丝松动等）
    
    LOG_INFO("[DAQ_WORKER] Patrolling work started successfully");
    return true;
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
bool start_diagnosing_work(int32_t rpm)
{
    if (s_sensor_driver == NULL)
    {
        LOG_ERROR("[DAQ_WORKER] Sensor driver not set");
        return false;
    }
    
    if (rpm <= 0)
    {
        LOG_ERRORF("[DAQ_WORKER] Invalid RPM for diagnosing: %d", rpm);
        return false;
    }
    
    LOG_INFOF("[DAQ_WORKER] Starting diagnosing work with RPM: %d", rpm);
    
    // 诊断工作参数配置
    float C = 10.0f;           // 最小覆盖周期 (转数)
    float H = 40.0f;           // 最大谐波阶数
    float F_env = 2000.0f;     // 包络最小高频载波起点 (Hz)
    float delta_f_max = 1.0f;  // 最大允许的频率分辨率 (Hz)
    
    // 计算DSP配置
    DSP_Config_t config = IMU_Calculate_DSP_Config(
        s_sensor_driver,
        (float)rpm,
        C,
        H,
        F_env,
        delta_f_max
    );
    
    // 检查配置是否有效
    if (config.actual_odr <= 0.0f || config.fft_points == 0)
    {
        LOG_ERROR("[DAQ_WORKER] Failed to calculate DSP config for diagnosing");
        return false;
    }
    
    LOG_INFOF("[DAQ_WORKER] Diagnosing DSP config: ODR=%.1f Hz, FFT points=%u, Time=%.3f s, F_max=%.1f Hz",
              config.actual_odr, config.fft_points, config.actual_time, config.f_max_interest);
    
    // 这里可以添加实际的诊断工作逻辑，例如：
    // 1. 配置传感器采样率
    // 2. 启动高频数据采集
    // 3. 执行峭度、包络解调、FFT分析
    // 4. 检测内部的轴承、叶轮异常
    
    LOG_INFO("[DAQ_WORKER] Diagnosing work started successfully");
    return true;
}