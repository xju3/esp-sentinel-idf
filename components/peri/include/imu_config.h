 #ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 1. 数据结构定义                                                            */
/* ========================================================================== */

/**
 * @brief DSP 综合配置结果结构体
 * 供上层算法分配内存和设置 FFT 参数使用
 */
typedef struct {
    float    actual_odr;      // 传感器真实配置成功的采样率 (Hz)
    float    actual_time;     // 补齐 FFT 点数后的真实采样总时长 (秒)
    uint32_t fft_points;      // 计算出的最适合的 FFT 数据点数 (N，必然是 2 的幂)
    float    f_max_interest;  // 当前任务实际关注的最高物理频率 (Hz)
} DSP_Config_t;

/**
 * @brief 传感器底层驱动接口 (HAL)
 * 采用函数指针实现面向对象的多态，彻底解耦具体硬件
 */
typedef struct {
    char name[32]; // 传感器名称，如 "IIS3DWB" 或 "LIS2DW12"
    
    /**
     * @brief 硬件 ODR 配置回调函数
     * @param ideal_odr 算法层期望的最低理想采样率 (Hz)
     * @return float    传感器实际配置成功的采样率 (Hz)
     */
    float (*config_hardware_odr)(float ideal_odr);
    
} SensorDriver_t;


/* ========================================================================== */
/* 2. 核心 API 声明                                                           */
/* ========================================================================== */

/**
 * @brief 动态计算并应用传感器 DSP 采样配置
 * * @param sensor        底层传感器驱动实例指针
 * @param target_rpm    目标设备的运行转速 (RPM)
 * @param C             最小覆盖周期 (转数，建议 >= 1)
 * @param H             最大谐波阶数 (如低频设 10，高频包络可不参考此项)
 * @param F_env         包络最小高频载波起点 / 低频任务的最高关注频率 (Hz)
 * @param delta_f_max   最大允许的频率分辨率 (Hz，如 1.0)
 * @return DSP_Config_t 返回给上层算法的最终物理参数
 */
DSP_Config_t IMU_Calculate_DSP_Config(
    SensorDriver_t* sensor, 
    float target_rpm, 
    float C, 
    float H, 
    float F_env, 
    float delta_f_max
);

#ifdef __cplusplus
}
#endif

#endif // IMU_CONFIG_H