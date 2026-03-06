#include "imu_config.h"
#include <math.h>

DSP_Config_t IMU_Calculate_DSP_Config(
    SensorDriver_t* sensor, 
    float target_rpm,  // 目标设备的运行转速 (RPM)
    float C,  // 最小覆盖周期 (转数, 建议为10)
    float H,  // 最大谐波阶数 (如低频设10, 高频包络可不参考此项, 高频建议在40, 50)
    float F_env,  // 如果不需要包络分析, 低频可设置为1000hz, 若需要包络分析, 建议在 2000hz以上
    float delta_f_max // 最大允许的频率分辨率, 为了看清频谱，通常要求小于1.0Hz, 但过小会导致采样时间过长
)
{
    DSP_Config_t result = {0};
    
    // 防御性编程：防止除以 0
    if (target_rpm <= 0.0f || delta_f_max <= 0.0f || sensor == NULL || sensor->config_hardware_odr == NULL) {
        return result; 
    }

    // 1. 计算基频 (Hz)
    float f_r = target_rpm / 60.0f; 
    
    // 2. 确定最高关注频率 f_max_interest
    // 比较包络载波频率与 (谐波阶数 * 基频)，取最大值
    float max_harmonic_freq = H * f_r;
    result.f_max_interest = (F_env > max_harmonic_freq) ? F_env : max_harmonic_freq;
    
    // 3. 计算理想 ODR (满足奈奎斯特及 2.56 倍工程抗混叠原则)
    float ideal_odr = 2.56f * result.f_max_interest;
    
    // 4. 计算理想采样总时长 T_ideal
    // 必须同时满足“转够 C 圈的时间”和“达到 delta_f_max 分辨率的时间”
    float t_req_coverage = C / f_r;
    float t_req_resolution = 1.0f / delta_f_max;
    float ideal_time = (t_req_coverage > t_req_resolution) ? t_req_coverage : t_req_resolution;

    // 5. 【核心调用】向下分发，让底层硬件去匹配最接近的真实 ODR
    result.actual_odr = sensor->config_hardware_odr(ideal_odr);
    
    // 若底层返回 0，说明硬件配置失败
    if (result.actual_odr <= 0.0f) {
        return result; 
    }

    // 6. 根据硬件返回的真实 ODR，计算所需的数据点数
    float required_points = result.actual_odr * ideal_time;
    
    // 7. 将点数向上取整到最近的 2 的幂次方 (专为 FFT 蝶形算法优化)
    // 这是一个极其高效的按位操作算法，替代了缓慢的 log2 和 pow 函数
    uint32_t n = (uint32_t)ceilf(required_points);
    n--;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n++;
    result.fft_points = n;
    
    // 8. 反向推导最终真实的采样时长
    result.actual_time = (float)result.fft_points / result.actual_odr;

    return result;
}