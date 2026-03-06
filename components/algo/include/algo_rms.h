#ifndef ALGO_RMS_H
#define ALGO_RMS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 振动 RMS 结果结构体 (速度有效值)
 */
typedef struct {
    float x; // X轴速度 RMS (mm/s)
    float y; // Y轴速度 RMS (mm/s)
    float z; // Z轴速度 RMS (mm/s)
} vib_rms_t;

/**
 * @brief 计算三轴振动速度 RMS (Velocity RMS)
 * 
 * 处理流程:
 * 1. Scale: g -> mm/s^2
 * 2. HPF (10Hz): 去除重力分量 (DC)
 * 3. LPF (1000Hz): 限制带宽
 * 4. Integrate: 加速度 -> 速度
 * 5. HPF (10Hz): 去除积分漂移
 * 6. RMS: 计算有效值
 * 
 * 注意：该函数会为单轴申请临时堆内存 (scratch buffer)，计算完成后释放。
 * 
 * @param x 指向 X 轴加速度数据的指针 (单位: g, 平面化布局)
 * @param y 指向 Y 轴加速度数据的指针 (单位: g, 平面化布局)
 * @param z 指向 Z 轴加速度数据的指针 (单位: g, 平面化布局)
 * @param length 单轴数据点数
 * @param sample_rate 实际采样率 (Hz)
 * @return vib_rms_t 计算出的三轴速度 RMS 值 (mm/s)
 */
vib_rms_t algo_rms_calculate(const float *x, const float *y, const float *z, uint32_t length, float sample_rate);

#ifdef __cplusplus
}
#endif

#endif // ALGO_RMS_H
