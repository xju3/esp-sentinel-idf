/* 
 * algo_math.h - 硬件隔离抽象层（Algorithm HAL）
 * 
 * 核心解耦层：隔离ESP32硬件加速与PC纯C实现
 * 使用 #ifdef ESP_PLATFORM 路由到 esp-dsp 或标准数学库
 * 
 * 设计原则：
 * 1. 统一接口：提供一致的数学函数接口
 * 2. 条件编译：ESP32平台使用硬件加速，PC平台使用纯C
 * 3. 零外部依赖：不直接包含 esp-dsp 头文件
 * 4. 占位符设计：PC端可接入KissFFT等第三方库
 */

#ifndef ALGO_MATH_H
#define ALGO_MATH_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= *
 * 1. 基本数学运算（硬件加速路由）
 * ========================================================================= */

/**
 * @brief 快速平方根（硬件加速）
 * @param x 输入值
 * @return 平方根结果
 */
float algo_math_sqrt(float x);

/**
 * @brief 快速正弦函数（硬件加速）
 * @param x 输入角度（弧度）
 * @return 正弦值
 */
float algo_math_sin(float x);

/**
 * @brief 快速余弦函数（硬件加速）
 * @param x 输入角度（弧度）
 * @return 余弦值
 */
float algo_math_cos(float x);

/**
 * @brief 快速反正切函数（硬件加速）
 * @param y 纵坐标
 * @param x 横坐标
 * @return 角度（弧度）
 */
float algo_math_atan2(float y, float x);

/**
 * @brief 快速指数函数（硬件加速）
 * @param x 指数
 * @return e^x
 */
float algo_math_exp(float x);

/**
 * @brief 快速自然对数（硬件加速）
 * @param x 输入值
 * @return ln(x)
 */
float algo_math_log(float x);

/* ========================================================================= *
 * 2. 向量和矩阵运算（SIMD加速）
 * ========================================================================= */

/**
 * @brief 向量点积（SIMD加速）
 * @param a 向量A
 * @param b 向量B
 * @param len 向量长度
 * @return 点积结果
 */
float algo_math_dot_product(const float *a, const float *b, size_t len);

/**
 * @brief 向量加法（SIMD加速）
 * @param dst 目标向量（输出）
 * @param src 源向量
 * @param len 向量长度
 */
void algo_math_vector_add(float *dst, const float *src, size_t len);

/**
 * @brief 向量减法（SIMD加速）
 * @param dst 目标向量（输出）
 * @param src 源向量
 * @param len 向量长度
 */
void algo_math_vector_sub(float *dst, const float *src, size_t len);

/**
 * @brief 向量乘法（SIMD加速）
 * @param dst 目标向量（输出）
 * @param src 源向量
 * @param len 向量长度
 */
void algo_math_vector_mul(float *dst, const float *src, size_t len);

/**
 * @brief 向量缩放（SIMD加速）
 * @param data 输入/输出向量
 * @param scale 缩放因子
 * @param len 向量长度
 */
void algo_math_vector_scale(float *data, float scale, size_t len);

/**
 * @brief 计算向量绝对值（SIMD加速）
 * @param dst 目标向量（输出）
 * @param src 源向量
 * @param len 向量长度
 */
void algo_math_vector_abs(float *dst, const float *src, size_t len);

/**
 * @brief 计算向量平方（SIMD加速）
 * @param dst 目标向量（输出）
 * @param src 源向量
 * @param len 向量长度
 */
void algo_math_vector_square(float *dst, const float *src, size_t len);

/* ========================================================================= *
 * 3. 滤波器运算（硬件加速）
 * ========================================================================= */

/**
 * @brief 双二阶滤波器结构（硬件加速）
 */
typedef struct {
    float coeffs[5];    // b0, b1, b2, a1, a2
    float delay[2];     // 延迟线
} algo_biquad_filter_t;

/**
 * @brief 初始化双二阶滤波器
 * @param filter 滤波器结构
 * @param coeffs 系数数组 [b0, b1, b2, a1, a2]
 */
void algo_math_biquad_init(algo_biquad_filter_t *filter, const float *coeffs);

/**
 * @brief 处理单个样本通过双二阶滤波器
 * @param filter 滤波器结构
 * @param input 输入样本
 * @return 滤波后的输出样本
 */
float algo_math_biquad_process(algo_biquad_filter_t *filter, float input);

/**
 * @brief 处理数组通过双二阶滤波器（硬件加速）
 * @param filter 滤波器结构
 * @param data 输入/输出数组
 * @param len 数组长度
 */
void algo_math_biquad_process_array(algo_biquad_filter_t *filter, float *data, size_t len);

/**
 * @brief 计算高通双二阶滤波器系数
 * @param coeffs 输出系数数组 [b0, b1, b2, a1, a2]
 * @param fs 采样频率（Hz）
 * @param fc 截止频率（Hz）
 * @param Q 品质因数（默认0.7071为巴特沃斯）
 */
void algo_math_biquad_calc_hp(float *coeffs, float fs, float fc, float Q);

/**
 * @brief 计算低通双二阶滤波器系数
 * @param coeffs 输出系数数组 [b0, b1, b2, a1, a2]
 * @param fs 采样频率（Hz）
 * @param fc 截止频率（Hz）
 * @param Q 品质因数（默认0.7071为巴特沃斯）
 */
void algo_math_biquad_calc_lp(float *coeffs, float fs, float fc, float Q);

/* ========================================================================= *
 * 4. FFT运算（硬件加速路由）
 * ========================================================================= */

/**
 * @brief FFT引擎状态（不透明指针）
 */
typedef void* algo_fft_engine_t;

/**
 * @brief 初始化FFT引擎（硬件加速）
 * @param max_fft_size 最大FFT大小（必须是2的幂）
 * @return FFT引擎句柄，NULL表示失败
 */
algo_fft_engine_t algo_math_fft_init(size_t max_fft_size);

/**
 * @brief 执行实数FFT（硬件加速）
 * @param engine FFT引擎句柄
 * @param input 实数输入信号（长度N）
 * @param n FFT大小（必须是2的幂）
 * @param work_buf 工作缓冲区（长度2*N float）
 * @param output 复数输出（长度N/2+1复数，交错存储实部虚部）
 */
void algo_math_fft_execute_real(algo_fft_engine_t engine,
                                const float *input,
                                size_t n,
                                float *work_buf,
                                float *output);

/**
 * @brief 计算幅度谱（硬件加速）
 * @param complex_fft 复数FFT结果（长度N/2+1复数）
 * @param magnitude 幅度谱输出（长度N/2 float）
 * @param n_fft FFT大小
 */
void algo_math_fft_compute_magnitude(const float *complex_fft,
                                     float *magnitude,
                                     size_t n_fft);

/**
 * @brief 清理FFT引擎资源
 * @param engine FFT引擎句柄
 */
void algo_math_fft_cleanup(algo_fft_engine_t engine);

/* ========================================================================= *
 * 5. 统计运算（硬件加速）
 * ========================================================================= */

/**
 * @brief 计算向量均值（SIMD加速）
 * @param data 输入向量
 * @param len 向量长度
 * @return 均值
 */
float algo_math_vector_mean(const float *data, size_t len);

/**
 * @brief 计算向量方差（SIMD加速）
 * @param data 输入向量
 * @param len 向量长度
 * @param mean 预计算的均值
 * @return 方差
 */
float algo_math_vector_variance(const float *data, size_t len, float mean);

/**
 * @brief 计算向量标准差（SIMD加速）
 * @param data 输入向量
 * @param len 向量长度
 * @param mean 预计算的均值
 * @return 标准差
 */
float algo_math_vector_std_dev(const float *data, size_t len, float mean);

/**
 * @brief 计算向量RMS（SIMD加速）
 * @param data 输入向量
 * @param len 向量长度
 * @return RMS值
 */
float algo_math_vector_rms(const float *data, size_t len);

/* ========================================================================= *
 * 6. 平台检测和配置
 * ========================================================================= */

/**
 * @brief 检测当前平台是否支持硬件加速
 * @return true如果支持硬件加速，false如果使用纯C实现
 */
bool algo_math_has_hardware_acceleration(void);

/**
 * @brief 获取平台信息字符串
 * @return 平台描述字符串
 */
const char* algo_math_get_platform_info(void);

#ifdef __cplusplus
}
#endif

#endif // ALGO_MATH_H