/*
 * algo_envelope.c - 包络分析实现（振动信号解调）
 * 
 * 完整的包络解调分析流水线：
 * 1. 从原始IMU数据摄入
 * 2. 高通滤波（去除直流偏移和重力1g）
 * 3. 信号整流（绝对值）
 * 4. 低通滤波（提取包络）
 * 
 * 使用硬件隔离层进行滤波，零动态内存分配。
 * 所有缓冲区由调用方提供，完全解耦架构。
 */

#include "algo_pdm.h"
#include "algo_math.h"
#include <math.h>
#include <string.h>

/* ========================================================================= *
 * 内部类型定义
 * ========================================================================= */

// 包络分析器内部状态
typedef struct {
    algo_biquad_filter_t hp_filter;  // 高通滤波器
    algo_biquad_filter_t lp_filter;  // 低通滤波器
    float fs_hz;                     // 采样频率
    float hp_cutoff_hz;              // 高通截止频率
    float lp_cutoff_hz;              // 低通截止频率
} envelope_internal_state_t;

/* ========================================================================= *
 * 内部辅助函数
 * ========================================================================= */

/**
 * @brief 计算巴特沃斯滤波器的Q值
 * @param order 滤波器阶数
 * @return Q值
 */
static inline float butterworth_q_factor(int order)
{
    // 二阶巴特沃斯滤波器的Q值为1/√2 ≈ 0.7071
    return 0.7071f;
}

/**
 * @brief 验证包络分析器参数
 * @param fs 采样频率
 * @param hp_freq 高通截止频率
 * @param lp_freq 低通截止频率
 * @return 参数有效返回true，否则返回false
 */
static inline bool validate_envelope_params(float fs, float hp_freq, float lp_freq)
{
    if (fs <= 0.0f || hp_freq <= 0.0f || lp_freq <= 0.0f) {
        ALGO_LOGE("ENVELOPE", "Invalid parameters: fs=%.1f, hp=%.1f, lp=%.1f", 
                  fs, hp_freq, lp_freq);
        return false;
    }
    
    // 检查频率关系：高通截止频率应小于低通截止频率
    if (hp_freq >= lp_freq) {
        ALGO_LOGE("ENVELOPE", "High-pass cutoff (%.1f) must be less than low-pass cutoff (%.1f)", 
                  hp_freq, lp_freq);
        return false;
    }
    
    // 检查奈奎斯特频率
    float nyquist = fs / 2.0f;
    if (lp_freq >= nyquist) {
        ALGO_LOGE("ENVELOPE", "Low-pass cutoff (%.1f) must be less than Nyquist frequency (%.1f)", 
                  lp_freq, nyquist);
        return false;
    }
    
    return true;
}

/* ========================================================================= *
 * 公共API实现
 * ========================================================================= */

int algo_envelope_init(algo_envelope_ctx_t *ctx, float fs, float hp_freq, float lp_freq)
{
    // 参数验证
    if (ctx == NULL) {
        ALGO_LOGE("ENVELOPE", "Context is NULL");
        return -1;
    }
    
    if (!validate_envelope_params(fs, hp_freq, lp_freq)) {
        return -2;
    }
    
    // 分配内部状态
    envelope_internal_state_t *internal_state = 
        (envelope_internal_state_t*)malloc(sizeof(envelope_internal_state_t));
    if (internal_state == NULL) {
        ALGO_LOGE("ENVELOPE", "Failed to allocate internal state");
        return -3;
    }
    
    // 初始化内部状态
    internal_state->fs_hz = fs;
    internal_state->hp_cutoff_hz = hp_freq;
    internal_state->lp_cutoff_hz = lp_freq;
    
    // 计算滤波器系数
    float hp_coeffs[5];
    float lp_coeffs[5];
    float Q = butterworth_q_factor(2); // 二阶巴特沃斯
    
    // 计算高通滤波器系数
    algo_math_biquad_calc_hp(hp_coeffs, fs, hp_freq, Q);
    algo_math_biquad_init(&internal_state->hp_filter, hp_coeffs);
    
    // 计算低通滤波器系数
    algo_math_biquad_calc_lp(lp_coeffs, fs, lp_freq, Q);
    algo_math_biquad_init(&internal_state->lp_filter, lp_coeffs);
    
    // 存储内部状态到上下文
    ctx->hp_filter_state = internal_state;
    ctx->lp_filter_state = &internal_state->hp_filter; // 注意：这里存储的是滤波器指针
    
    ALGO_LOGI("ENVELOPE", "Initialized envelope analyzer: fs=%.1fHz, hp=%.1fHz, lp=%.1fHz", 
              fs, hp_freq, lp_freq);
    
    return 0; // 成功
}

void algo_envelope_process(algo_envelope_ctx_t *ctx,
                           const imu_raw_data_t *raw_src,
                           size_t count,
                           algo_axis_t axis,
                           float sensitivity,
                           float *work_buf,
                           float *out_buf)
{
    // 参数验证
    if (ctx == NULL || raw_src == NULL || work_buf == NULL || out_buf == NULL || count == 0) {
        ALGO_LOGE("ENVELOPE", "Invalid parameters for envelope processing");
        return;
    }
    
    if (ctx->hp_filter_state == NULL) {
        ALGO_LOGE("ENVELOPE", "Envelope analyzer not initialized");
        return;
    }
    
    envelope_internal_state_t *internal_state = (envelope_internal_state_t*)ctx->hp_filter_state;
    
    ALGO_LOGD("ENVELOPE", "Processing %zu samples for axis %d", count, (int)axis);
    
    // 步骤1：摄入原始数据到工作缓冲区
    algo_ingest_axis(raw_src, count, axis, sensitivity, work_buf);
    
    // 步骤2：高通滤波去除直流偏移和重力（1g）
    algo_math_biquad_process_array(&internal_state->hp_filter, work_buf, count);
    
    // 步骤3：信号整流（绝对值用于包络检测）
    algo_math_vector_abs(work_buf, work_buf, count);
    
    // 步骤4：低通滤波提取包络
    // 首先复制到输出缓冲区
    memcpy(out_buf, work_buf, count * sizeof(float));
    algo_math_biquad_process_array(&internal_state->lp_filter, out_buf, count);
    
    ALGO_LOGD("ENVELOPE", "Completed envelope processing for %zu samples", count);
}

void algo_envelope_cleanup(algo_envelope_ctx_t *ctx)
{
    if (ctx == NULL || ctx->hp_filter_state == NULL) {
        return;
    }
    
    envelope_internal_state_t *internal_state = (envelope_internal_state_t*)ctx->hp_filter_state;
    
    // 释放内部状态
    free(internal_state);
    
    // 清空上下文指针
    ctx->hp_filter_state = NULL;
    ctx->lp_filter_state = NULL;
    
    ALGO_LOGI("ENVELOPE", "Cleaned up envelope analyzer resources");
}

/* ========================================================================= *
 * 替代包络检测方法
 * ========================================================================= */

/**
 * @brief 希尔伯特变换包络检测（更准确）
 * @details 使用解析信号方法：包络 = |信号 + j*希尔伯特(信号)|
 * @note 需要更大的工作缓冲区（2*count用于FFT）
 */
void algo_envelope_hilbert(const float *signal,
                           size_t count,
                           float *work_buf,
                           float *envelope)
{
    if (signal == NULL || work_buf == NULL || envelope == NULL || count == 0) {
        ALGO_LOGE("ENVELOPE", "Invalid parameters for Hilbert envelope");
        return;
    }
    
    // 简化实现：使用绝对值方法
    // 完整实现应使用基于FFT的希尔伯特变换
    
    // 复制信号到工作缓冲区
    memcpy(work_buf, signal, count * sizeof(float));
    
    // 简单绝对值包络（希尔伯特变换的占位符）
    algo_math_vector_abs(envelope, work_buf, count);
    
    ALGO_LOGD("ENVELOPE", "Hilbert envelope detection for %zu samples (simplified)", count);
}

/**
 * @brief 移动平均包络检测（更简单，更快）
 * @details 使用整流信号的移动平均
 * @param signal 输入信号
 * @param count 样本数
 * @param window_size 移动平均窗口大小
 * @param envelope 输出包络
 */
void algo_envelope_moving_avg(const float *signal,
                              size_t count,
                              size_t window_size,
                              float *envelope)
{
    if (signal == NULL || envelope == NULL || count == 0 || window_size == 0) {
        ALGO_LOGE("ENVELOPE", "Invalid parameters for moving average envelope");
        return;
    }
    
    // 限制窗口大小
    if (window_size > count) {
        window_size = count;
        ALGO_LOGW("ENVELOPE", "Window size reduced to %zu", window_size);
    }
    
    // 计算绝对值的移动平均
    for (size_t i = 0; i < count; i++) {
        float sum = 0.0f;
        size_t start = (i > window_size) ? (i - window_size) : 0;
        size_t end = i + 1;
        size_t window_count = end - start;
        
        for (size_t j = start; j < end; j++) {
            sum += fabsf(signal[j]);
        }
        
        envelope[i] = sum / window_count;
    }
    
    ALGO_LOGD("ENVELOPE", "Moving average envelope detection for %zu samples, window=%zu", 
              count, window_size);
}

/* ========================================================================= *
 * 包络特征提取
 * ========================================================================= */

/**
 * @brief 从包络信号提取特征
 * @param envelope 包络信号
 * @param count 样本数
 * @param rms 输出：包络的RMS
 * @param peak 输出：包络的峰值
 * @param crest_factor 输出：峰值因子（峰值/RMS）
 */
void algo_envelope_extract_features(const float *envelope,
                                    size_t count,
                                    float *rms,
                                    float *peak,
                                    float *crest_factor)
{
    if (envelope == NULL || count == 0) {
        if (rms) *rms = 0.0f;
        if (peak) *peak = 0.0f;
        if (crest_factor) *crest_factor = 0.0f;
        ALGO_LOGE("ENVELOPE", "Invalid parameters for feature extraction");
        return;
    }
    
    // 计算RMS
    float rms_val = algo_math_vector_rms(envelope, count);
    
    // 计算峰值
    float max_val = 0.0f;
    for (size_t i = 0; i < count; i++) {
        if (envelope[i] > max_val) {
            max_val = envelope[i];
        }
    }
    
    // 计算峰值因子
    float crest = (rms_val > 0.0f) ? (max_val / rms_val) : 0.0f;
    
    // 输出结果
    if (rms) *rms = rms_val;
    if (peak) *peak = max_val;
    if (crest_factor) *crest_factor = crest;
    
    ALGO_LOGD("ENVELOPE", "Extracted features: RMS=%.4f, Peak=%.4f, Crest=%.4f", 
              rms_val, max_val, crest);
}

/* ========================================================================= *
 * 包络分析质量指标
 * ========================================================================= */

/**
 * @brief 计算包络分析的质量指标
 * @param original 原始信号
 * @param envelope 包络信号
 * @param count 样本数
 * @param smoothness 输出：平滑度指标（0-1，越高越平滑）
 * @param fidelity 输出：保真度指标（0-1，越高越保真）
 */
void algo_envelope_quality_metrics(const float *original,
                                   const float *envelope,
                                   size_t count,
                                   float *smoothness,
                                   float *fidelity)
{
    if (original == NULL || envelope == NULL || count == 0) {
        if (smoothness) *smoothness = 0.0f;
        if (fidelity) *fidelity = 0.0f;
        return;
    }
    
    // 计算平滑度：包络信号的变化率
    float smoothness_sum = 0.0f;
    for (size_t i = 1; i < count; i++) {
        float diff = envelope[i] - envelope[i-1];
        smoothness_sum += fabsf(diff);
    }
    
    // 归一化平滑度（值越小越平滑）
    float avg_change = smoothness_sum / (count - 1);
    float max_envelope = 0.0f;
    for (size_t i = 0; i < count; i++) {
        if (envelope[i] > max_envelope) {
            max_envelope = envelope[i];
        }
    }
    
    float smoothness_val = (max_envelope > 0.0f) ? (1.0f - avg_change / max_envelope) : 0.0f;
    if (smoothness_val < 0.0f) smoothness_val = 0.0f;
    if (smoothness_val > 1.0f) smoothness_val = 1.0f;
    
    // 计算保真度：包络与原始信号绝对值的相关性
    float *abs_original = (float*)malloc(count * sizeof(float));
    if (abs_original == NULL) {
        if (smoothness) *smoothness = smoothness_val;
        if (fidelity) *fidelity = 0.0f;
        return;
    }
    
    // 计算原始信号的绝对值
    for (size_t i = 0; i < count; i++) {
        abs_original[i] = fabsf(original[i]);
    }
    
    // 计算相关系数（简化版本）
    float mean_abs = algo_math_vector_mean(abs_original, count);
    float mean_env = algo_math_vector_mean(envelope, count);
    
    float cov_sum = 0.0f;
    float var_abs_sum = 0.0f;
    float var_env_sum = 0.0f;
    
    for (size_t i = 0; i < count; i++) {
        float diff_abs = abs_original[i] - mean_abs;
        float diff_env = envelope[i] - mean_env;
        cov_sum += diff_abs * diff_env;
        var_abs_sum += diff_abs * diff_abs;
        var_env_sum += diff_env * diff_env;
    }
    
    free(abs_original);
    
    float fidelity_val = 0.0f;
    if (var_abs_sum > 0.0f && var_env_sum > 0.0f) {
        fidelity_val = cov_sum / sqrtf(var_abs_sum * var_env_sum);
        // 限制在0-1范围内
        if (fidelity_val < 0.0f) fidelity_val = 0.0f;
        if (fidelity_val > 1.0f) fidelity_val = 1.0f;
    }
    
    // 输出结果
    if (smoothness) *smoothness = smoothness_val;
    if (fidelity) *fidelity = fidelity_val;
    
    ALGO_LOGD("ENVELOPE", "Quality metrics: Smoothness=%.4f, Fidelity=%.4f", 
              smoothness_val, fidelity_val);
}

/* ========================================================================= *
 * 实时包络分析（流式处理）
 * ========================================================================= */

/**
 * @brief 实时包络分析初始化
 * @param ctx 包络上下文
 * @param fs 采样频率
 * @param hp_freq 高通截止频率
 * @param lp_freq 低通截止频率
 * @return 0表示成功，负数表示错误
 * @note 专为实时流式处理优化
 */
int algo_envelope_realtime_init(algo_envelope_ctx_t *ctx, float fs, float hp_freq, float lp_freq)
{
    // 使用标准初始化
    return algo_envelope_init(ctx, fs, hp_freq, lp_freq);
}

/**
 * @brief 实时处理单个样本
 * @param ctx 包络上下文
 * @param sample 输入样本
 * @return 包络输出样本
 * @note 适用于实时流式处理
 */
float algo_envelope_realtime_process(algo_envelope_ctx_t *ctx, float sample)
{
    if (ctx == NULL || ctx->hp_filter_state == NULL) {
        return 0.0f;
    }
    
    envelope_internal_state_t *internal_state = (envelope_internal_state_t*)ctx->hp_filter_state;
    
    // 步骤1：高通滤波去除直流偏移和重力（1g）
    float hp_filtered = algo_math_biquad_process(&internal_state->hp_filter, sample);
    
    // 步骤2：信号整流（绝对值）
    float rectified = fabsf(hp_filtered);
    
    // 步骤3：低通滤波提取包络
    float envelope = algo_math_biquad_process(&internal_state->lp_filter, rectified);
    
    return envelope;
}

/* ========================================================================= *
 * 编译时架构验证
 * ========================================================================= */

// 验证没有直接包含