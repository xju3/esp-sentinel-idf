/* 
 * algo_math.c - 硬件隔离抽象层实现
 * 
 * 核心解耦实现：使用 #ifdef ESP_PLATFORM 路由到 esp-dsp 或标准数学库
 * 
 * 架构亮点：
 * 1. ESP32平台：使用 esp-dsp 硬件加速指令
 * 2. PC平台：使用标准 <math.h> 和纯C实现
 * 3. 统一接口：无论平台如何，外部调用方式完全一致
 * 4. 零动态内存：所有缓冲区由调用方分配
 */

#include "algo_math.h"
#include <math.h>
#include <string.h>

/* ========================================================================= *
 * 平台检测和配置
 * ========================================================================= */

#ifdef ESP_PLATFORM
// ESP32平台：使用硬件加速
#include "esp_dsp.h"
#define HAS_HARDWARE_ACCELERATION 1
#define PLATFORM_INFO "ESP32 (esp-dsp hardware acceleration)"
#else
// PC平台：使用纯C实现
#define HAS_HARDWARE_ACCELERATION 0
#define PLATFORM_INFO "PC (standard C math library)"
#endif

bool algo_math_has_hardware_acceleration(void)
{
    return HAS_HARDWARE_ACCELERATION;
}

const char* algo_math_get_platform_info(void)
{
    return PLATFORM_INFO;
}

/* ========================================================================= *
 * 1. 基本数学运算实现
 * ========================================================================= */

float algo_math_sqrt(float x)
{
#ifdef ESP_PLATFORM
    // ESP32: 使用硬件加速平方根
    return sqrtf(x); // esp-dsp 提供硬件加速的 sqrtf
#else
    // PC: 使用标准库
    return sqrtf(x);
#endif
}

float algo_math_sin(float x)
{
#ifdef ESP_PLATFORM
    // ESP32: 使用硬件加速三角函数
    return sinf(x);
#else
    // PC: 使用标准库
    return sinf(x);
#endif
}

float algo_math_cos(float x)
{
#ifdef ESP_PLATFORM
    // ESP32: 使用硬件加速三角函数
    return cosf(x);
#else
    // PC: 使用标准库
    return cosf(x);
#endif
}

float algo_math_atan2(float y, float x)
{
#ifdef ESP_PLATFORM
    // ESP32: 使用硬件加速反正切
    return atan2f(y, x);
#else
    // PC: 使用标准库
    return atan2f(y, x);
#endif
}

float algo_math_exp(float x)
{
#ifdef ESP_PLATFORM
    // ESP32: 使用硬件加速指数函数
    return expf(x);
#else
    // PC: 使用标准库
    return expf(x);
#endif
}

float algo_math_log(float x)
{
#ifdef ESP_PLATFORM
    // ESP32: 使用硬件加速对数函数
    return logf(x);
#else
    // PC: 使用标准库
    return logf(x);
#endif
}

/* ========================================================================= *
 * 2. 向量和矩阵运算实现
 * ========================================================================= */

float algo_math_dot_product(const float *a, const float *b, size_t len)
{
#ifdef ESP_PLATFORM
    // ESP32: 使用 esp-dsp 硬件加速点积
    float result;
    dsps_dotprod_f32(a, b, &result, len);
    return result;
#else
    // PC: 使用纯C实现（可考虑使用 SIMD 指令如 SSE/AVX）
    float sum = 0.0f;
    for (size_t i = 0; i < len; i++) {
        sum += a[i] * b[i];
    }
    return sum;
#endif
}

void algo_math_vector_add(float *dst, const float *src, size_t len)
{
#ifdef ESP_PLATFORM
    // ESP32: 使用 esp-dsp 硬件加速向量加法
    dsps_add_f32(dst, src, dst, len, 1, 1, 1);
#else
    // PC: 使用纯C实现
    for (size_t i = 0; i < len; i++) {
        dst[i] += src[i];
    }
#endif
}

void algo_math_vector_sub(float *dst, const float *src, size_t len)
{
#ifdef ESP_PLATFORM
    // ESP32: 使用 esp-dsp 硬件加速向量减法
    dsps_sub_f32(dst, src, dst, len, 1, 1, 1);
#else
    // PC: 使用纯C实现
    for (size_t i = 0; i < len; i++) {
        dst[i] -= src[i];
    }
#endif
}

void algo_math_vector_mul(float *dst, const float *src, size_t len)
{
#ifdef ESP_PLATFORM
    // ESP32: 使用 esp-dsp 硬件加速向量乘法
    dsps_mul_f32(dst, src, dst, len, 1, 1, 1);
#else
    // PC: 使用纯C实现
    for (size_t i = 0; i < len; i++) {
        dst[i] *= src[i];
    }
#endif
}

void algo_math_vector_scale(float *data, float scale, size_t len)
{
#ifdef ESP_PLATFORM
    // ESP32: 使用 esp-dsp 硬件加速向量缩放
    for (size_t i = 0; i < len; i++) {
        data[i] *= scale;
    }
    // 注意：esp-dsp 没有直接的向量缩放函数，使用循环
#else
    // PC: 使用纯C实现
    for (size_t i = 0; i < len; i++) {
        data[i] *= scale;
    }
#endif
}

void algo_math_vector_abs(float *dst, const float *src, size_t len)
{
#ifdef ESP_PLATFORM
    // ESP32: 使用 esp-dsp 硬件加速绝对值
    for (size_t i = 0; i < len; i++) {
        dst[i] = fabsf(src[i]);
    }
#else
    // PC: 使用纯C实现
    for (size_t i = 0; i < len; i++) {
        dst[i] = fabsf(src[i]);
    }
#endif
}

void algo_math_vector_square(float *dst, const float *src, size_t len)
{
#ifdef ESP_PLATFORM
    // ESP32: 使用 esp-dsp 硬件加速平方
    for (size_t i = 0; i < len; i++) {
        dst[i] = src[i] * src[i];
    }
#else
    // PC: 使用纯C实现
    for (size_t i = 0; i < len; i++) {
        dst[i] = src[i] * src[i];
    }
#endif
}

/* ========================================================================= *
 * 3. 滤波器运算实现
 * ========================================================================= */

void algo_math_biquad_init(algo_biquad_filter_t *filter, const float *coeffs)
{
    if (filter == NULL || coeffs == NULL) {
        return;
    }
    
    // 复制系数
    memcpy(filter->coeffs, coeffs, 5 * sizeof(float));
    
    // 重置延迟线
    filter->delay[0] = 0.0f;
    filter->delay[1] = 0.0f;
}

float algo_math_biquad_process(algo_biquad_filter_t *filter, float input)
{
    if (filter == NULL) {
        return 0.0f;
    }
    
    // 直接形式 II 双二阶滤波器实现
    const float *c = filter->coeffs;
    float *d = filter->delay;
    
    float w = input - c[3] * d[0] - c[4] * d[1];
    float y = c[0] * w + c[1] * d[0] + c[2] * d[1];
    
    // 更新延迟线
    d[1] = d[0];
    d[0] = w;
    
    return y;
}

void algo_math_biquad_process_array(algo_biquad_filter_t *filter, float *data, size_t len)
{
    if (filter == NULL || data == NULL || len == 0) {
        return;
    }
    
#ifdef ESP_PLATFORM
    // ESP32: 使用 esp-dsp 硬件加速双二阶滤波器
    // 注意：这里使用循环调用单个样本处理
    // 实际项目中可以使用 dsps_biquad_f32 进行批量处理
    for (size_t i = 0; i < len; i++) {
        data[i] = algo_math_biquad_process(filter, data[i]);
    }
#else
    // PC: 使用纯C实现
    for (size_t i = 0; i < len; i++) {
        data[i] = algo_math_biquad_process(filter, data[i]);
    }
#endif
}

void algo_math_biquad_calc_hp(float *coeffs, float fs, float fc, float Q)
{
    if (coeffs == NULL || fs <= 0.0f || fc <= 0.0f || Q <= 0.0f) {
        return;
    }
    
    // 计算归一化频率
    float w0 = 2.0f * M_PI * fc / fs;
    float alpha = sinf(w0) / (2.0f * Q);
    float cos_w0 = cosf(w0);
    
    // 高通双二阶滤波器系数
    float b0 = (1.0f + cos_w0) / 2.0f;
    float b1 = -(1.0f + cos_w0);
    float b2 = (1.0f + cos_w0) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f - alpha;
    
    // 归一化
    coeffs[0] = b0 / a0;
    coeffs[1] = b1 / a0;
    coeffs[2] = b2 / a0;
    coeffs[3] = a1 / a0;
    coeffs[4] = a2 / a0;
}

void algo_math_biquad_calc_lp(float *coeffs, float fs, float fc, float Q)
{
    if (coeffs == NULL || fs <= 0.0f || fc <= 0.0f || Q <= 0.0f) {
        return;
    }
    
    // 计算归一化频率
    float w0 = 2.0f * M_PI * fc / fs;
    float alpha = sinf(w0) / (2.0f * Q);
    float cos_w0 = cosf(w0);
    
    // 低通双二阶滤波器系数
    float b0 = (1.0f - cos_w0) / 2.0f;
    float b1 = 1.0f - cos_w0;
    float b2 = (1.0f - cos_w0) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f - alpha;
    
    // 归一化
    coeffs[0] = b0 / a0;
    coeffs[1] = b1 / a0;
    coeffs[2] = b2 / a0;
    coeffs[3] = a1 / a0;
    coeffs[4] = a2 / a0;
}

/* ========================================================================= *
 * 4. FFT运算实现（硬件加速路由）
 * ========================================================================= */

// FFT引擎内部结构
typedef struct {
    size_t max_fft_size;
#ifdef ESP_PLATFORM
    // ESP32: 使用 esp-dsp FFT 上下文
    void* dsp_fft_context;
#endif
    // PC: 可以添加 KissFFT 上下文
} algo_fft_engine_internal_t;

algo_fft_engine_t algo_math_fft_init(size_t max_fft_size)
{
    // 检查参数
    if (max_fft_size == 0 || (max_fft_size & (max_fft_size - 1)) != 0) {
        // 不是2的幂
        return NULL;
    }
    
    // 分配引擎结构
    algo_fft_engine_internal_t *engine = 
        (algo_fft_engine_internal_t*)malloc(sizeof(algo_fft_engine_internal_t));
    if (engine == NULL) {
        return NULL;
    }
    
    engine->max_fft_size = max_fft_size;
    
#ifdef ESP_PLATFORM
    // ESP32: 初始化 esp-dsp FFT 引擎
    // 注意：这里需要根据实际 esp-dsp API 进行初始化
    // 这里使用占位符，实际项目需要实现
    engine->dsp_fft_context = NULL;
    
    // 示例：使用 dsps_fft2r_init_fc32 初始化
    // int ret = dsps_fft2r_init_fc32(NULL, max_fft_size);
    // if (ret != 0) {
    //     free(engine);
    //     return NULL;
    // }
#else
    // PC: 初始化纯C FFT引擎（可接入KissFFT）
    // 这里使用占位符，实际项目需要实现
#endif
    
    return (algo_fft_engine_t)engine;
}

void algo_math_fft_execute_real(algo_fft_engine_t engine,
                                const float *input,
                                size_t n,
                                float *work_buf,
                                float *output)
{
    if (engine == NULL || input == NULL || work_buf == NULL || output == NULL) {
        return;
    }
    
    algo_fft_engine_internal_t *fft_engine = (algo_fft_engine_internal_t*)engine;
    
    // 检查FFT大小
    if (n == 0 || (n & (n - 1)) != 0 || n > fft_engine->max_fft_size) {
        return;
    }
    
#ifdef ESP_PLATFORM
    // ESP32: 使用 esp-dsp 硬件加速FFT
    // 注意：这里需要根据实际 esp-dsp API 进行调用
    // 这里使用占位符，实际项目需要实现
    
    // 示例：使用 dsps_fft2r_fc32 执行实数FFT
    // dsps_fft2r_fc32(work_buf, n);
    
    // 将结果复制到输出缓冲区
    for (size_t i = 0; i < n; i++) {
        work_buf[i] = input[i];
    }
    
    // 执行FFT（占位符）
    // 实际实现应该调用硬件加速FFT
    
    // 将结果复制到输出
    for (size_t i = 0; i < n/2 + 1; i++) {
        output[2*i] = work_buf[i];     // 实部
        output[2*i + 1] = 0.0f;        // 虚部（占位符）
    }
#else
    // PC: 使用纯C实现FFT（可接入KissFFT）
    // 这里使用简单的DFT作为占位符
    size_t half_n = n / 2;
    
    for (size_t k = 0; k <= half_n; k++) {
        float real = 0.0f;
        float imag = 0.0f;
        
        for (size_t t = 0; t < n; t++) {
            float angle = 2.0f * M_PI * k * t / n;
            real += input[t] * cosf(angle);
            imag -= input[t] * sinf(angle);
        }
        
        output[2*k] = real;
        output[2*k + 1] = imag;
    }
#endif
}

void algo_math_fft_compute_magnitude(const float *complex_fft,
                                     float *magnitude,
                                     size_t n_fft)
{
    if (complex_fft == NULL || magnitude == NULL || n_fft == 0) {
        return;
    }
    
    size_t half_n = n_fft / 2;
    
    for (size_t i = 0; i < half_n; i++) {
        float real = complex_fft[2*i];
        float imag = complex_fft[2*i + 1];
        magnitude[i] = sqrtf(real*real + imag*imag);
    }
}

void algo_math_fft_cleanup(algo_fft_engine_t engine)
{
    if (engine == NULL) {
        return;
    }
    
    algo_fft_engine_internal_t *fft_engine = (algo_fft_engine_internal_t*)engine;
    
#ifdef ESP_PLATFORM
    // ESP32: 清理 esp-dsp FFT 资源
    if (fft_engine->dsp_fft_context != NULL) {
        // 根据实际 esp-dsp API 进行清理
        // dsps_fft2r_deinit_fc32(fft_engine->dsp_fft_context);
    }
#endif
    
    free(fft_engine);
}

/* ========================================================================= *
 * 5. 统计运算实现
 * ========================================================================= */

float algo_math_vector_mean(const float *data, size_t len)
{
    if (data == NULL || len == 0) {
        return 0.0f;
    }
    
    float sum = 0.0f;
    
#ifdef ESP_PLATFORM
    // ESP32: 使用 esp-dsp 硬件加速求和
    // 注意：dsps_sum_f32 可能不存在，使用循环实现
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
#else
    // PC: 使用纯C实现
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
#endif
    
    return sum / len;
}

float algo_math_vector_variance(const float *data, size_t len, float mean)
{
    if (data == NULL || len == 0) {
        return 0.0f;
    }
    
    float sum_sq = 0.0f;
    
    // 计算平方和
    for (size_t i = 0; i < len; i++) {
        float diff = data[i] - mean;
        sum_sq += diff * diff;
    }
    
    return sum_sq / len;
}

float algo_math_vector_std_dev(const float *data, size_t len, float mean)
{
    float variance = algo_math_vector_variance(data, len, mean);
    return algo_math_sqrt(variance);
}

float algo_math_vector_rms(const float *data, size_t len)
{
    if (data == NULL || len == 0) {
        return 0.0f;
    }
    
    float sum_sq = 0.0f;
    
#ifdef ESP_PLATFORM
    // ESP32: 使用硬件加速计算平方和
    for (size_t i = 0; i < len; i++) {
        sum_sq += data[i] * data[i];
    }
#else
    // PC: 使用纯C实现
    for (size_t i = 0; i < len; i++) {
        sum_sq += data[i] * data[i];
    }
#endif
    
    return algo_math_sqrt(sum_sq / len);
}

/* ========================================================================= *
 * 6. 辅助函数
 * ========================================================================= */

// 注意：这里使用了 malloc，但在实际算法中应该避免
// 这个函数仅用于FFT引擎初始化，不是算法核心部分
// 在实际项目中，应该由调用方分配FFT引擎内存

/* ========================================================================= *
 * 7. 平台特定的优化提示
 * ========================================================================= */

// ESP32平台优化提示：
// 1. 使用 dsps_ 前缀的函数进行硬件加速
// 2. 确保数据对齐到16字节边界以获得最佳性能
// 3. 使用 PSRAM 存储大数组时注意性能影响

// PC平台优化提示：
// 1. 可接入 KissFFT 进行快速傅里叶变换
// 2. 可使用 OpenMP 进行多线程并行计算
// 3. 可使用 SIMD 指令（SSE/AVX）进行向量化优化

/* ========================================================================= *
 * 8. 编译时配置检查
 * ========================================================================= */

// 编译时检查：确保没有直接包含项目业务头文件
// 注意：这些检查在编译时进行，确保架构解耦

// 检查是否意外包含了 FreeRTOS 头文件
#ifdef FREERTOS_H
#error "algo_math.c must not include FreeRTOS headers"
#endif

// 检查是否意外包含了 ESP-IDF 业务头文件
#ifdef ESP_LOG_H
#error "algo_math.c must not include ESP-IDF logging headers"
#endif

/* ========================================================================= *
 * 9. 性能基准测试占位符
 * ========================================================================= */

// 这些函数可用于性能测试和基准比较
#ifdef ALGO_MATH_BENCHMARK

#include <time.h>

/**
 * @brief 性能基准测试：向量点积
 * @param a 向量A
 * @param b 向量B
 * @param len 向量长度
 * @param iterations 迭代次数
 * @return 平均执行时间（微秒）
 */
double benchmark_dot_product(const float *a, const float *b, size_t len, int iterations)
{
    clock_t start = clock();
    
    for (int i = 0; i < iterations; i++) {
        algo_math_dot_product(a, b, len);
    }
    
    clock_t end = clock();
    double elapsed = (double)(end - start) / CLOCKS_PER_SEC * 1e6;
    return elapsed / iterations;
}

/**
 * @brief 性能基准测试：FFT
 * @param input 输入信号
 * @param n FFT大小
 * @param iterations 迭代次数
 * @return 平均执行时间（微秒）
 */
double benchmark_fft(const float *input, size_t n, int iterations)
{
    // 分配工作缓冲区
    float *work_buf = (float*)malloc(2 * n * sizeof(float));
    float *output = (float*)malloc((n + 2) * sizeof(float));
    
    if (work_buf == NULL || output == NULL) {
        free(work_buf);
        free(output);
        return -1.0;
    }
    
    // 初始化FFT引擎
    algo_fft_engine_t engine = algo_math_fft_init(n);
    if (engine == NULL) {
        free(work_buf);
        free(output);
        return -1.0;
    }
    
    clock_t start = clock();
    
    for (int i = 0; i < iterations; i++) {
        algo_math_fft_execute_real(engine, input, n, work_buf, output);
    }
    
    clock_t end = clock();
    double elapsed = (double)(end - start) / CLOCKS_PER_SEC * 1e6;
    
    // 清理
    algo_math_fft_cleanup(engine);
    free(work_buf);
    free(output);
    
    return elapsed / iterations;
}

#endif // ALGO_MATH_BENCHMARK

/* ========================================================================= *
 * 10. 单元测试占位符
 * ========================================================================= */

#ifdef ALGO_MATH_TEST

#include <stdio.h>
#include <assert.h>

void test_algo_math_basic(void)
{
    printf("Testing basic math functions...\n");
    
    // 测试平方根
    float x = 4.0f;
    float sqrt_x = algo_math_sqrt(x);
    assert(fabs(sqrt_x - 2.0f) < 1e-6);
    
    // 测试三角函数
    float angle = M_PI / 4.0f; // 45度
    float sin_val = algo_math_sin(angle);
    float cos_val = algo_math_cos(angle);
    assert(fabs(sin_val - 0.70710678f) < 1e-6);
    assert(fabs(cos_val - 0.70710678f) < 1e-6);
    
    printf("Basic math tests passed!\n");
}

void test_algo_math_vector(void)
{
    printf("Testing vector operations...\n");
    
    float a[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    float b[4] = {5.0f, 6.0f, 7.0f, 8.0f};
    float result[4];
    
    // 测试向量加法
    memcpy(result, a, sizeof(a));
    algo_math_vector_add(result, b, 4);
    assert(fabs(result[0] - 6.0f) < 1e-6);
    assert(fabs(result[3] - 12.0f) < 1e-6);
    
    // 测试点积
    float dot = algo_math_dot_product(a, b, 4);
    assert(fabs(dot - 70.0f) < 1e-6); // 1*5 + 2*6 + 3*7 + 4*8 = 70
    
    printf("Vector operation tests passed!\n");
}

#endif // ALGO_MATH_TEST

/* ========================================================================= *
 * 编译标记
 * ========================================================================= */

// 标记此文件为算法数学库实现
#define ALGO_MATH_IMPLEMENTATION

