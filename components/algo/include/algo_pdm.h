/* 
 * algo_pdm.h - 预测性维护算法库顶层接口
 * 
 * 工业级DSP库，用于ESP32-S3/ESP32上的振动分析。
 * 纯数学函数，零外部依赖（除硬件隔离层外）。
 * 
 * 核心架构原则：
 * 1. 绝对零业务耦合 - 不包含任何项目内部头文件
 * 2. 零动态内存分配 - 所有缓冲区由调用方分配
 * 3. 硬件加速隔离 - 通过algo_math层路由到esp-dsp或纯C实现
 * 4. 高效数据摄入 - 处理打包的大端int16_t DMA数据
 * 5. 完全解耦的日志系统 - 通过回调注入日志
 */

#ifndef ALGO_PDM_H
#define ALGO_PDM_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LSB_TO_G (16.0f / 32768.0f)

/* ========================================================================= *
 * 1. 日志系统定义（完全解耦）
 * ========================================================================= */

/**
 * @brief 算法库日志级别
 */
typedef enum {
    ALGO_LOG_ERROR = 0,     // 错误条件
    ALGO_LOG_WARN  = 1,     // 警告条件
    ALGO_LOG_INFO  = 2,     // 信息消息
    ALGO_LOG_DEBUG = 3      // 调试消息
} algo_log_level_t;

/**
 * @brief 日志回调函数类型
 * @param level 日志级别
 * @param tag 日志标签（模块标识符）
 * @param fmt 格式字符串（printf风格）
 * @param args 可变参数列表
 */
typedef void (*algo_log_cb_t)(algo_log_level_t level, const char *tag, 
                              const char *fmt, va_list args);



/**
 * @brief 算法库全局配置结构
 * @note 所有字段都有合理的默认值，可以部分配置
 */
typedef struct {
    /**
     * @brief 日志回调函数
     * @note 如果为NULL，则禁用所有日志输出
     */
    algo_log_cb_t log_cb;
    
    /**
     * @brief 默认灵敏度系数（g/LSB）
     * @note 用于未指定灵敏度的API调用
     */
    float default_sensitivity;
    
    /**
     * @brief 默认采样频率（Hz）
     * @note 用于需要采样频率但未指定的API调用
     */
    float default_fs_hz;
    
    /**
     * @brief 性能优化标志
     * @note 位掩码，控制各种优化选项
     */
    uint32_t optimization_flags;
    
    /**
     * @brief 平台特定配置
     * @note 用于存储平台特定的配置参数
     */
    void* platform_config;
} algo_config_t;

/**
 * @brief 注册日志回调函数
 * @param cb 日志回调函数指针（可为NULL，表示禁用日志）
 * @note 这是算法库与外部日志系统解耦的唯一接口
 */
void algo_register_log_callback(algo_log_cb_t cb);

/**
 * @brief 初始化算法库
 * @param config 配置参数（可为NULL，使用默认配置）
 * @return 0表示成功，负数表示错误
 * @note 这是算法库的全局初始化函数，应在使用任何算法功能前调用
 */
int algo_pdm_init(const algo_config_t *config);

/* ========================================================================= *
 * 2. 物理层数据定义（与DMA结构完全一致）
 * ========================================================================= */

/**
 * @brief 三轴加速度计原始数据结构（Raw Data）
 * @note 必须保持为int16_t以最小化DMA传输带宽和PSRAM使用
 * __attribute__((packed))确保结构体大小恰好为8字节。
 * 这对于DMA直接将SPI硬件流映射到内存结构至关重要。
 */
typedef struct
{
    uint8_t header; // 数据包头
    int16_t x;      // 原始X轴（注意：内部大端格式）
    int16_t y;      // 原始Y轴
    int16_t z;      // 原始Z轴
    int8_t temp;    // 8位截断温度辅助数据
} __attribute__((packed)) imu_raw_data_t;

// RMS Data Structure
typedef struct {
    int64_t timestamp;
    float rms_x;
    float rms_y;
    float rms_z;
    float rms_3d;
} imu_rms_data_t;

/**
 * @brief 数据处理的轴选择
 */
typedef enum {
    ALGO_AXIS_X = 0,
    ALGO_AXIS_Y = 1,
    ALGO_AXIS_Z = 2
} algo_axis_t;

/* ========================================================================= *
 * 3. 算法上下文结构（由调用方分配）
 * ========================================================================= */


/**
 * @brief 包络分析配置和状态
 * 使用硬件隔离层进行滤波
 */
typedef struct {
    float fs_hz;            // 采样频率（Hz）
    float hp_cutoff_hz;     // 高通截止频率（Hz）
    float lp_cutoff_hz;     // 低通截止频率（Hz）
    
    // 内部滤波器状态（通过硬件隔离层管理）
    void* hp_filter_state;  // 高通滤波器状态（不透明指针）
    void* lp_filter_state;  // 低通滤波器状态（不透明指针）
} algo_envelope_ctx_t;

/**
 * @brief FFT上下文（硬件加速频率分析）
 */
typedef struct {
    size_t max_fft_size;    // 支持的最大FFT大小
    void* fft_state;        // FFT引擎状态（不透明指针，通过硬件隔离层管理）
} algo_fft_ctx_t;

/* ========================================================================= *
 * 4. 顶层API函数（应用接口层）
 * ========================================================================= */

/**
 * @brief [数据摄入] 高效原始数据摄入算子
 * @details 提取指定轴 -> 大端到小端转换 -> 乘以灵敏度系数 -> 存储到调用方提供的float数组
 * @param src 原始IMU数据数组指针
 * @param count 要处理的样本数
 * @param axis 要提取的轴（X、Y或Z）
 * @param sensitivity 灵敏度系数（g/LSB或m/s²/LSB）
 * @param out_buf 调用方分配的输出缓冲区（大小 >= count）
 */
void algo_ingest_axis(const imu_raw_data_t *src, 
                      size_t count, 
                      algo_axis_t axis, 
                      float sensitivity, 
                      float *out_buf);


/**
 * @brief Welford在线统计上下文
 * O(1)空间复杂度，无需大数组
 */
typedef struct {
    uint32_t count;     // 已处理的样本数
    double mean;        // 当前均值
    double m2;          // 与均值差的平方和
    float min_val;      // 观察到的最小值
    float max_val;      // 观察到的最大值
} algo_welford_ctx_t;

/**
 * @brief [WELFORD] 获取当前统计信息
 * @param ctx Welford上下文
 * @param mean 输出：当前均值
 * @param variance 输出：当前方差
 * @param std_dev 输出：当前标准差
 */
// Deprecated/removed helper接口已删，仅保留 algo_welford_ctx_t 结构兼容

/**
 * @brief [RMS] 计算float数组的均方根
 * @param data 输入数据数组
 * @param count 样本数
 * @return RMS值
 */
float algo_calc_rms(const float *data, size_t count);

/**
 * @brief [KURTOSIS] 计算float数组的峭度
 * @param data 输入数据数组
 * @param count 样本数
 * @param mean 预计算的均值（可从Welford获取）
 * @param std_dev 预计算的标准差（可从Welford获取）
 * @return 峭度值（正态分布 = 3.0）
 */
float algo_calc_kurtosis(const float *data, size_t count, float mean, float std_dev);

/**
 * @brief [ENVELOPE] 初始化包络分析器
 * @param ctx 要初始化的包络上下文
 * @param fs 采样频率（Hz）
 * @param hp_freq 高通截止频率（Hz）
 * @param lp_freq 低通截止频率（Hz）
 * @return 0表示成功，负数表示错误
 * @note 内部使用硬件隔离层进行滤波器初始化
 */
int algo_envelope_init(algo_envelope_ctx_t *ctx, float fs, float hp_freq, float lp_freq);

/**
 * @brief [ENVELOPE] 完整的包络分析流水线
 * @details 摄入 -> 高通滤波 -> 整流 -> 低通滤波
 * @param ctx 已初始化的包络上下文
 * @param raw_src 原始IMU数据数组
 * @param count 要处理的样本数
 * @param axis 要处理的轴
 * @param sensitivity 灵敏度系数
 * @param work_buf 调用方分配的工作缓冲区（大小 >= count）
 * @param out_buf 调用方分配的输出缓冲区（大小 >= count）
 */
void algo_envelope_process(algo_envelope_ctx_t *ctx,
                           const imu_raw_data_t *raw_src,
                           size_t count,
                           algo_axis_t axis,
                           float sensitivity,
                           float *work_buf,
                           float *out_buf);

/**
 * @brief [FFT] 初始化FFT引擎
 * @param ctx FFT上下文
 * @param max_fft_size 支持的最大FFT大小（必须是2的幂）
 * @return 0表示成功，负数表示错误
 * @note 内部使用硬件隔离层进行FFT引擎初始化
 */
int algo_fft_init(algo_fft_ctx_t *ctx, size_t max_fft_size);

/**
 * @brief [FFT] 执行实数FFT并计算幅度谱
 * @param ctx 已初始化的FFT上下文
 * @param input 实数输入信号（长度N）
 * @param n FFT大小（必须是2的幂且 <= max_fft_size）
 * @param work_buf 工作缓冲区（长度2*N float）
 * @param output 幅度谱输出（长度N/2 float）
 */
void algo_fft_execute(algo_fft_ctx_t *ctx,
                      const float *input,
                      size_t n,
                      float *work_buf,
                      float *output);

/**
 * @brief [FFT] 清理FFT引擎资源
 * @param ctx FFT上下文
 * @note 内部使用硬件隔离层进行资源清理
 */
void algo_fft_cleanup(algo_fft_ctx_t *ctx);

/**
 * @brief [ENVELOPE] 清理包络分析器资源
 * @param ctx 包络上下文
 * @note 内部使用硬件隔离层进行资源清理
 */
void algo_envelope_cleanup(algo_envelope_ctx_t *ctx);

/* ========================================================================= *
 * 5. 内部日志辅助函数和宏（安全、空指针检查）
 * ========================================================================= */

// 前向声明内部日志函数
void _algo_log_internal(algo_log_level_t level, const char *tag, const char *fmt, ...);

/**
 * @brief 内部日志宏（仅限算法库内部使用）
 * @details 安全地调用注册的日志回调，处理空指针情况
 */
#define ALGO_LOG_INTERNAL(level, tag, fmt, ...) \
    do { \
        _algo_log_internal(level, tag, fmt, ##__VA_ARGS__); \
    } while (0)

/**
 * @brief 错误日志宏
 */
#define ALGO_LOGE(tag, fmt, ...) \
    ALGO_LOG_INTERNAL(ALGO_LOG_ERROR, tag, fmt, ##__VA_ARGS__)

/**
 * @brief 警告日志宏
 */
#define ALGO_LOGW(tag, fmt, ...) \
    ALGO_LOG_INTERNAL(ALGO_LOG_WARN, tag, fmt, ##__VA_ARGS__)

/**
 * @brief 信息日志宏
 */
#define ALGO_LOGI(tag, fmt, ...) \
    ALGO_LOG_INTERNAL(ALGO_LOG_INFO, tag, fmt, ##__VA_ARGS__)

/**
 * @brief 调试日志宏
 */
#define ALGO_LOGD(tag, fmt, ...) \
    ALGO_LOG_INTERNAL(ALGO_LOG_DEBUG, tag, fmt, ##__VA_ARGS__)
#define ALGO_LOGD(tag, fmt, ...) \
    ALGO_LOG_INTERNAL(ALGO_LOG_DEBUG, tag, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif // ALGO_PDM_H
