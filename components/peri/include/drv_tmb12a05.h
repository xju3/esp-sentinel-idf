#ifndef DRV_TMB12A05_H
#define DRV_TMB12A05_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file drv_tmb12a05.h
 * @brief TMB12A05蜂鸣器驱动
 * 
 * 本驱动提供了TMB12A05有源蜂鸣器的完整控制接口，支持多种蜂鸣模式：
 * - 单次蜂鸣
 * - 双次蜂鸣
 * - 三次蜂鸣
 * - 连续蜂鸣
 * - 常开模式
 * 
 * @note 蜂鸣器使用GPIO_NUM_3引脚控制，支持高电平/低电平触发配置
 * 
 * 使用示例：
 * @code
 * // 初始化蜂鸣器
 * esp_err_t ret = drv_tmb12a05_init();
 * if (ret != ESP_OK) {
 *     // 处理错误
 * }
 * 
 * // 配置蜂鸣器参数
 * tmb12a05_config_t config = {
 *     .beep_duration_ms = 100,
 *     .beep_interval_ms = 100,
 *     .beep_count = 2,
 *     .active_high = true
 * };
 * ret = drv_tmb12a05_set_config(&config);
 * 
 * // 单次蜂鸣（100ms）
 * ret = drv_tmb12a05_beep_once(100);
 * 
 * // 多次蜂鸣（3次，每次80ms，间隔100ms）
 * ret = drv_tmb12a05_beep_multiple(3, 80, 100);
 * 
 * // 连续蜂鸣（200ms周期，持续1秒）
 * ret = drv_tmb12a05_start_continuous(200);
 * vTaskDelay(pdMS_TO_TICKS(1000));
 * ret = drv_tmb12a05_stop();
 * 
 * // 设置模式
 * ret = drv_tmb12a05_set_mode(TMB12A05_MODE_BEEP_DOUBLE);
 * 
 * // 注册状态回调
 * void buzzer_state_callback(tmb12a05_mode_t mode) {
 *     printf("Buzzer mode changed to: %d\n", mode);
 * }
 * ret = drv_tmb12a05_register_state_callback(buzzer_state_callback);
 * 
 * // 自检
 * ret = drv_tmb12a05_self_test();
 * 
 * // 反初始化
 * ret = drv_tmb12a05_deinit();
 * @endcode
 */

// GPIO引脚定义 - 根据用户要求使用GPIO_NUM=3
#define TMB12A05_PIN  GPIO_NUM_3

/**
 * @brief 蜂鸣器工作模式枚举
 * 
 * 定义了蜂鸣器支持的六种工作模式。
 */
typedef enum {
    TMB12A05_MODE_OFF = 0,      ///< 关闭模式，蜂鸣器不发声
    TMB12A05_MODE_ON,           ///< 常开模式，蜂鸣器持续发声直到手动关闭
    TMB12A05_MODE_BEEP_SINGLE,  ///< 单次蜂鸣模式，蜂鸣一次后自动关闭
    TMB12A05_MODE_BEEP_DOUBLE,  ///< 双次蜂鸣模式，蜂鸣两次后自动关闭
    TMB12A05_MODE_BEEP_TRIPLE,  ///< 三次蜂鸣模式，蜂鸣三次后自动关闭
    TMB12A05_MODE_BEEP_CONTINUOUS ///< 连续蜂鸣模式，按照配置周期连续蜂鸣
} tmb12a05_mode_t;

/**
 * @brief 蜂鸣器配置结构
 * 
 * 包含蜂鸣器的所有可配置参数。
 */
typedef struct {
    uint16_t beep_duration_ms;      ///< 单次蜂鸣持续时间（毫秒），范围1-5000ms
    uint16_t beep_interval_ms;      ///< 蜂鸣间隔时间（毫秒），范围0-10000ms
    uint8_t beep_count;             ///< 蜂鸣次数（用于单次/双次/三次模式），范围1-10
    bool active_high;               ///< 触发电平：true为高电平触发，false为低电平触发
} tmb12a05_config_t;

/**
 * @brief 蜂鸣器状态回调函数类型
 * @param current_mode 当前蜂鸣器模式
 */
typedef void (*tmb12a05_state_cb_t)(tmb12a05_mode_t current_mode);

/**
 * @brief 初始化蜂鸣器驱动
 * 
 * 配置GPIO引脚，创建蜂鸣任务和定时器，设置默认配置。
 * 默认配置：蜂鸣持续时间100ms，间隔100ms，高电平触发。
 * 
 * @return 
 *     - ESP_OK: 初始化成功
 *     - ESP_FAIL: GPIO配置失败或任务创建失败
 */
esp_err_t drv_tmb12a05_init(void);

/**
 * @brief 反初始化蜂鸣器驱动
 * 
 * 停止所有蜂鸣，删除任务和定时器，释放资源。
 * 
 * @return 
 *     - ESP_OK: 反初始化成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 */
esp_err_t drv_tmb12a05_deinit(void);

/**
 * @brief 设置蜂鸣器工作模式
 * 
 * 设置蜂鸣器的工作模式，支持六种预定义模式。
 * 
 * @param mode 蜂鸣器模式枚举值
 * @return 
 *     - ESP_OK: 模式设置成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_ERR_INVALID_ARG: 无效的模式参数
 */
esp_err_t drv_tmb12a05_set_mode(tmb12a05_mode_t mode);

/**
 * @brief 设置蜂鸣器配置参数
 * 
 * 设置蜂鸣器的详细配置参数，包括持续时间、间隔、次数和触发电平。
 * 
 * @param config 配置结构体指针
 * @return 
 *     - ESP_OK: 配置设置成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_ERR_INVALID_ARG: 参数为空或参数值无效
 */
esp_err_t drv_tmb12a05_set_config(const tmb12a05_config_t *config);

/**
 * @brief 获取当前蜂鸣器配置
 * 
 * 获取当前蜂鸣器的配置参数。
 * 
 * @param[out] config 存储配置的结构体指针
 * @return 
 *     - ESP_OK: 获取成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_ERR_INVALID_ARG: 参数为空
 */
esp_err_t drv_tmb12a05_get_config(tmb12a05_config_t *config);

/**
 * @brief 获取当前蜂鸣器模式
 * 
 * 获取蜂鸣器当前的工作模式。
 * 
 * @return 当前蜂鸣器模式
 */
tmb12a05_mode_t drv_tmb12a05_get_current_mode(void);

/**
 * @brief 执行单次蜂鸣
 * 
 * 执行一次指定持续时间的蜂鸣，蜂鸣完成后自动停止。
 * 
 * @param duration_ms 蜂鸣持续时间（毫秒），范围1-5000ms
 * @return 
 *     - ESP_OK: 蜂鸣启动成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_ERR_INVALID_ARG: 持续时间无效
 */
esp_err_t drv_tmb12a05_beep_once(uint16_t duration_ms);

/**
 * @brief 执行多次蜂鸣
 * 
 * 执行指定次数的蜂鸣，可配置每次蜂鸣的持续时间和间隔时间。
 * 
 * @param count 蜂鸣次数，范围1-10
 * @param duration_ms 每次蜂鸣的持续时间（毫秒），范围1-5000ms
 * @param interval_ms 蜂鸣间隔时间（毫秒），范围0-10000ms
 * @return 
 *     - ESP_OK: 蜂鸣启动成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_ERR_INVALID_ARG: 参数无效
 */
esp_err_t drv_tmb12a05_beep_multiple(uint8_t count, uint16_t duration_ms, uint16_t interval_ms);

/**
 * @brief 启动连续蜂鸣模式
 * 
 * 启动连续蜂鸣模式，蜂鸣器按照指定周期（开启时间+关闭时间）连续蜂鸣。
 * 开启时间和关闭时间各占周期的一半。
 * 
 * @param period_ms 蜂鸣周期（毫秒），范围50-5000ms
 * @return 
 *     - ESP_OK: 连续蜂鸣启动成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_ERR_INVALID_ARG: 周期参数无效
 */
esp_err_t drv_tmb12a05_start_continuous(uint16_t period_ms);

/**
 * @brief 停止蜂鸣器
 * 
 * 立即停止所有蜂鸣，将蜂鸣器设置为关闭模式。
 * 
 * @return 
 *     - ESP_OK: 停止成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 */
esp_err_t drv_tmb12a05_stop(void);

/**
 * @brief 注册蜂鸣器状态变化回调函数
 * 
 * 注册回调函数，当蜂鸣器模式发生变化时会调用该函数。
 * 
 * @param callback 状态变化回调函数
 * @return 
 *     - ESP_OK: 注册成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 */
esp_err_t drv_tmb12a05_register_state_callback(tmb12a05_state_cb_t callback);

/**
 * @brief 执行蜂鸣器自检
 * 
 * 执行完整的蜂鸣器功能测试，包括：
 * 1. 单次蜂鸣测试
 * 2. 双次蜂鸣测试
 * 3. 常开模式测试
 * 4. 停止功能测试
 * 5. 连续蜂鸣模式测试
 * 
 * @return 
 *     - ESP_OK: 自检通过
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_FAIL: 自检失败
 */
esp_err_t drv_tmb12a05_self_test(void);

#ifdef __cplusplus
}
#endif

#endif // DRV_TMB12A05_H
