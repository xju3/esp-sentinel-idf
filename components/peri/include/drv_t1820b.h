#ifndef DRV_T1820B_H_H_
#define DRV_T1820B_H_H_

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file drv_t1820b.h
 * @brief T1820B数字温度传感器驱动
 *
 * 本驱动保留现有1-Wire GPIO时序框架，并按T1820B协议完成：
 * 1. READ ROM (0x33) + ROM CRC 校验
 * 2. SKIP ROM (0xCC) + CONVERT T (0x44)
 * 3. SKIP ROM + READ TEMPERATURE (0xBC)
 * 4. 温度 CRC 校验与温度换算
 */

// GPIO引脚定义
#define T1820B_PIN GPIO_NUM_16

// T1820B单总线命令定义
#define T1820B_CMD_READ_ROM          0x33
#define T1820B_CMD_SKIP_ROM          0xCC
#define T1820B_CMD_CONVERT_T         0x44
#define T1820B_CMD_READ_TEMPERATURE  0xBC
#define T1820B_CMD_READ_SCRATCHPAD   0xBE
#define T1820B_CMD_SOFT_RESET        0x6A
#define T1820B_CMD_HEAT_OFF          0x92

/**
 * @brief 温度数据回调函数类型
 * @param temperature_celsius 读取到的温度值（摄氏度）
 */
typedef void (*t1820b_temp_cb_t)(float temperature_celsius);

/**
 * @brief 初始化T1820B温度传感器
 *
 * 配置GPIO引脚并尝试检测传感器。
 *
 * @return
 *     - ESP_OK: 初始化成功
 *     - ESP_ERR_NOT_FOUND: 未检测到传感器
 *     - ESP_FAIL: 其他错误
 */
esp_err_t drv_t1820b_init(void);

/**
 * @brief 开始温度转换
 *
 * 执行 READ ROM + CRC 校验，再发送 SKIP ROM + CONVERT T。
 *
 * @return
 *     - ESP_OK: 转换开始成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_FAIL: 通信错误
 */
esp_err_t drv_t1820b_start_conversion(void);

/**
 * @brief 检查温度转换是否完成
 *
 * @return
 *     - true: 转换已完成
 *     - false: 转换未完成或未开始
 */
bool drv_t1820b_is_conversion_done(void);

/**
 * @brief 读取温度值（同步）
 *
 * 温度换算公式：
 *     temp_c = ((int16_t)raw_temp / 256.0f) + 25.0f
 *
 * @param[out] temperature 存储读取到的温度值（摄氏度）
 * @return
 *     - ESP_OK: 读取成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_ERR_INVALID_ARG: 参数为空指针
 *     - ESP_FAIL: 通信错误或CRC校验失败
 */
esp_err_t drv_t1820b_read_temperature(float *temperature);

/**
 * @brief 异步读取温度值
 *
 * 注册温度读取回调函数，启动温度转换。
 * 用户需要定期调用drv_t1820b_is_conversion_done()检查转换状态，
 * 并在转换完成后调用drv_t1820b_read_temperature()。
 *
 * @param callback 温度读取回调函数
 * @return
 *     - ESP_OK: 异步读取启动成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_ERR_INVALID_ARG: 回调函数为空
 *     - ESP_FAIL: 启动转换失败
 */
esp_err_t drv_t1820b_read_temperature_async(t1820b_temp_cb_t callback);

/**
 * @brief 执行传感器自检
 *
 * 执行完整的传感器功能测试，包括：
 * 1. 总线复位测试
 * 2. ROM 读取与 CRC 校验
 * 3. 温度转换测试
 * 4. 温度读取与 CRC 校验
 *
 * @return
 *     - ESP_OK: 自检通过
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_FAIL: 自检失败
 */
esp_err_t drv_t1820b_self_test(void);

extern bool g_t1820b_initialized;

#ifdef __cplusplus
}
#endif

#endif // DRV_T1820B_H_H_
