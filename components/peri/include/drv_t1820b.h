#ifndef DRV_T1820B_H_
#define DRV_T1820B_H_

#include <stdint.h>
#include <stdbool.h>
#include "bsp_board.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file drv_t1820b.h
 * @brief 1-Wire温度传感器驱动（当前板卡为 T1820B）
 * 
 * @note 板级引脚分配表指定温度传感器 DQ 使用 GPIO16
 * 
 * 使用示例：
 * @code
 * // 初始化传感器
 * esp_err_t ret = drv_t1820b_init();
 * if (ret != ESP_OK) {
 *     // 处理错误
 * }
 * 
 * // 设置兼容分辨率等级（内部映射到 T1820B 的平均次数）
 * ret = drv_t1820b_set_resolution(T1820B_RESOLUTION_12BIT);
 * 
 * // 同步读取温度
 * float temperature;
 * ret = drv_t1820b_start_conversion();
 * if (ret == ESP_OK) {
 *     while (!drv_t1820b_is_conversion_done()) {
 *         vTaskDelay(pdMS_TO_TICKS(10));
 *     }
 *     ret = drv_t1820b_read_temperature(&temperature);
 * }
 * 
 * // 异步读取温度
 * void temperature_callback(float temp) {
 *     printf("Temperature: %.2f°C\n", temp);
 * }
 * 
 * ret = drv_t1820b_read_temperature_async(temperature_callback);
 * 
 * // 自检
 * ret = drv_t1820b_self_test();
 * @endcode
 */

// GPIO引脚定义
#define T1820B_PIN  BOARD_GPIO_T1820B_DQ

// T1820B 1-Wire 命令定义
#define T1820B_CMD_SKIP_ROM          0xCC  ///< 跳过 ROM 匹配，用于单设备总线
#define T1820B_CMD_CONVERT_T         0x44  ///< 开始温度转换
#define T1820B_CMD_READ_TEMPERATURE  0xBC  ///< 读取温度 2 字节 + CRC
#define T1820B_CMD_READ_SCRATCHPAD   0xBE  ///< 读取 0x03~0x0B Scratch 寄存器
#define T1820B_CMD_WRITE_SCRATCHPAD  0x4E  ///< 写入 0x04~0x0A 配置寄存器
#define T1820B_CMD_COPY_SCRATCHPAD   0x48  ///< 复制寄存器到 E2PROM
#define T1820B_CMD_RECALL_E2         0xB8  ///< 从 E2PROM 装载全部寄存器
#define T1820B_CMD_BREAK             0x93  ///< 退出连续测量模式
#define T1820B_CMD_SOFT_RESET        0x6A  ///< 软复位并重装载 E2PROM

/**
 * @brief 温度分辨率枚举
 * 
 * 历史兼容枚举。T1820B 本身不是 DS18B20 的 9/10/11/12-bit 分辨率模型，
 * 这里将四档映射为 AVG=1/8/16/32，对应约 2.2/5.2/8.5/15.3ms 转换时间。
 */
typedef enum {
    T1820B_RESOLUTION_9BIT = 0,   ///< 映射到 T1820B AVG=1，约 2.2ms
    T1820B_RESOLUTION_10BIT = 1,  ///< 映射到 T1820B AVG=8，约 5.2ms
    T1820B_RESOLUTION_11BIT = 2,  ///< 映射到 T1820B AVG=16，约 8.5ms
    T1820B_RESOLUTION_12BIT = 3   ///< 映射到 T1820B AVG=32，约 15.3ms
} t1820b_resolution_t;

/**
 * @brief 温度数据回调函数类型
 * @param temperature_celsius 读取到的温度值（摄氏度）
 */
typedef void (*t1820b_temp_cb_t)(float temperature_celsius);

/**
 * @brief 初始化 T1820B 温度传感器
 * 
 * 配置 GPIO 引脚，检测传感器是否存在，并设置默认兼容分辨率档位。
 * 
 * @return 
 *     - ESP_OK: 初始化成功
 *     - ESP_ERR_NOT_FOUND: 未检测到传感器
 *     - ESP_FAIL: 其他错误
 */
esp_err_t drv_t1820b_init(void);

/**
 * @brief 设置兼容分辨率档位
 * 
 * 设置兼容分辨率档位。T1820B 内部会映射到不同 AVG 配置，并保存到 E2PROM。
 * 
 * @param resolution 兼容分辨率枚举值
 * @return 
 *     - ESP_OK: 设置成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_ERR_INVALID_ARG: 无效的分辨率参数
 *     - ESP_FAIL: 通信错误
 */
esp_err_t drv_t1820b_set_resolution(t1820b_resolution_t resolution);

/**
 * @brief 开始温度转换
 * 
 * 发送开始转换命令，转换时间取决于当前兼容分辨率档位。
 * 转换完成后才能读取温度值。
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
 * 根据当前兼容分辨率档位和转换开始时间判断转换是否完成。
 * 
 * @return 
 *     - true: 转换已完成
 *     - false: 转换未完成或未开始
 */
bool drv_t1820b_is_conversion_done(void);

/**
 * @brief 读取温度值（同步）
 * 
 * 读取当前温度值，如果转换未完成会等待直到转换完成。
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
 * 用户需要定期调用 drv_t1820b_is_conversion_done() 检查转换状态，
 * 转换完成后会自动调用回调函数。
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
 * 2. Scratch 寄存器读取测试
 * 3. 温度转换测试
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

#endif // DRV_T1820B_H_
