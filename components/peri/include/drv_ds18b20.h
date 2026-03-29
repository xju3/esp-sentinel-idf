#ifndef DRV_DS18B20_H_H_
#define DRV_DS18B20_H_H_

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file drv_ds18b20.h
 * @brief DS18B20数字温度传感器驱动
 * 
 * 本驱动提供了DS18B20温度传感器的完整控制接口，支持同步和异步温度读取，
 * 可配置温度分辨率，并包含自检功能。
 * 
 * @note 传感器使用GPIO_NUM_8引脚进行1-Wire通信
 * 
 * 使用示例：
 * @code
 * // 初始化传感器
 * esp_err_t ret = drv_ds18b20_init();
 * if (ret != ESP_OK) {
 *     // 处理错误
 * }
 * 
 * // 设置分辨率（可选，默认为12位）
 * ret = drv_ds18b20_set_resolution(DS18B20_RESOLUTION_12BIT);
 * 
 * // 同步读取温度
 * float temperature;
 * ret = drv_ds18b20_start_conversion();
 * if (ret == ESP_OK) {
 *     while (!drv_ds18b20_is_conversion_done()) {
 *         vTaskDelay(pdMS_TO_TICKS(10));
 *     }
 *     ret = drv_ds18b20_read_temperature(&temperature);
 * }
 * 
 * // 异步读取温度
 * void temperature_callback(float temp) {
 *     printf("Temperature: %.2f°C\n", temp);
 * }
 * 
 * ret = drv_ds18b20_read_temperature_async(temperature_callback);
 * 
 * // 自检
 * ret = drv_ds18b20_self_test();
 * @endcode
 */

// GPIO引脚定义
#define DS18B20_PIN  GPIO_NUM_16

// DS18B20命令定义
#define DS18B20_CMD_SKIP_ROM         0xCC  ///< 跳过ROM匹配，用于单设备总线
#define DS18B20_CMD_CONVERT_T        0x44  ///< 开始温度转换
#define DS18B20_CMD_READ_SCRATCHPAD  0xBE  ///< 读取暂存器
#define DS18B20_CMD_WRITE_SCRATCHPAD 0x4E  ///< 写入暂存器
#define DS18B20_CMD_COPY_SCRATCHPAD  0x48  ///< 复制暂存器到EEPROM
#define DS18B20_CMD_RECALL_E2        0xB8  ///< 从EEPROM恢复暂存器
#define DS18B20_CMD_READ_POWER_SUPPLY 0xB4 ///< 读取电源模式

/**
 * @brief 温度分辨率枚举
 * 
 * 定义了DS18B20支持的四种温度分辨率，分辨率越高转换时间越长。
 */
typedef enum {
    DS18B20_RESOLUTION_9BIT = 0,   ///< 9位分辨率，0.5°C精度，转换时间93.75ms
    DS18B20_RESOLUTION_10BIT = 1,  ///< 10位分辨率，0.25°C精度，转换时间187.5ms
    DS18B20_RESOLUTION_11BIT = 2,  ///< 11位分辨率，0.125°C精度，转换时间375ms
    DS18B20_RESOLUTION_12BIT = 3   ///< 12位分辨率，0.0625°C精度，转换时间750ms
} ds18b20_resolution_t;

/**
 * @brief 温度数据回调函数类型
 * @param temperature_celsius 读取到的温度值（摄氏度）
 */
typedef void (*ds18b20_temp_cb_t)(float temperature_celsius);

/**
 * @brief 初始化DS18B20温度传感器
 * 
 * 配置GPIO引脚，检测传感器是否存在，并设置默认分辨率（12位）。
 * 
 * @return 
 *     - ESP_OK: 初始化成功
 *     - ESP_ERR_NOT_FOUND: 未检测到传感器
 *     - ESP_FAIL: 其他错误
 */
esp_err_t drv_ds18b20_init(void);

/**
 * @brief 设置温度分辨率
 * 
 * 设置DS18B20的温度分辨率，分辨率越高精度越高但转换时间越长。
 * 设置后会保存到传感器的EEPROM中。
 * 
 * @param resolution 分辨率枚举值
 * @return 
 *     - ESP_OK: 设置成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_ERR_INVALID_ARG: 无效的分辨率参数
 *     - ESP_FAIL: 通信错误
 */
esp_err_t drv_ds18b20_set_resolution(ds18b20_resolution_t resolution);

/**
 * @brief 开始温度转换
 * 
 * 发送开始转换命令，转换时间取决于当前设置的分辨率。
 * 转换完成后才能读取温度值。
 * 
 * @return 
 *     - ESP_OK: 转换开始成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_FAIL: 通信错误
 */
esp_err_t drv_ds18b20_start_conversion(void);

/**
 * @brief 检查温度转换是否完成
 * 
 * 根据当前分辨率和转换开始时间判断转换是否完成。
 * 
 * @return 
 *     - true: 转换已完成
 *     - false: 转换未完成或未开始
 */
bool drv_ds18b20_is_conversion_done(void);

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
esp_err_t drv_ds18b20_read_temperature(float *temperature);

/**
 * @brief 异步读取温度值
 * 
 * 注册温度读取回调函数，启动温度转换。
 * 用户需要定期调用drv_ds18b20_is_conversion_done()检查转换状态，
 * 转换完成后会自动调用回调函数。
 * 
 * @param callback 温度读取回调函数
 * @return 
 *     - ESP_OK: 异步读取启动成功
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_ERR_INVALID_ARG: 回调函数为空
 *     - ESP_FAIL: 启动转换失败
 */
esp_err_t drv_ds18b20_read_temperature_async(ds18b20_temp_cb_t callback);

/**
 * @brief 执行传感器自检
 * 
 * 执行完整的传感器功能测试，包括：
 * 1. 总线复位测试
 * 2. 电源模式检测
 * 3. 暂存器读写测试
 * 4. 温度转换测试
 * 
 * @return 
 *     - ESP_OK: 自检通过
 *     - ESP_ERR_INVALID_STATE: 驱动未初始化
 *     - ESP_FAIL: 自检失败
 */
esp_err_t drv_ds18b20_self_test(void);

extern bool g_ds18b20_initialized;

#ifdef __cplusplus
}
#endif

#endif // DRV_DS18B20_H_H_
