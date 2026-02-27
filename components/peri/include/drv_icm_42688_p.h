/**
 * @file drv_icm_42688_p.h
 * @brief ICM-42688-P 纯加速度/高频振动采集驱动 (异步 DMA 流模式)
 * * 架构说明:
 * 1. 彻底禁用陀螺仪以节省功耗。
 * 2. 采用硬件 FIFO + SPI DMA 乒乓缓存架构，释放 CPU 算力。
 * 3. 驱动层仅输出 Raw Data (int16_t)，浮点转换留给 DSP/算法层处理。
 */

#ifndef DRV_ICM_42688_P_H_
#define DRV_ICM_42688_P_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "algo_pdm.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* ========================================================================= *
     * 1. 核心数据结构定义 (纯数据，无业务逻辑)
     * ========================================================================= */

    // Note: imu_raw_data_t is now defined in algo_types.h
    // This allows the algorithm layer to be independent of hardware-specific types

    /**
     * @brief 采样率 (ODR) 枚举
     */
    // ACCEL_ODR[3:0] 寄存器位定义 (Bank 0, 地址 0x50)
    typedef enum
    {
        ICM_ODR_RESERVED = 0x00, // 保留
        ICM_ODR_32KHZ = 0x01,    // 32 kHz (LN mode)
        ICM_ODR_16KHZ = 0x02,    // 16 kHz (LN mode)
        ICM_ODR_8KHZ = 0x03,     // 8 kHz (LN mode)
        ICM_ODR_4KHZ = 0x04,     // 4 kHz (LN mode)
        ICM_ODR_2KHZ = 0x05,     // 2 kHz (LN mode)
        ICM_ODR_1KHZ = 0x06,     // 1 kHz (LN mode, 默认值)
        ICM_ODR_500HZ = 0x0F,    // 500 Hz (LP or LN mode)
        ICM_ODR_200HZ = 0x07,    // 200 Hz (LP or LN mode)
        ICM_ODR_100HZ = 0x08,    // 100 Hz (LP or LN mode)
        ICM_ODR_50HZ = 0x09,     // 50 Hz (LP or LN mode)
        ICM_ODR_25HZ = 0x0A,     // 25 Hz (LP or LN mode)
        ICM_ODR_12_5HZ = 0x0B,   // 12.5 Hz (LP or LN mode)
        ICM_ODR_6_25HZ = 0x0C,   // 6.25 Hz (LP mode only)
        ICM_ODR_3_125HZ = 0x0D,  // 3.125 Hz (LP mode only)
        ICM_ODR_1_5625HZ = 0x0E, // 1.5625 Hz (LP mode only)
    } icm_odr_t;

    /**
     * @brief 满量程 (FS) 枚举
     */
    typedef enum
    {
        ICM_FS_16G = 0x00, // 工业高频测振首选
        ICM_FS_8G = 0x01,
        ICM_FS_4G = 0x02,
        ICM_FS_2G = 0x03 // 适合纯姿态/微小位移监控
    } icm_fs_t;

    /**
     * @brief 驱动动态配置参数
     */
    typedef struct
    {
        icm_odr_t odr;      // 采样频率
        icm_fs_t fs;        // 量程
        bool enable_wom;    // 是否开启 Wake-on-Motion (休眠守卫)
        uint8_t wom_thr_mg; // WoM 唤醒阈值 (单位: 毫g，例如 200)
    } icm_cfg_t;

    /* ========================================================================= *
     * 2. 异步回调契约 (The Contract)
     * ========================================================================= */

    /**
     * @brief DMA 批量数据就绪回调函数签名
     * @param data 指向内部 SRAM 乒乓缓存中 Raw Data 数组的指针
     * @param count 本次 DMA 成功搬运的采样点数量 (通常为 128 或 256)
     * @note 该回调运行在极高优先级任务或 ISR 上下文中，严禁在此执行 printf 或大运算！
     * 上层只需将 data 块 memcpy 到 PSRAM 的 RingBuffer 中即可。
     * 明确告诉上层业务逻辑：
     * 这个由驱动底层传上来的 DMA 内存块是"只读"的，
     * 上层（如 task_monitor）只能把它 Copy 走，绝对不能在这块极速交替的内存里原地做修改。
     */
    typedef void (*icm_data_cb_t)(const imu_raw_data_t *data, size_t count);

    /* ========================================================================= *
     * 3. 驱动层 API (三阶段状态机)
     * ========================================================================= */

    /**
     * @brief 阶段一：基础物理初始化
     * @details 挂载 SPI 总线，分配 DMA 内存，软复位 IMU，验证 WHO_AM_I。
     * 整个系统生命周期仅需调用一次。
     * @return ESP_OK 成功; ESP_ERR_NOT_FOUND 芯片未找到
     */
    esp_err_t drv_icm42688_init(void);

    /**
     * @brief 阶段二：动态业务配置
     * @details 将抽象的 fs/odr 翻译为寄存器指令下发。
     * 强制关闭陀螺仪电源。如果配置了 WoM，则一并写入阈值。
     * @param cfg 配置参数结构体指针
     * @return ESP_OK 成功
     */
    esp_err_t drv_icm42688_config(const icm_cfg_t *cfg);

    /**
     * @brief 阶段三：开启异步数据流
     * @details 清空 FIFO 残留，启动 SPI DMA 硬件无限循环搬运。
     * @param cb 数据满水位的回调函数
     * @return ESP_OK 成功
     */
    esp_err_t drv_icm42688_start_stream(icm_data_cb_t cb);

    /**
     * @brief 阶段三：停止异步数据流
     * @details 停止 DMA 搬运，将 IMU 切入低功耗模式或 WoM 守卫模式。
     * @return ESP_OK 成功
     */
    esp_err_t drv_icm42688_stop_stream(void);

    /**
     * 巡逻阶段的ODR
     */
    icm_odr_t calculate_patrol_odr(float rpm);
#ifdef __cplusplus
}
#endif

#endif // DRV_ICM_42688_P_H_
