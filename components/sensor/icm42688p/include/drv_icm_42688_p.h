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

#ifdef __cplusplus
extern "C"
{
#endif

    /* ========================================================================= *
     * 1. 核心数据结构定义 (纯数据，无业务逻辑)
     * ========================================================================= */

    /**
     * @brief 三轴加速度原始数据 (Raw Data)
     * @note 必须保持为 int16_t 以最小化 DMA 搬运带宽和 PSRAM 占用
     * __attribute__((packed)), 结构体上加了这个修饰符。
     * 因为 C 语言编译器有时会为了内存对齐而在结构体里塞入空白字节（Padding）。
     * 加了强制紧凑修饰后，确保结构体大小永远是绝对的 6 个字节（X,Y,Z 各 2 字节）。
     * 这对于 DMA 直接把 SPI 硬件流映射到内存结构体至关重要，哪怕错位 1 个字节，整个波形就全毁了。
     */
    typedef struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    } __attribute__((packed)) icm_raw_data_t;

    /**
     * @brief 采样率 (ODR) 枚举
     */
    typedef enum
    {
        ICM_ODR_1KHZ = 0x05, // 适合常规设备状态监测
        ICM_ODR_4KHZ = 0x04, // 适合低速电机监测
        ICM_ODR_8KHZ = 0x03, // 适合常规轴承异常频段
        ICM_ODR_16KHZ = 0x02 // 适合极早期微小故障/齿轮啮合高频分析
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
     * 这个由驱动底层传上来的 DMA 内存块是“只读”的，
     * 上层（如 task_monitor）只能把它 Copy 走，绝对不能在这块极速交替的内存里原地做修改。
     */
    typedef void (*icm_data_cb_t)(const icm_raw_data_t *data, size_t count);

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

#ifdef __cplusplus
}
#endif

#endif // DRV_ICM_42688_P_H_