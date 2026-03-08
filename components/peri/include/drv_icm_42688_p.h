/**
 * @file drv_icm_42688_p.h
 * @brief ICM-42688-P 纯加速度/高频振动采集驱动 (异步 DMA 流模式)
 */
#ifndef DRV_ICM_42688_P_H_
#define DRV_ICM_42688_P_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_config.h" // 引入 HAL 层抽象结构

#define PIN_NUM_MISO GPIO_NUM_9
#define PIN_NUM_MOSI GPIO_NUM_11
#define PIN_NUM_CLK  GPIO_NUM_12
#define PIN_NUM_CS   GPIO_NUM_10

// WoM 核心寄存器宏定义
#define ICM42688P_REG_PWR_MGMT0     0x4E
#define ICM42688P_REG_INT_CONFIG    0x14
#define ICM42688P_REG_INT_SOURCE0   0x65
#define ICM42688P_REG_INT_SOURCE1   0x66
#define ICM42688P_REG_WOM_CONFIG    0x56
#define ICM42688P_REG_SMD_CONFIG    0x57

#ifdef __cplusplus
extern "C" {
#endif

// 满量程 (FS) 枚举
typedef enum {
    ICM_FS_16G = 0x00, 
    ICM_FS_8G  = 0x01,
    ICM_FS_4G  = 0x02,
    ICM_FS_2G  = 0x03  
} icm_fs_t;

// 原始数据包结构 (DMA 直接灌装)
typedef struct {
    uint8_t header; // 数据包头
    int16_t x;      // 原始X轴
    int16_t y;      // 原始Y轴
    int16_t z;      // 原始Z轴
    int8_t  temp;   // 8位截断温度辅助数据
} __attribute__((packed)) imu_raw_data_t;

// 业务下发的硬件配置 (去除了 ODR)
typedef struct {
    icm_fs_t fs;        // 量程
    bool enable_wom;    // 是否开启 Wake-on-Motion (休眠守卫)
    uint16_t wom_thr_mg;// WoM 唤醒阈值 (单位: 毫g，例如 200)
} icm_cfg_t;

// DMA 批量数据就绪回调函数签名
typedef void (*icm_data_cb_t)(const imu_raw_data_t *data, size_t count);

// 暴露出 HAL 驱动实例，供应用层传入调度器
extern SensorDriver_t icm42688_driver;

/* --- 驱动 API --- */
esp_err_t drv_icm42688_init(void);
esp_err_t drv_icm42688_config(const icm_cfg_t *cfg);
esp_err_t drv_icm42688_start_stream(icm_data_cb_t cb);
esp_err_t drv_icm42688_stop_stream(void);

/* --- WoM API --- */
esp_err_t enable_icm42688p_wom(uint16_t threshold_mg);
esp_err_t disable_icm42688p_wom(void);
void drv_icm42688_clear_wom_interrupt(void);

#ifdef __cplusplus
}
#endif

#endif // DRV_ICM_42688_P_H_