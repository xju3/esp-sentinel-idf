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

// Canonical SPI pin macros used by peri_spi_bus.c
#define PIN_NUM_MISO GPIO_NUM_4
#define PIN_NUM_MOSI GPIO_NUM_5
#define PIN_NUM_CLK  GPIO_NUM_6
#define PIN_NUM_CS   GPIO_NUM_7

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



// 业务下发的硬件配置 (去除了 ODR)
typedef struct {
    icm_fs_t fs;        // 量程
    bool enable_wom;    // 是否开启 Wake-on-Motion (休眠守卫)
    uint16_t wom_thr_mg;// WoM 唤醒阈值 (单位: 毫g，例如 200)
} icm_cfg_t;

extern icm_cfg_t icm42688p_accel_fs_cfg_16;

// 暴露出 HAL 驱动实例，供应用层传入调度器
extern SensorDriver_t icm42688_driver;

/* --- 驱动 API --- */
esp_err_t drv_icm42688_init();
esp_err_t drv_icm42688_config(const icm_cfg_t *cfg);
esp_err_t drv_icm42688_start_stream(imu_data_cb_t cb);
esp_err_t drv_icm42688_stop_stream(void);

/* --- WoM API --- */
esp_err_t enable_icm42688p_wom(uint16_t threshold_mg);
esp_err_t disable_icm42688p_wom(void);
void drv_icm42688_clear_wom_interrupt(void);

#ifdef __cplusplus
}
#endif

#endif // DRV_ICM_42688_P_H_
