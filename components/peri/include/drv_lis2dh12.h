#ifndef DRV_LIS2DH12_H
#define DRV_LIS2DH12_H

#include <stdint.h>
#include <esp_err.h>
#include "imu_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// GPIO assignments
#define LIS2DH12_PIN_NUM_INT1   GPIO_NUM_16
#define LIS2DH12_PIN_NUM_INT2   GPIO_NUM_15
#define LIS2DH12_PIN_NUM_CS     GPIO_NUM_4      // Independent CS (different from ICM-42688-P)
// LIS2DH12: 引脚 9 (SCL/SPC), 11 (SDA/SDI), 12 (SDO/SA0)。
#define LIS2DH12_PIN_NUM_SCL    GPIO_NUM_7     // Shared with ICM-42688-P (SPI2 SCLK)
#define LIS2DH12_PIN_NUM_SDA    GPIO_NUM_6     // SDA就是MOSI，接 ESP32 的 MOSI (GPIO 11)
#define LIS2DH12_PIN_NUM_SDO    GPIO_NUM_5

// Register addresses (exposed for debugging and configuration)
#define LIS2DH12_REG_WHO_AM_I      0x0F
#define LIS2DH12_REG_CTRL_REG1     0x20
#define LIS2DH12_REG_CTRL_REG2     0x21
#define LIS2DH12_REG_CTRL_REG3     0x22
#define LIS2DH12_REG_CTRL_REG4     0x23
#define LIS2DH12_REG_CTRL_REG5     0x24
#define LIS2DH12_REG_CTRL_REG6     0x25
#define LIS2DH12_REG_OUT_X_L       0x28
#define LIS2DH12_REG_INT1_CFG      0x30
#define LIS2DH12_REG_INT1_SRC      0x31
#define LIS2DH12_REG_INT1_THS      0x32
#define LIS2DH12_REG_INT1_DURATION 0x33
#define LIS2DH12_REG_INT2_CFG      0x34
#define LIS2DH12_REG_INT2_SRC      0x35
#define LIS2DH12_REG_INT2_THS      0x36
#define LIS2DH12_REG_INT2_DURATION 0x37
#define LIS2DH12_REG_FIFO_CTRL     0x2E
#define LIS2DH12_REG_FIFO_SRC      0x2F

#define LIS2DH12_WHO_AM_I_VAL      0x33

// Full-scale selection
typedef enum {
    LIS2DH12_FS_2G  = 0,
    LIS2DH12_FS_4G  = 1,
    LIS2DH12_FS_8G  = 2,
    LIS2DH12_FS_16G = 3,
} lis2dh12_fs_t;

// Raw accelerometer data
typedef struct {
    uint8_t header; // 保留字段，对齐 imu_raw_data_t
    int16_t x;      // X轴
    int16_t y;      // Y轴
    int16_t z;      // Z轴
    int8_t  temp;   // 保留字段，对齐 imu_raw_data_t
} __attribute__((packed)) lis2dh12_raw_data_t;

// Data callback function pointer
typedef void (*lis2dh12_data_cb_t)(const lis2dh12_raw_data_t *data, size_t count);

// WoM configuration
typedef struct {
    lis2dh12_fs_t fs;        // Full-scale used in WoM mode (affects INT_THS LSB)
    uint16_t threshold_mg_int1; // Threshold for INT1 in mg
    uint16_t threshold_mg_int2; // Threshold for INT2 in mg
    uint8_t duration_int1;      // Duration for INT1 event
    uint8_t duration_int2;      // Duration for INT2 event
} lis2dh12_wom_cfg_t;

// Public Functions
esp_err_t drv_lis2dh12_init(void);
esp_err_t drv_lis2dh12_enable_wom(const lis2dh12_wom_cfg_t *wom_cfg);
esp_err_t drv_lis2dh12_disable_wom(void);
esp_err_t drv_lis2dh12_set_config(lis2dh12_fs_t fs, float odr);
esp_err_t drv_lis2dh12_get_raw_data(lis2dh12_raw_data_t *data);

lis2dh12_fs_t drv_lis2dh12_get_current_fs(void);
esp_err_t drv_lis2dh12_read_int1_source(uint8_t *source);
esp_err_t drv_lis2dh12_read_int2_source(uint8_t *source);

// debug helper: read arbitrary register (non-atomic)
esp_err_t drv_lis2dh12_read_register(uint8_t reg, uint8_t *value);

// Sensor driver instance for imu_config
extern SensorDriver_t lis2dh12_driver;

#ifdef __cplusplus
}
#endif

#endif // DRV_LIS2DH12_H
