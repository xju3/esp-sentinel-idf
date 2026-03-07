#ifndef DRV_LIS2DH12_H
#define DRV_LIS2DH12_H

#include <stdint.h>
#include <esp_err.h>
#include "imu_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// GPIO assignments
#define LIS2DH12_PIN_NUM_INT1   GPIO_NUM_5
#define LIS2DH12_PIN_NUM_INT2   GPIO_NUM_6
#define LIS2DH12_PIN_NUM_CS     GPIO_NUM_7
#define LIS2DH12_PIN_NUM_SCL    GPIO_NUM_8
#define LIS2DH12_PIN_NUM_SDO    GPIO_NUM_15
#define LIS2DH12_PIN_NUM_SDA    GPIO_NUM_16

// Full-scale selection
typedef enum {
    LIS2DH12_FS_2G  = 0,
    LIS2DH12_FS_4G  = 1,
    LIS2DH12_FS_8G  = 2,
    LIS2DH12_FS_16G = 3,
} lis2dh12_fs_t;

// Raw accelerometer data
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lis2dh12_raw_data_t;

// Data callback function pointer
typedef void (*lis2dh12_data_cb_t)(const lis2dh12_raw_data_t *data, size_t count);

// WoM configuration
typedef struct {
    uint8_t threshold_mg_int1; // Threshold for INT1 in mg
    uint8_t threshold_mg_int2; // Threshold for INT2 in mg
    uint8_t duration_int1;     // Duration for INT1 event
    uint8_t duration_int2;     // Duration for INT2 event
} lis2dh12_wom_cfg_t;

// Public Functions
esp_err_t drv_lis2dh12_init(void);
esp_err_t drv_lis2dh12_self_test(void);
esp_err_t drv_lis2dh12_enable_wom(const lis2dh12_wom_cfg_t *wom_cfg);
esp_err_t drv_lis2dh12_disable_wom(void);
esp_err_t drv_lis2dh12_set_config(lis2dh12_fs_t fs, float odr);
esp_err_t drv_lis2dh12_get_raw_data(lis2dh12_raw_data_t *data);
esp_err_t drv_lis2dh12_capture(uint32_t duration_ms, lis2dh12_data_cb_t cb);

esp_err_t drv_lis2dh12_start_fifo_capture(uint8_t watermark_level, lis2dh12_data_cb_t cb);
esp_err_t drv_lis2dh12_stop_fifo_capture(void);

// Sensor driver instance for imu_config
extern SensorDriver_t lis2dh12_driver;

#ifdef __cplusplus
}
#endif

#endif // DRV_LIS2DH12_H