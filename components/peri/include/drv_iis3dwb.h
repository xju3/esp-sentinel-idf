#ifndef DRV_IIS3DWB_H_H_
#define DRV_IIS3DWB_H_H_

#include <stdint.h>
#include <stddef.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "imu_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// 业务层用来接收数据的 回调合同
// #define IIS3DWB_PIN_NUM_INT1   GPIO_NUM_8
// #define IIS3DWB_PIN_NUM_INT2   GPIO_NUM_3
#define IIS3DWB_PIN_NUM_CS     GPIO_NUM_4
#define IIS3DWB_PIN_NUM_SCL    GPIO_NUM_5
#define IIS3DWB_PIN_NUM_SDA    GPIO_NUM_6
#define IIS3DWB_PIN_NUM_SDO    GPIO_NUM_7

// Register addresses (for debug/config)
#define IIS3DWB_REG_WHO_AM_I          0x0F
#define IIS3DWB_REG_CTRL1_XL          0x10
#define IIS3DWB_REG_CTRL3_C           0x12
#define IIS3DWB_REG_CTRL6_C           0x15
#define IIS3DWB_REG_FIFO_CTRL1        0x07
#define IIS3DWB_REG_FIFO_CTRL2        0x08
#define IIS3DWB_REG_FIFO_CTRL3        0x09
#define IIS3DWB_REG_FIFO_CTRL4        0x0A
#define IIS3DWB_REG_FIFO_STATUS1      0x3A
#define IIS3DWB_REG_FIFO_STATUS2      0x3B
#define IIS3DWB_REG_FIFO_DATA_OUT_TAG 0x78

#define IIS3DWB_WHO_AM_I_VAL 0x7B

typedef enum {
    IIS3DWB_FS_2G  = 0x00, // 00b
    IIS3DWB_FS_16G = 0x01, // 01b
    IIS3DWB_FS_4G  = 0x02, // 10b
    IIS3DWB_FS_8G  = 0x03  // 11b
} iis3dwb_fs_t;

typedef struct {
    iis3dwb_fs_t fs;
} iis3dwb_cfg_t;


// Sensor driver instance for imu_config
extern SensorDriver_t iis3dwb_driver;

esp_err_t drv_iis3dwb_init(void);
esp_err_t drv_iis3dwb_config(const iis3dwb_cfg_t *cfg);
esp_err_t drv_iis3dwb_start_stream(imu_data_cb_t cb);
esp_err_t drv_iis3dwb_start_stream_ex(imu_data_cb_ctx_t cb, void *user_ctx);
esp_err_t drv_iis3dwb_stop_stream(void);

#ifdef __cplusplus
}
#endif

#endif // DRV_IIS3DWB_H_H_
