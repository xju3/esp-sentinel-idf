#ifndef DRV_IIS3DWB_H_H_
#define DRV_IIS3DWB_H_H_

#include <stdint.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// 业务层用来接收数据的 回调合同
#define IIS3DWB_PIN_NUM_INT1   GPIO_NUM_8
#define IIS3DWB_PIN_NUM_INT2   GPIO_NUM_3
#define IIS3DWB_PIN_NUM_CS     GPIO_NUM_9
#define IIS3DWB_PIN_NUM_SDO    GPIO_NUM_10
#define IIS3DWB_PIN_NUM_SDA    GPIO_NUM_11
#define IIS3DWB_PIN_NUM_SCL    GPIO_NUM_12

#ifdef __cplusplus
}
#endif

#endif // DRV_IIS3DWB_H_H_