#ifndef DRV_DS18B20_H_H_
#define DRV_DS18B20_H_H_

#include <stdint.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// 业务层用来接收数据的 回调合同
#define DS18B20_PIN  GPIO_NUM_46

#ifdef __cplusplus
}
#endif

#endif // DRV_DS18B20_H_H_