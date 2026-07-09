#ifndef BSP_BOARD_H_
#define BSP_BOARD_H_

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Board revision reference: ESP32-S3-N16R8 GPIO allocation table provided on
 * 2026-04-25. Keep all board-level pin ownership here so driver headers only
 * map sensor-specific aliases onto this source of truth.
 */

/* Power and control */
#define BOARD_GPIO_SENSOR_EN       GPIO_NUM_2

/* Analog monitoring */






#ifdef __cplusplus
}
#endif

#endif /* BSP_BOARD_H_ */
