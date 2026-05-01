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
#define BOARD_GPIO_CHARGE_PWM      GPIO_NUM_1
#define BOARD_GPIO_SENSOR_EN       GPIO_NUM_2
#define BOARD_GPIO_4G_PWR_EN       GPIO_NUM_13
#define BOARD_GPIO_LED             GPIO_NUM_40

/* Analog monitoring */
#define BOARD_GPIO_ADC_BATTERY     GPIO_NUM_42
#define BOARD_GPIO_ADC_SUPERCAP    GPIO_NUM_41

/* LIS2DH12 */
#define BOARD_GPIO_LIS2DH12_SCL    GPIO_NUM_12
#define BOARD_GPIO_LIS2DH12_CS     GPIO_NUM_9
#define BOARD_GPIO_LIS2DH12_MISO   GPIO_NUM_10
#define BOARD_GPIO_LIS2DH12_MOSI   GPIO_NUM_11
#define BOARD_GPIO_LIS2DH12_INT1   GPIO_NUM_8
#define BOARD_GPIO_LIS2DH12_INT2   GPIO_NUM_3

/* IIS3DWB */
#define BOARD_GPIO_IIS3DWB_SCL     GPIO_NUM_6
#define BOARD_GPIO_IIS3DWB_CS      GPIO_NUM_7
#define BOARD_GPIO_IIS3DWB_MISO    GPIO_NUM_4
#define BOARD_GPIO_IIS3DWB_MOSI    GPIO_NUM_5
#define BOARD_GPIO_IIS3DWB_INT1    GPIO_NUM_15
#define BOARD_GPIO_IIS3DWB_INT2    GPIO_NUM_38

/* 4G modem */
#define BOARD_GPIO_4G_UART_RX      GPIO_NUM_18
#define BOARD_GPIO_4G_UART_TX      GPIO_NUM_17
#define BOARD_GPIO_4G_PWRKEY       GPIO_NUM_14
#define BOARD_GPIO_4G_STATUS       GPIO_NUM_21
#define BOARD_GPIO_4G_NET_STATUS   GPIO_NUM_47
#define BOARD_GPIO_4G_RESET_N      GPIO_NUM_48

/* 1-Wire temperature sensor */
#define BOARD_GPIO_T1820B_DQ       GPIO_NUM_16

#ifdef __cplusplus
}
#endif

#endif /* BSP_BOARD_H_ */
