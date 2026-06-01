#pragma once

#include <stdbool.h>

#include "driver/gpio.h"

#define BOARD_CHARGE_PWM_FREQ_HZ           200000
#define BOARD_CHARGE_RAMP_TIME_MS          3000
#define BOARD_CHARGE_RECHECK_INTERVAL_MS   1000
#define BOARD_CHARGE_MAX_HOLD_MS           60000
#define BOARD_CHARGE_MAX_DUTY_PERCENT      50
#define BOARD_CHARGE_SOURCE_MIN_VOLTS      3.0f
#define BOARD_ADC_VOLTAGE_SCALE            1.047f
#define BOARD_MODULE_SUPPLY_MIN_VOLTS      3.3f
#define BOARD_MODULE_SUPPLY_TOLERANCE_VOLTS 0.02f
#define BOARD_WIFI_AP_SSID                 "Sentinel-AP"
#define BOARD_WIFI_AP_PASSWORD             "sentinel123"
#define BOARD_WIFI_AP_CHANNEL              6
#define BOARD_WIFI_AP_MAX_CONN             4
#define BOARD_WIFI_AP_HOLD_MS              60000
#define BOARD_WIFI_STA_SSID                "CU_Up3k"
#define BOARD_WIFI_STA_PASSWORD            "hen6n6c7"
#define BOARD_WIFI_STA_MAX_RETRY           5
#define BOARD_WIFI_STA_TIMEOUT_MS          20000
#define BOARD_MQTT_BROKER_URI              "mqtt://139.9.50.7"
#define BOARD_MQTT_KEEPALIVE_SEC           120
#define BOARD_MQTT_RECONNECT_TIMEOUT_MS    10000
#define BOARD_4G_MQTT_HOST                 "139.9.50.7"
#define BOARD_4G_MQTT_PORT                 1883
#define BOARD_4G_MQTT_CLIENT_ID            "sentinel-pcb-4g"
#define BOARD_4G_KEEP_POWER_ON_AFTER_TEST  1

#define BOARD_GPIO_BATTERY_ADC             GPIO_NUM_2
#define BOARD_GPIO_SUPERCAP_ADC            GPIO_NUM_1
#define BOARD_GPIO_IIS3DWB_MISO            GPIO_NUM_4
#define BOARD_GPIO_IIS3DWB_MOSI            GPIO_NUM_5
#define BOARD_GPIO_IIS3DWB_SCLK            GPIO_NUM_6
#define BOARD_GPIO_IIS3DWB_CS              GPIO_NUM_7
#define BOARD_GPIO_LIS2DH12_INT1           GPIO_NUM_8
#define BOARD_GPIO_LIS2DH12_CS             GPIO_NUM_9
#define BOARD_GPIO_LIS2DH12_MISO           GPIO_NUM_10
#define BOARD_GPIO_LIS2DH12_MOSI           GPIO_NUM_11
#define BOARD_GPIO_LIS2DH12_SCLK           GPIO_NUM_12
#define BOARD_GPIO_LIS2DH12_INT2           GPIO_NUM_3

#define BOARD_GPIO_TEMP_DQ                 GPIO_NUM_16

#define BOARD_GPIO_4G_PWR                  GPIO_NUM_13
#define BOARD_GPIO_4G_PWRKEY               GPIO_NUM_14
#define BOARD_GPIO_4G_UART_TX              GPIO_NUM_17
#define BOARD_GPIO_4G_UART_RX              GPIO_NUM_18
#define BOARD_GPIO_4G_STATUS               GPIO_NUM_21
#define BOARD_GPIO_4G_NET_STATUS           GPIO_NUM_47
#define BOARD_GPIO_4G_RESET_N              GPIO_NUM_48

#define BOARD_GPIO_IIS3DWB_INT1            GPIO_NUM_15
#define BOARD_GPIO_IIS3DWB_INT2            GPIO_NUM_38
#define BOARD_GPIO_IIS3DWB_PWR             BOARD_GPIO_SENSOR_PWR

#define BOARD_GPIO_SENSOR_PWR              GPIO_NUM_42

#define BOARD_GPIO_LED                     GPIO_NUM_40
#define BOARD_GPIO_CHARGE                  GPIO_NUM_41

#define BOARD_LIS2DH12_WHO_AM_I_REG        0x0F
#define BOARD_LIS2DH12_WHO_AM_I_EXPECTED   0x33
#define BOARD_IIS3DWB_WHO_AM_I_REG         0x0F
#define BOARD_IIS3DWB_WHO_AM_I_EXPECTED    0x7B

static inline bool board_gpio_supports_adc(gpio_num_t gpio)
{
    return gpio >= GPIO_NUM_1 && gpio <= GPIO_NUM_20;
}

static inline bool board_gpio_supports_adc1(gpio_num_t gpio)
{
    return gpio >= GPIO_NUM_1 && gpio <= GPIO_NUM_10;
}

static inline bool board_gpio_supports_adc2(gpio_num_t gpio)
{
    return gpio >= GPIO_NUM_11 && gpio <= GPIO_NUM_20;
}
