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
#define BOARD_4G_KEEP_POWER_ON_AFTER_TEST  1


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
