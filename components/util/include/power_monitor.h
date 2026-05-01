#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_BATTERY_ADC_GPIO GPIO_NUM_2
#define BOARD_SUPERCAP_ADC_GPIO GPIO_NUM_1
#define BOARD_CHARGE_SOURCE_MIN_VOLTS 3.0f
#define BOARD_MODULE_SUPPLY_MIN_VOLTS 3.3f
#define BOARD_MODULE_SUPPLY_TOLERANCE_VOLTS 0.02f
#define BOARD_ADC_VOLTAGE_SCALE 1.047f
/* Board analog front-end is currently treated as a 1:1 divider on both rails. */
#define BOARD_BATTERY_DIVIDER_SCALE 2.0f
#define BOARD_SUPERCAP_DIVIDER_SCALE 2.0f

typedef struct
{
    float battery_volts;
    float supercap_volts;
    float ready_threshold_volts;
    bool charge_allowed;
    bool battery_ready_for_4g;
    bool supercap_ready_for_4g;
    bool module_supply_ready;
} power_monitor_reading_t;

esp_err_t power_monitor_run(power_monitor_reading_t *reading);

#ifdef __cplusplus
}
#endif

#endif // POWER_MONITOR_H
