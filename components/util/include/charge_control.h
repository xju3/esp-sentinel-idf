#ifndef CHARGE_CONTROL_H
#define CHARGE_CONTROL_H

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_GPIO_CHARGE GPIO_NUM_41
#define BOARD_CHARGE_PWM_FREQ_HZ 200000U
#define BOARD_CHARGE_MAX_DUTY_PERCENT 50U
#define BOARD_CHARGE_RAMP_TIME_MS 3000U

esp_err_t charge_control_init(void);
esp_err_t charge_control_set_duty_percent(uint32_t duty_percent);
esp_err_t charge_control_stop(void);
esp_err_t charge_control_ramp_to_half_scale(void);

#ifdef __cplusplus
}
#endif

#endif // CHARGE_CONTROL_H
