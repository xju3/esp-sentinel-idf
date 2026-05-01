#ifndef BSP_POWER_H_
#define BSP_POWER_H_

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bsp_power_init(void);

esp_err_t bsp_power_read_battery_mv(int *battery_mv);
esp_err_t bsp_power_read_supercap_mv(int *supercap_mv);
esp_err_t bsp_power_read_delta_mv(int *delta_mv);

bool bsp_power_cap_is_ready(int battery_mv, int supercap_mv);
esp_err_t bsp_power_charge_cap_until_ready(const char *reason);
esp_err_t bsp_power_prepare_boot_energy(void);
esp_err_t bsp_power_prepare_4g_energy(void);

esp_err_t bsp_power_sensor_enable(void);
esp_err_t bsp_power_sensor_disable(void);
esp_err_t bsp_power_4g_enable(void);
esp_err_t bsp_power_4g_disable(void);
esp_err_t bsp_power_set_status_led(bool on);

#ifdef __cplusplus
}
#endif

#endif /* BSP_POWER_H_ */
