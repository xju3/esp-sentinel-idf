#include "bsp_power.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "bsp_board.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "logger.h"
#include "sdkconfig.h"

#define BSP_POWER_VMON_DIVIDER_NUM      2
#define BSP_POWER_VMON_DIVIDER_DEN      1
#define BSP_POWER_ADC_ATTEN             ADC_ATTEN_DB_12
#define BSP_POWER_ADC_BITWIDTH          ADC_BITWIDTH_DEFAULT
#define BSP_POWER_ADC_SAMPLE_COUNT      8
#define BSP_POWER_CHARGE_MAX_DUTY_PCT   50.0f
#define BSP_POWER_CHARGE_RAMP_STEP_MS   100

#define BSP_POWER_LEDC_MODE             LEDC_LOW_SPEED_MODE
#define BSP_POWER_LEDC_TIMER            LEDC_TIMER_0
#define BSP_POWER_LEDC_CHANNEL          LEDC_CHANNEL_0
#define BSP_POWER_LEDC_DUTY_RES         LEDC_TIMER_13_BIT
#define BSP_POWER_LEDC_DUTY_MAX         ((1U << BSP_POWER_LEDC_DUTY_RES) - 1U)

typedef struct
{
    const char *name;
    int gpio_num;
    adc_unit_t unit_id;
    adc_channel_t channel;
    adc_cali_handle_t cali_handle;
    bool available;
    bool calibrated;
} bsp_power_adc_input_t;

static bool s_initialized = false;
static bool s_voltage_monitor_available = false;
static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
static adc_oneshot_unit_handle_t s_adc2_handle = NULL;
static bsp_power_adc_input_t s_battery_input = {
    .name = "battery",
    .gpio_num = BOARD_GPIO_ADC_BATTERY,
};
static bsp_power_adc_input_t s_supercap_input = {
    .name = "supercap",
    .gpio_num = BOARD_GPIO_ADC_SUPERCAP,
};

static adc_oneshot_unit_handle_t *bsp_power_unit_handle_slot(adc_unit_t unit_id)
{
    if (unit_id == ADC_UNIT_1)
    {
        return &s_adc1_handle;
    }

    if (unit_id == ADC_UNIT_2)
    {
        return &s_adc2_handle;
    }

    return NULL;
}

static bool bsp_power_adc_calibration_init(bsp_power_adc_input_t *input)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t curve_config = {
        .unit_id = input->unit_id,
        .chan = input->channel,
        .atten = BSP_POWER_ADC_ATTEN,
        .bitwidth = BSP_POWER_ADC_BITWIDTH,
    };
    ret = adc_cali_create_scheme_curve_fitting(&curve_config, &handle);
    if (ret == ESP_OK)
    {
        calibrated = true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        adc_cali_line_fitting_config_t line_config = {
            .unit_id = input->unit_id,
            .atten = BSP_POWER_ADC_ATTEN,
            .bitwidth = BSP_POWER_ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&line_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    if (!calibrated)
    {
        if (ret == ESP_ERR_NOT_SUPPORTED)
        {
            LOG_WARNF("ADC calibration unavailable for %s monitor on GPIO%d, using raw fallback.",
                      input->name,
                      input->gpio_num);
        }
        else
        {
            LOG_WARNF("ADC calibration init failed for %s monitor on GPIO%d: %s",
                      input->name,
                      input->gpio_num,
                      esp_err_to_name(ret));
        }
        return false;
    }

    input->cali_handle = handle;
    return true;
}

static esp_err_t bsp_power_adc_input_init(bsp_power_adc_input_t *input)
{
    adc_unit_t unit_id = ADC_UNIT_1;
    adc_channel_t channel = ADC_CHANNEL_0;
    esp_err_t ret = adc_oneshot_io_to_channel(input->gpio_num, &unit_id, &channel);
    if (ret != ESP_OK)
    {
        LOG_WARNF("GPIO%d is not mapped as a native ADC input by ESP-IDF for %s monitoring.",
                  input->gpio_num,
                  input->name);
        input->available = false;
        return ret;
    }

    adc_oneshot_unit_handle_t *unit_handle_slot = bsp_power_unit_handle_slot(unit_id);
    if (unit_handle_slot == NULL)
    {
        input->available = false;
        return ESP_ERR_INVALID_ARG;
    }

    if (*unit_handle_slot == NULL)
    {
        adc_oneshot_unit_init_cfg_t unit_config = {
            .unit_id = unit_id,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ret = adc_oneshot_new_unit(&unit_config, unit_handle_slot);
        if (ret != ESP_OK)
        {
            LOG_ERRORF("Failed to create ADC unit %d for %s monitor: %s",
                       unit_id + 1,
                       input->name,
                       esp_err_to_name(ret));
            input->available = false;
            return ret;
        }
    }

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = BSP_POWER_ADC_ATTEN,
        .bitwidth = BSP_POWER_ADC_BITWIDTH,
    };
    ret = adc_oneshot_config_channel(*unit_handle_slot, channel, &channel_config);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("Failed to configure ADC channel for %s monitor on GPIO%d: %s",
                   input->name,
                   input->gpio_num,
                   esp_err_to_name(ret));
        input->available = false;
        return ret;
    }

    input->unit_id = unit_id;
    input->channel = channel;
    input->available = true;
    input->calibrated = bsp_power_adc_calibration_init(input);

    LOG_INFOF("Power monitor %s mapped to ADC%d channel %d (GPIO%d)%s",
              input->name,
              unit_id + 1,
              channel,
              input->gpio_num,
              input->calibrated ? " with calibration" : " without calibration");

    return ESP_OK;
}

static esp_err_t bsp_power_init_gpio_defaults(void)
{
    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << BOARD_GPIO_SENSOR_EN) |
                        (1ULL << BOARD_GPIO_4G_PWR_EN) |
                        (1ULL << BOARD_GPIO_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&output_config);
    if (ret != ESP_OK)
    {
        return ret;
    }

    gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);
    gpio_set_level(BOARD_GPIO_4G_PWR_EN, 0);
    gpio_set_level(BOARD_GPIO_LED, 0);
    return ESP_OK;
}

static esp_err_t bsp_power_init_charge_pwm(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = BSP_POWER_LEDC_MODE,
        .duty_resolution = BSP_POWER_LEDC_DUTY_RES,
        .timer_num = BSP_POWER_LEDC_TIMER,
        .freq_hz = CONFIG_SENTINEL_CAP_CHARGE_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ledc_channel_config_t channel_config = {
        .gpio_num = BOARD_GPIO_CHARGE_PWM,
        .speed_mode = BSP_POWER_LEDC_MODE,
        .channel = BSP_POWER_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BSP_POWER_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };

    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ledc_update_duty(BSP_POWER_LEDC_MODE, BSP_POWER_LEDC_CHANNEL);
}

static esp_err_t bsp_power_set_charge_duty_pct(float duty_pct)
{
    if (duty_pct < 0.0f)
    {
        duty_pct = 0.0f;
    }

    if (duty_pct > BSP_POWER_CHARGE_MAX_DUTY_PCT)
    {
        duty_pct = BSP_POWER_CHARGE_MAX_DUTY_PCT;
    }

    uint32_t duty = (uint32_t)lroundf((duty_pct / 100.0f) * (float)BSP_POWER_LEDC_DUTY_MAX);
    esp_err_t ret = ledc_set_duty(BSP_POWER_LEDC_MODE, BSP_POWER_LEDC_CHANNEL, duty);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ledc_update_duty(BSP_POWER_LEDC_MODE, BSP_POWER_LEDC_CHANNEL);
}

static esp_err_t bsp_power_read_adc_pin_mv(const bsp_power_adc_input_t *input, int *pin_mv)
{
    if (!input->available)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }

    adc_oneshot_unit_handle_t *unit_handle_slot = bsp_power_unit_handle_slot(input->unit_id);
    if (unit_handle_slot == NULL || *unit_handle_slot == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    int raw_acc = 0;
    for (int i = 0; i < BSP_POWER_ADC_SAMPLE_COUNT; ++i)
    {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(*unit_handle_slot, input->channel, &raw);
        if (ret != ESP_OK)
        {
            return ret;
        }
        raw_acc += raw;
    }

    int raw_avg = raw_acc / BSP_POWER_ADC_SAMPLE_COUNT;
    if (input->calibrated && input->cali_handle != NULL)
    {
        return adc_cali_raw_to_voltage(input->cali_handle, raw_avg, pin_mv);
    }

    *pin_mv = (raw_avg * 3300) / 4095;
    return ESP_OK;
}

static esp_err_t bsp_power_read_input_mv(const bsp_power_adc_input_t *input, int *actual_mv)
{
    int pin_mv = 0;
    esp_err_t ret = bsp_power_read_adc_pin_mv(input, &pin_mv);
    if (ret != ESP_OK)
    {
        return ret;
    }

    *actual_mv = (pin_mv * BSP_POWER_VMON_DIVIDER_NUM) / BSP_POWER_VMON_DIVIDER_DEN;
    return ESP_OK;
}

static esp_err_t bsp_power_prepare_energy_common(const char *reason, bool allow_skip_without_monitor)
{
    if (!s_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_voltage_monitor_available)
    {
        if (allow_skip_without_monitor)
        {
            LOG_WARNF("Skipping %s energy guard: ADC monitor pins are not available in current ESP-IDF target mapping.",
                      reason);
            return ESP_OK;
        }
        return ESP_ERR_NOT_SUPPORTED;
    }

    int battery_mv = 0;
    int supercap_mv = 0;
    esp_err_t ret = bsp_power_read_battery_mv(&battery_mv);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = bsp_power_read_supercap_mv(&supercap_mv);
    if (ret != ESP_OK)
    {
        return ret;
    }

    LOG_INFOF("%s energy check: battery=%dmV supercap=%dmV delta=%dmV",
              reason,
              battery_mv,
              supercap_mv,
              battery_mv - supercap_mv);

    if (bsp_power_cap_is_ready(battery_mv, supercap_mv))
    {
        LOG_INFOF("%s energy already ready.", reason);
        return ESP_OK;
    }

    return bsp_power_charge_cap_until_ready(reason);
}

esp_err_t bsp_power_init(void)
{
    if (s_initialized)
    {
        return ESP_OK;
    }

    esp_err_t ret = bsp_power_init_gpio_defaults();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = bsp_power_init_charge_pwm();
    if (ret != ESP_OK)
    {
        return ret;
    }

    (void)bsp_power_adc_input_init(&s_battery_input);
    (void)bsp_power_adc_input_init(&s_supercap_input);

    s_voltage_monitor_available = s_battery_input.available && s_supercap_input.available;
    if (!s_voltage_monitor_available)
    {
        LOG_WARN("Voltage monitor inputs are not fully available; energy guard will log and skip until pin mapping is validated.");
    }

    s_initialized = true;
    LOG_INFO("Board power manager initialized.");
    return ESP_OK;
}

esp_err_t bsp_power_read_battery_mv(int *battery_mv)
{
    if (battery_mv == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return bsp_power_read_input_mv(&s_battery_input, battery_mv);
}

esp_err_t bsp_power_read_supercap_mv(int *supercap_mv)
{
    if (supercap_mv == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return bsp_power_read_input_mv(&s_supercap_input, supercap_mv);
}

esp_err_t bsp_power_read_delta_mv(int *delta_mv)
{
    if (delta_mv == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int battery_mv = 0;
    int supercap_mv = 0;
    esp_err_t ret = bsp_power_read_battery_mv(&battery_mv);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = bsp_power_read_supercap_mv(&supercap_mv);
    if (ret != ESP_OK)
    {
        return ret;
    }

    *delta_mv = battery_mv - supercap_mv;
    return ESP_OK;
}

bool bsp_power_cap_is_ready(int battery_mv, int supercap_mv)
{
    return (battery_mv - supercap_mv) <= CONFIG_SENTINEL_CAP_READY_DELTA_MV;
}

esp_err_t bsp_power_charge_cap_until_ready(const char *reason)
{
    if (!s_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_voltage_monitor_available)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }

    const char *charge_reason = (reason != NULL) ? reason : "power";
    const int64_t start_tick = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(CONFIG_SENTINEL_CAP_CHARGE_TIMEOUT_SEC * 1000U);
    const int ramp_steps = (CONFIG_SENTINEL_CAP_CHARGE_RAMP_MS + BSP_POWER_CHARGE_RAMP_STEP_MS - 1) / BSP_POWER_CHARGE_RAMP_STEP_MS;

    LOG_INFOF("Start super-cap charging for %s.", charge_reason);

    for (int step = 1; step <= ramp_steps; ++step)
    {
        float duty_pct = (BSP_POWER_CHARGE_MAX_DUTY_PCT * (float)step) / (float)ramp_steps;
        esp_err_t ret = bsp_power_set_charge_duty_pct(duty_pct);
        if (ret != ESP_OK)
        {
            (void)bsp_power_set_charge_duty_pct(0.0f);
            return ret;
        }

        vTaskDelay(pdMS_TO_TICKS(BSP_POWER_CHARGE_RAMP_STEP_MS));
        if ((xTaskGetTickCount() - start_tick) >= timeout_ticks)
        {
            (void)bsp_power_set_charge_duty_pct(0.0f);
            return ESP_ERR_TIMEOUT;
        }
    }

    esp_err_t ret = bsp_power_set_charge_duty_pct(BSP_POWER_CHARGE_MAX_DUTY_PCT);
    if (ret != ESP_OK)
    {
        (void)bsp_power_set_charge_duty_pct(0.0f);
        return ret;
    }

    if (CONFIG_SENTINEL_CAP_CHARGE_HOLD_MS > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_SENTINEL_CAP_CHARGE_HOLD_MS));
    }

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks)
    {
        int battery_mv = 0;
        int supercap_mv = 0;
        ret = bsp_power_read_battery_mv(&battery_mv);
        if (ret != ESP_OK)
        {
            break;
        }

        ret = bsp_power_read_supercap_mv(&supercap_mv);
        if (ret != ESP_OK)
        {
            break;
        }

        int delta_mv = battery_mv - supercap_mv;
        LOG_INFOF("Charging %s: battery=%dmV supercap=%dmV delta=%dmV",
                  charge_reason,
                  battery_mv,
                  supercap_mv,
                  delta_mv);

        if (bsp_power_cap_is_ready(battery_mv, supercap_mv))
        {
            LOG_INFOF("Super-cap ready for %s.", charge_reason);
            (void)bsp_power_set_charge_duty_pct(0.0f);
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(CONFIG_SENTINEL_CAP_CHARGE_POLL_MS));
    }

    (void)bsp_power_set_charge_duty_pct(0.0f);
    return (ret == ESP_OK) ? ESP_ERR_TIMEOUT : ret;
}

esp_err_t bsp_power_prepare_boot_energy(void)
{
    return bsp_power_prepare_energy_common("boot", true);
}

esp_err_t bsp_power_prepare_4g_energy(void)
{
    esp_err_t ret = bsp_power_prepare_energy_common("4G", false);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = bsp_power_4g_enable();
    if (ret != ESP_OK)
    {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(CONFIG_SENTINEL_4G_POWER_SETTLE_MS));
    return ESP_OK;
}

esp_err_t bsp_power_sensor_enable(void)
{
    gpio_set_level(BOARD_GPIO_SENSOR_EN, 0);
    return ESP_OK;
}

esp_err_t bsp_power_sensor_disable(void)
{
    gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);
    return ESP_OK;
}

esp_err_t bsp_power_4g_enable(void)
{
    gpio_set_level(BOARD_GPIO_4G_PWR_EN, 1);
    return ESP_OK;
}

esp_err_t bsp_power_4g_disable(void)
{
    gpio_set_level(BOARD_GPIO_4G_PWR_EN, 0);
    return ESP_OK;
}

esp_err_t bsp_power_set_status_led(bool on)
{
    gpio_set_level(BOARD_GPIO_LED, on ? 1 : 0);
    return ESP_OK;
}
