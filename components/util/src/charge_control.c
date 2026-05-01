#include "charge_control.h"

#include <stdint.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CHARGE_LEDC_MODE LEDC_LOW_SPEED_MODE
#define CHARGE_LEDC_TIMER LEDC_TIMER_0
#define CHARGE_LEDC_CHANNEL LEDC_CHANNEL_0
#define CHARGE_LEDC_DUTY_RESOLUTION LEDC_TIMER_8_BIT
#define CHARGE_RAMP_STEPS 20U

static bool s_charge_control_initialized = false;

static uint32_t charge_control_get_max_duty(void)
{
    return (1U << CHARGE_LEDC_DUTY_RESOLUTION) - 1U;
}

static uint32_t charge_control_percent_to_duty(uint32_t duty_percent)
{
    if (duty_percent > BOARD_CHARGE_MAX_DUTY_PERCENT)
    {
        duty_percent = BOARD_CHARGE_MAX_DUTY_PERCENT;
    }

    return (charge_control_get_max_duty() * duty_percent) / 100U;
}

esp_err_t charge_control_init(void)
{
    if (s_charge_control_initialized)
    {
        return ESP_OK;
    }

    ledc_timer_config_t timer_cfg = {
        .speed_mode = CHARGE_LEDC_MODE,
        .duty_resolution = CHARGE_LEDC_DUTY_RESOLUTION,
        .timer_num = CHARGE_LEDC_TIMER,
        .freq_hz = BOARD_CHARGE_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    ledc_channel_config_t channel_cfg = {
        .gpio_num = BOARD_GPIO_CHARGE,
        .speed_mode = CHARGE_LEDC_MODE,
        .channel = CHARGE_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = CHARGE_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
    err = ledc_channel_config(&channel_cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    s_charge_control_initialized = true;
    return charge_control_stop();
}

esp_err_t charge_control_set_duty_percent(uint32_t duty_percent)
{
    esp_err_t err = charge_control_init();
    if (err != ESP_OK)
    {
        return err;
    }

    uint32_t duty = charge_control_percent_to_duty(duty_percent);
    err = ledc_set_duty(CHARGE_LEDC_MODE, CHARGE_LEDC_CHANNEL, duty);
    if (err != ESP_OK)
    {
        return err;
    }

    return ledc_update_duty(CHARGE_LEDC_MODE, CHARGE_LEDC_CHANNEL);
}

esp_err_t charge_control_stop(void)
{
    if (!s_charge_control_initialized)
    {
        return ESP_OK;
    }

    esp_err_t err = ledc_set_duty(CHARGE_LEDC_MODE, CHARGE_LEDC_CHANNEL, 0);
    if (err != ESP_OK)
    {
        return err;
    }

    err = ledc_update_duty(CHARGE_LEDC_MODE, CHARGE_LEDC_CHANNEL);
    if (err != ESP_OK)
    {
        return err;
    }

    ledc_stop(CHARGE_LEDC_MODE, CHARGE_LEDC_CHANNEL, 0);
    return ESP_OK;
}

esp_err_t charge_control_ramp_to_half_scale(void)
{
    esp_err_t err = charge_control_set_duty_percent(0);
    if (err != ESP_OK)
    {
        return err;
    }

    const TickType_t step_delay_ticks =
        pdMS_TO_TICKS(BOARD_CHARGE_RAMP_TIME_MS / CHARGE_RAMP_STEPS);

    for (uint32_t step = 1; step <= CHARGE_RAMP_STEPS; ++step)
    {
        uint32_t duty_percent = (BOARD_CHARGE_MAX_DUTY_PERCENT * step) / CHARGE_RAMP_STEPS;
        err = charge_control_set_duty_percent(duty_percent);
        if (err != ESP_OK)
        {
            charge_control_stop();
            return err;
        }
        vTaskDelay(step_delay_ticks);
    }

    return charge_control_set_duty_percent(BOARD_CHARGE_MAX_DUTY_PERCENT);
}
