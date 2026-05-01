#include "power_monitor.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#define POWER_MONITOR_ADC_UNIT ADC_UNIT_1
#define POWER_MONITOR_BATTERY_CHANNEL ADC_CHANNEL_1
#define POWER_MONITOR_SUPERCAP_CHANNEL ADC_CHANNEL_0
#define POWER_MONITOR_ADC_ATTEN ADC_ATTEN_DB_12

typedef struct
{
    adc_channel_t channel;
    adc_cali_handle_t cali_handle;
} power_monitor_channel_ctx_t;

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static bool s_power_monitor_initialized = false;
static power_monitor_channel_ctx_t s_battery_ctx = {
    .channel = POWER_MONITOR_BATTERY_CHANNEL,
    .cali_handle = NULL,
};
static power_monitor_channel_ctx_t s_supercap_ctx = {
    .channel = POWER_MONITOR_SUPERCAP_CHANNEL,
    .cali_handle = NULL,
};

static void power_monitor_cleanup_partial(void)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (s_battery_ctx.cali_handle != NULL)
    {
        adc_cali_delete_scheme_curve_fitting(s_battery_ctx.cali_handle);
        s_battery_ctx.cali_handle = NULL;
    }
    if (s_supercap_ctx.cali_handle != NULL)
    {
        adc_cali_delete_scheme_curve_fitting(s_supercap_ctx.cali_handle);
        s_supercap_ctx.cali_handle = NULL;
    }
#endif

    if (s_adc_handle != NULL)
    {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
    }
}

static float power_monitor_get_ready_threshold_volts(void)
{
    return BOARD_MODULE_SUPPLY_MIN_VOLTS - BOARD_MODULE_SUPPLY_TOLERANCE_VOLTS;
}

static float power_monitor_convert_mv_to_volts(int adc_mv, float divider_scale)
{
    return ((float)adc_mv * divider_scale / 1000.0f) * BOARD_ADC_VOLTAGE_SCALE;
}

static esp_err_t power_monitor_create_cali_handle(adc_channel_t channel,
                                                  adc_cali_handle_t *out_handle)
{
    if (out_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = POWER_MONITOR_ADC_UNIT,
        .chan = channel,
        .atten = POWER_MONITOR_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    return adc_cali_create_scheme_curve_fitting(&cali_cfg, out_handle);
#else
    (void)channel;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static esp_err_t power_monitor_configure_channel(power_monitor_channel_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    adc_oneshot_chan_cfg_t channel_cfg = {
        .atten = POWER_MONITOR_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t err = adc_oneshot_config_channel(s_adc_handle, ctx->channel, &channel_cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    return power_monitor_create_cali_handle(ctx->channel, &ctx->cali_handle);
}

static esp_err_t power_monitor_init(void)
{
    if (s_power_monitor_initialized)
    {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = POWER_MONITOR_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&adc_cfg, &s_adc_handle);
    if (err != ESP_OK)
    {
        return err;
    }

    err = power_monitor_configure_channel(&s_battery_ctx);
    if (err != ESP_OK)
    {
        power_monitor_cleanup_partial();
        return err;
    }

    err = power_monitor_configure_channel(&s_supercap_ctx);
    if (err != ESP_OK)
    {
        power_monitor_cleanup_partial();
        return err;
    }

    s_power_monitor_initialized = true;
    return ESP_OK;
}

static esp_err_t power_monitor_read_mv(const power_monitor_channel_ctx_t *ctx, int *out_mv)
{
    if (ctx == NULL || out_mv == NULL || ctx->cali_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int raw = 0;
    esp_err_t err = adc_oneshot_read(s_adc_handle, ctx->channel, &raw);
    if (err != ESP_OK)
    {
        return err;
    }

    return adc_cali_raw_to_voltage(ctx->cali_handle, raw, out_mv);
}

esp_err_t power_monitor_run(power_monitor_reading_t *reading)
{
    if (reading == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = power_monitor_init();
    if (err != ESP_OK)
    {
        return err;
    }

    int battery_mv = 0;
    int supercap_mv = 0;

    err = power_monitor_read_mv(&s_battery_ctx, &battery_mv);
    if (err != ESP_OK)
    {
        return err;
    }

    err = power_monitor_read_mv(&s_supercap_ctx, &supercap_mv);
    if (err != ESP_OK)
    {
        return err;
    }

    reading->battery_volts = power_monitor_convert_mv_to_volts(
        battery_mv, BOARD_BATTERY_DIVIDER_SCALE);
    reading->supercap_volts = power_monitor_convert_mv_to_volts(
        supercap_mv, BOARD_SUPERCAP_DIVIDER_SCALE);
    reading->ready_threshold_volts = power_monitor_get_ready_threshold_volts();
    reading->battery_ready_for_4g = reading->battery_volts >= reading->ready_threshold_volts;
    reading->supercap_ready_for_4g = reading->supercap_volts >= reading->ready_threshold_volts;
    reading->module_supply_ready =
        reading->battery_ready_for_4g && reading->supercap_ready_for_4g;
    reading->charge_allowed =
        (reading->battery_volts >= BOARD_CHARGE_SOURCE_MIN_VOLTS) &&
        !reading->supercap_ready_for_4g;

    return ESP_OK;
}
