#include "comm_diag.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_config.h"

#define MODEM_UART_PORT                  UART_NUM_1
#define MODEM_UART_BAUD_RATE             115200
#define MODEM_UART_RX_BUF_SIZE           2048
#define MODEM_UART_TX_BUF_SIZE           1024
#define MODEM_RESP_BUF_SIZE              1024

#define MODEM_POWER_ENABLE_LEVEL         1
#define MODEM_POWER_DISABLE_LEVEL        0

#define MODEM_POWER_SETTLE_MS            100
#define MODEM_AUTO_BOOT_DETECT_MS        2500
#define MODEM_PULSE_PWRKEY_MS            600
#define MODEM_BOOT_TIMEOUT_MS            15000
#define MODEM_STATUS_LOW_WINDOW_OBSERVE_MS 3000
#define MODEM_STATUS_LOW_SYNC_TIMEOUT_MS   600
#define MODEM_STATUS_POLL_MS               20
#define MODEM_REG_TIMEOUT_MS             60000
#define MODEM_ATTACH_TIMEOUT_MS          30000
#define MODEM_PDP_TIMEOUT_MS             30000
#define MODEM_SHUTDOWN_TIMEOUT_MS        65000
#define MODEM_MQTT_OPEN_TIMEOUT_MS       30000
#define MODEM_MQTT_CONNECT_TIMEOUT_MS    30000
#define MODEM_MQTT_HOLD_MS               20000
#define MODEM_REG_POLL_MS                200

#define MODEM_SYNC_AT_CMD                "AT"
#define MODEM_VDD_EXT_AT_CMD             "AT+QGPIOV=0"
#define STRINGIFY_VALUE(x)               #x
#define STRINGIFY(x)                     STRINGIFY_VALUE(x)

static const char *TAG = "comm_diag";

typedef struct {
    bool uart_ready;
} modem_ctx_t;

static bool response_has_token(const char *response, const char *token)
{
    return response != NULL && token != NULL && strstr(response, token) != NULL;
}

static int64_t deadline_after_ms(uint32_t timeout_ms)
{
    return esp_timer_get_time() + ((int64_t)timeout_ms * 1000LL);
}

static esp_err_t modem_uart_init(modem_ctx_t *ctx)
{
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (ctx->uart_ready) {
        return ESP_OK;
    }

    const uart_config_t config = {
        .baud_rate = MODEM_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(MODEM_UART_PORT,
                                        MODEM_UART_RX_BUF_SIZE,
                                        MODEM_UART_TX_BUF_SIZE,
                                        0,
                                        NULL,
                                        0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = uart_param_config(MODEM_UART_PORT, &config);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_set_pin(MODEM_UART_PORT,
                       BOARD_GPIO_4G_UART_TX,
                       BOARD_GPIO_4G_UART_RX,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_flush_input(MODEM_UART_PORT);
    if (err != ESP_OK) {
        return err;
    }

    ctx->uart_ready = true;
    return ESP_OK;
}

static void modem_uart_deinit(modem_ctx_t *ctx)
{
    if (ctx != NULL && ctx->uart_ready) {
        (void)uart_driver_delete(MODEM_UART_PORT);
        ctx->uart_ready = false;
    }
}

static esp_err_t modem_gpio_init(void)
{
    const gpio_config_t power_cfg = {
        .pin_bit_mask = 1ULL << BOARD_GPIO_4G_PWR,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&power_cfg);
    if (err != ESP_OK) {
        return err;
    }

    const gpio_config_t pwrkey_cfg = {
        .pin_bit_mask = (1ULL << BOARD_GPIO_4G_PWRKEY),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&pwrkey_cfg);
    if (err != ESP_OK) {
        return err;
    }

    const gpio_config_t input_cfg = {
        .pin_bit_mask = (1ULL << BOARD_GPIO_4G_STATUS) | (1ULL << BOARD_GPIO_4G_NET_STATUS),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&input_cfg);
    if (err != ESP_OK) {
        return err;
    }

    (void)gpio_set_level(BOARD_GPIO_4G_PWRKEY, 1);
    return ESP_OK;
}

static esp_err_t modem_power_enable(void)
{
    esp_err_t err = gpio_set_level(BOARD_GPIO_4G_PWR, MODEM_POWER_ENABLE_LEVEL);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(MODEM_POWER_SETTLE_MS));
    }
    return err;
}

static esp_err_t modem_power_disable(void)
{
    return gpio_set_level(BOARD_GPIO_4G_PWR, MODEM_POWER_DISABLE_LEVEL);
}

static int modem_status_level(void)
{
    return gpio_get_level(BOARD_GPIO_4G_STATUS);
}

static bool modem_status_is_on(void)
{
    return modem_status_level() == 1;
}

static int modem_net_status_level(void)
{
    return gpio_get_level(BOARD_GPIO_4G_NET_STATUS);
}

static bool modem_net_is_connected(void)
{
    return modem_net_status_level() == 0;
}

static void comm_diag_log_gpio_snapshot(const char *context)
{
    return;

    ESP_LOGI(TAG,
             "gpio_snapshot=%s 4g_pwr[gpio=%d]=%d 4g_pwrkey[gpio=%d]=%d "
             "4g_status[gpio=%d]=%d 4g_net_status[gpio=%d]=%d 4g_uart_tx[gpio=%d]=%d 4g_uart_rx[gpio=%d]=%d",
             context != NULL ? context : "unknown",
             BOARD_GPIO_4G_PWR,
             gpio_get_level(BOARD_GPIO_4G_PWR),
             BOARD_GPIO_4G_PWRKEY,
             gpio_get_level(BOARD_GPIO_4G_PWRKEY),
             BOARD_GPIO_4G_STATUS,
             modem_status_level(),
             BOARD_GPIO_4G_NET_STATUS,
             modem_net_status_level(),
             BOARD_GPIO_4G_UART_TX,
             gpio_get_level(BOARD_GPIO_4G_UART_TX),
             BOARD_GPIO_4G_UART_RX,
             gpio_get_level(BOARD_GPIO_4G_UART_RX));
}

static esp_err_t modem_release_low_active_line(gpio_num_t pin)
{
    return gpio_set_level(pin, 1);
}

static esp_err_t modem_pulse_low_active_line(gpio_num_t pin, uint32_t pulse_ms)
{
    esp_err_t err = gpio_set_level(pin, 0);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(pulse_ms));
    return gpio_set_level(pin, 1);
}

static esp_err_t modem_wait_for_status_level(int expected_level, uint32_t timeout_ms)
{
    int64_t deadline = deadline_after_ms(timeout_ms);
    while (esp_timer_get_time() < deadline) {
        if (modem_status_level() == expected_level) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t modem_read_response(char *response, size_t response_size, uint32_t timeout_ms)
{
    if (response == NULL || response_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t used = 0;
    response[0] = '\0';

    int64_t deadline = deadline_after_ms(timeout_ms);
    while (esp_timer_get_time() < deadline) {
        uint8_t rx_buf[128];
        int read_len = uart_read_bytes(MODEM_UART_PORT,
                                       rx_buf,
                                       sizeof(rx_buf),
                                       pdMS_TO_TICKS(100));
        if (read_len > 0) {
            size_t copy_len = (size_t)read_len;
            if (used + copy_len >= response_size) {
                copy_len = response_size - used - 1;
            }
            memcpy(response + used, rx_buf, copy_len);
            used += copy_len;
            response[used] = '\0';

            if (response_has_token(response, "\r\nOK\r\n") ||
                response_has_token(response, "\r\nERROR\r\n") ||
                response_has_token(response, "+CME ERROR:") ||
                response_has_token(response, "POWERED DOWN")) {
                return ESP_OK;
            }
        }
    }

    return used > 0 ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t modem_read_until_pattern(char *response,
                                          size_t response_size,
                                          const char *pattern,
                                          uint32_t timeout_ms)
{
    if (response == NULL || response_size == 0 || pattern == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t used = 0;
    response[0] = '\0';

    int64_t deadline = deadline_after_ms(timeout_ms);
    while (esp_timer_get_time() < deadline) {
        uint8_t rx_buf[128];
        int read_len = uart_read_bytes(MODEM_UART_PORT,
                                       rx_buf,
                                       sizeof(rx_buf),
                                       pdMS_TO_TICKS(100));
        if (read_len <= 0) {
            continue;
        }

        size_t copy_len = (size_t)read_len;
        if (used + copy_len >= response_size) {
            copy_len = response_size - used - 1;
        }
        memcpy(response + used, rx_buf, copy_len);
        used += copy_len;
        response[used] = '\0';

        if (strstr(response, pattern) != NULL) {
            return ESP_OK;
        }
    }

    return used > 0 ? ESP_ERR_NOT_FOUND : ESP_ERR_TIMEOUT;
}

#if defined(AT) && AT == 1
static void format_visible_bytes(const char *input, char *output, size_t output_size)
{
    if (output == NULL || output_size == 0) {
        return;
    }

    size_t out = 0;
    output[0] = '\0';
    if (input == NULL) {
        return;
    }

    for (size_t i = 0; input[i] != '\0' && out + 1 < output_size; ++i) {
        unsigned char ch = (unsigned char)input[i];
        const char *escaped = NULL;
        char hex[5] = { 0 };

        switch (ch) {
        case '\r':
            escaped = "\\r";
            break;
        case '\n':
            escaped = "\\n";
            break;
        case '\t':
            escaped = "\\t";
            break;
        default:
            if (!isprint(ch)) {
                snprintf(hex, sizeof(hex), "\\x%02X", ch);
                escaped = hex;
            }
            break;
        }

        if (escaped != NULL) {
            size_t escaped_len = strlen(escaped);
            if (out + escaped_len >= output_size) {
                break;
            }
            memcpy(output + out, escaped, escaped_len);
            out += escaped_len;
        } else {
            output[out++] = (char)ch;
        }
    }
    output[out] = '\0';
}
#endif

static esp_err_t modem_send_command(const char *cmd,
                                    char *response,
                                    size_t response_size,
                                    uint32_t timeout_ms)
{
    if (cmd == NULL || response == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

#if defined(AT) && AT == 1
    char tx_visible[160];
    char tx_frame[128];
    int tx_frame_len = snprintf(tx_frame, sizeof(tx_frame), "%s\r\n", cmd);
    if (tx_frame_len < 0) {
        return ESP_FAIL;
    }
    format_visible_bytes(tx_frame, tx_visible, sizeof(tx_visible));
    ESP_LOGI(TAG, ">>> UART TX AT frame=\"%s\" cmd=\"%s\" len=%d", tx_visible, cmd, tx_frame_len);
#endif

    (void)uart_flush_input(MODEM_UART_PORT);
    int written = uart_write_bytes(MODEM_UART_PORT, cmd, (size_t)strlen(cmd));
    if (written < 0) {
        return ESP_FAIL;
    }
    written = uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);
    if (written < 0) {
        return ESP_FAIL;
    }
    (void)uart_wait_tx_done(MODEM_UART_PORT, pdMS_TO_TICKS(1000));

    esp_err_t err = modem_read_response(response, response_size, timeout_ms);
#if defined(AT) && AT == 1
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "<<< AT RX:\n%s", response);
    } else {
        ESP_LOGW(TAG, "<<< AT RX: (timeout/error=%d)", err);
    }
#endif
    return err;
}

static bool modem_response_is_ok(const char *response)
{
    return response_has_token(response, "\r\nOK\r\n") || response_has_token(response, "\nOK\r\n");
}

static bool modem_response_is_registered(const char *response)
{
    return response_has_token(response, "+CEREG: 1") ||
           response_has_token(response, "+CEREG: 5") ||
           response_has_token(response, "+CEREG: 0,1") ||
           response_has_token(response, "+CEREG: 0,5");
}

static bool modem_response_has_ip(const char *response)
{
    return response != NULL &&
           strstr(response, "+CGPADDR:") != NULL &&
           strstr(response, "0.0.0.0") == NULL &&
           strstr(response, "\"\"") == NULL;
}

static int modem_parse_cereg_stat(const char *response)
{
    int n = -1;
    int stat = -1;
    const char *line = response != NULL ? strstr(response, "+CEREG:") : NULL;
    if (line == NULL) {
        return -1;
    }

    if (sscanf(line, "+CEREG: %d,%d", &n, &stat) == 2) {
        return stat;
    }
    if (sscanf(line, "+CEREG: %d", &stat) == 1) {
        return stat;
    }
    return -1;
}

static int modem_parse_qmtopen_result(const char *response)
{
    int connect_id = -1;
    int result = -1;
    const char *line = response != NULL ? strstr(response, "+QMTOPEN:") : NULL;
    if (line != NULL && sscanf(line, "+QMTOPEN: %d,%d", &connect_id, &result) == 2) {
        return result;
    }
    return -1;
}

static int modem_parse_qmtconn_retcode(const char *response)
{
    int connect_id = -1;
    int result = -1;
    int retcode = -1;
    const char *line = response != NULL ? strstr(response, "+QMTCONN:") : NULL;
    if (line != NULL && sscanf(line, "+QMTCONN: %d,%d,%d", &connect_id, &result, &retcode) == 3) {
        return retcode;
    }
    return -1;
}

static void modem_copy_cgpaddr_ip(const char *response, char *ip_addr, size_t ip_addr_size)
{
    if (response == NULL || ip_addr == NULL || ip_addr_size == 0) {
        return;
    }

    const char *line = strstr(response, "+CGPADDR:");
    if (line == NULL) {
        return;
    }

    const char *comma = strchr(line, ',');
    if (comma == NULL) {
        return;
    }

    const char *start = comma + 1;
    while (*start == ' ' || *start == '"') {
        ++start;
    }

    size_t len = 0;
    while (start[len] != '\0' &&
           start[len] != '"' &&
           start[len] != '\r' &&
           start[len] != '\n' &&
           start[len] != ',') {
        ++len;
    }

    if (len == 0 || len >= ip_addr_size) {
        return;
    }
    memcpy(ip_addr, start, len);
    ip_addr[len] = '\0';
}

static bool modem_attached(char *response, size_t response_size)
{
    if (modem_send_command("AT+CGATT?", response, response_size, 1000) != ESP_OK) {
        return false;
    }
    return response_has_token(response, "+CGATT: 1");
}

static bool modem_pdp_active(char *response, size_t response_size)
{
    if (modem_send_command("AT+CGACT?", response, response_size, 1000) != ESP_OK) {
        return false;
    }
    return response_has_token(response, "+CGACT: 1,1");
}

static esp_err_t modem_sync(void)
{
    char response[MODEM_RESP_BUF_SIZE];
    int64_t deadline = deadline_after_ms(MODEM_BOOT_TIMEOUT_MS);
    while (esp_timer_get_time() < deadline) {
        esp_err_t err = modem_send_command(MODEM_SYNC_AT_CMD, response, sizeof(response), 200);
        if (err == ESP_OK && modem_response_is_ok(response)) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_ERR_TIMEOUT;
}



static esp_err_t modem_shutdown_gracefully(bool at_ready)
{
    if (at_ready) {
        (void)uart_flush_input(MODEM_UART_PORT);
        static const char shutdown_cmd[] = "AT+QPOWD=1\r\n";
        int written = uart_write_bytes(MODEM_UART_PORT, shutdown_cmd, sizeof(shutdown_cmd) - 1);
        if (written >= 0) {
            (void)uart_wait_tx_done(MODEM_UART_PORT, pdMS_TO_TICKS(1000));
            // ESP_LOGI(TAG, "Sent AT+QPOWD=1, waiting for STATUS pin to go low...");
            if (modem_wait_for_status_level(0, MODEM_SHUTDOWN_TIMEOUT_MS) == ESP_OK) {
                ESP_LOGI(TAG, "Module gracefully powered down after holding MQTT connection for 20 seconds.");
                ESP_LOGI(TAG, "=======================================");
                return ESP_OK;
            } else {
                ESP_LOGW(TAG, "Graceful shutdown timeout via STATUS pin.");
                ESP_LOGI(TAG, "=======================================");
            }
        }
    }

    if (modem_status_is_on()) {
        ESP_LOGW(TAG, "Forcing shutdown via PWRKEY...");
        (void)modem_pulse_low_active_line(BOARD_GPIO_4G_PWRKEY, 700);
        (void)modem_wait_for_status_level(0, 5000);
    }

    return ESP_OK;
}

static esp_err_t modem_mqtt_open(char *response, size_t response_size)
{
    esp_err_t err = modem_send_command("AT+QMTOPEN=0,\"" BOARD_4G_MQTT_HOST "\"," STRINGIFY(BOARD_4G_MQTT_PORT),
                                       response,
                                       response_size,
                                       10000);
    if (err != ESP_OK || !modem_response_is_ok(response)) {
        return err != ESP_OK ? err : ESP_FAIL;
    }
    if (response_has_token(response, "+QMTOPEN:")) {
        return ESP_OK;
    }
    return modem_read_until_pattern(response, response_size, "+QMTOPEN:", MODEM_MQTT_OPEN_TIMEOUT_MS);
}

static esp_err_t modem_mqtt_connect(char *response, size_t response_size)
{
    esp_err_t err = modem_send_command("AT+QMTCONN=0,\"" BOARD_4G_MQTT_CLIENT_ID "\"",
                                       response,
                                       response_size,
                                       10000);
    if (err != ESP_OK || !modem_response_is_ok(response)) {
        return err != ESP_OK ? err : ESP_FAIL;
    }
    if (response_has_token(response, "+QMTCONN:")) {
        return ESP_OK;
    }
    return modem_read_until_pattern(response, response_size, "+QMTCONN:", MODEM_MQTT_CONNECT_TIMEOUT_MS);
}

static esp_err_t modem_mqtt_disconnect(char *response, size_t response_size)
{
    return modem_send_command("AT+QMTDISC=0", response, response_size, 5000);
}

static esp_err_t modem_mqtt_close(char *response, size_t response_size)
{
    return modem_send_command("AT+QMTCLOSE=0", response, response_size, 5000);
}

const char *comm_diag_code_to_str(comm_diag_code_t code)
{
    switch (code) {
    case COMM_DIAG_OK:
        return "ok";
    case COMM_DIAG_SKIPPED:
        return "skipped";
    case COMM_DIAG_POWER_ON_TIMEOUT:
        return "power_on_timeout";
    case COMM_DIAG_AT_NO_RESPONSE:
        return "at_no_response";
    case COMM_DIAG_SIM_NOT_READY:
        return "sim_not_ready";
    case COMM_DIAG_NOT_REGISTERED:
        return "not_registered";
    case COMM_DIAG_ATTACH_FAILED:
        return "attach_failed";
    case COMM_DIAG_PDP_FAILED:
        return "pdp_failed";
    case COMM_DIAG_NO_IP:
        return "no_ip";
    case COMM_DIAG_MQTT_OPEN_FAILED:
        return "mqtt_open_failed";
    case COMM_DIAG_MQTT_CONNECT_FAILED:
        return "mqtt_connect_failed";
    case COMM_DIAG_IO_ERROR:
        return "io_error";
    default:
        return "unknown";
    }
}

esp_err_t comm_diag_force_safe_off(bool *status_high)
{
    esp_err_t err = modem_gpio_init();
    if (err != ESP_OK) {
        return err;
    }

    err = modem_release_low_active_line(BOARD_GPIO_4G_PWRKEY);
    if (err != ESP_OK) {
        return err;
    }

    err = modem_power_disable();
    if (err != ESP_OK) {
        return err;
    }

    if (status_high != NULL) {
        *status_high = modem_status_level() != 0;
    }

    return ESP_OK;
}


void comm_diag_run(comm_diag_result_t *result)
{
    if (result == NULL) {
        return;
    }

    memset(result, 0, sizeof(*result));
    result->code = COMM_DIAG_IO_ERROR;

    modem_ctx_t ctx = { 0 };
    bool at_ready = false;
    bool report_logged = false;

    int64_t total_start_us = esp_timer_get_time();
    int64_t stage_start_us;

    // ---------------------------------------------------------
    // Stage 1: Power On
    // ---------------------------------------------------------
    stage_start_us = esp_timer_get_time();
    esp_err_t err = modem_gpio_init();
    if (err != ESP_OK) goto cleanup;
    comm_diag_log_gpio_snapshot("after_gpio_init");

    err = modem_uart_init(&ctx);
    if (err != ESP_OK) goto cleanup;

    (void)modem_release_low_active_line(BOARD_GPIO_4G_PWRKEY);
    (void)gpio_set_level(BOARD_GPIO_4G_PWRKEY, 1);
    (void)modem_power_disable();
    vTaskDelay(pdMS_TO_TICKS(100));

    err = modem_power_enable();
    if (err != ESP_OK) goto cleanup;

    result->timing.power_on_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);

    // ---------------------------------------------------------
    // Stage 2: Boot Handshake
    // ---------------------------------------------------------
    stage_start_us = esp_timer_get_time();
    err = modem_pulse_low_active_line(BOARD_GPIO_4G_PWRKEY, MODEM_PULSE_PWRKEY_MS);
    if (err != ESP_OK) goto cleanup;

    err = modem_sync();
    result->timing.boot_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    if (err != ESP_OK) {
        result->code = COMM_DIAG_POWER_ON_TIMEOUT;
        goto cleanup;
    }
    at_ready = true;

    // ---------------------------------------------------------
    // Stage 3: SIM Ready
    // ---------------------------------------------------------
    stage_start_us = esp_timer_get_time();
    char response[MODEM_RESP_BUF_SIZE];
    
    // Just check CPIN until READY
    int64_t cpin_deadline = deadline_after_ms(10000);
    bool sim_ready = false;
    while (esp_timer_get_time() < cpin_deadline) {
        if (modem_send_command("AT+CPIN?", response, sizeof(response), 1000) == ESP_OK) {
            if (response_has_token(response, "+CPIN: READY")) {
                sim_ready = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    result->timing.sim_ready_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    if (!sim_ready) {
        result->code = COMM_DIAG_SIM_NOT_READY;
        goto cleanup;
    }
    result->sim_ready = true;

    // ---------------------------------------------------------
    // Stage 4: Network Registration & Attach
    // ---------------------------------------------------------
    stage_start_us = esp_timer_get_time();
    
    if (modem_send_command("AT+CFUN?", response, sizeof(response), 1000) != ESP_OK ||
        !response_has_token(response, "+CFUN: 1")) {
        (void)modem_send_command("AT+CFUN=1", response, sizeof(response), 2000);
    }

    int64_t reg_deadline = deadline_after_ms(MODEM_REG_TIMEOUT_MS);
    bool registered = false;
    while (esp_timer_get_time() < reg_deadline) {
        if (modem_send_command("AT+CEREG?", response, sizeof(response), 1000) == ESP_OK) {
            result->cereg_stat = modem_parse_cereg_stat(response);
            if (modem_response_is_registered(response)) {
                registered = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(MODEM_REG_POLL_MS));
    }

    result->timing.network_attach_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    if (!registered) {
        result->code = COMM_DIAG_NOT_REGISTERED;
        goto cleanup;
    }
    result->registered = true;

    if (!modem_attached(response, sizeof(response))) {
        err = modem_send_command("AT+CGATT=1", response, sizeof(response), MODEM_ATTACH_TIMEOUT_MS);
        if (err != ESP_OK || !modem_response_is_ok(response)) {
            result->timing.network_attach_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
            result->code = COMM_DIAG_ATTACH_FAILED;
            goto cleanup;
        }
    }
    result->attached = true;
    result->timing.network_attach_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);

    // ---------------------------------------------------------
    // Stage 5: PDP/IP Ready
    // ---------------------------------------------------------
    stage_start_us = esp_timer_get_time();

    if (!modem_pdp_active(response, sizeof(response))) {
        err = modem_send_command("AT+CGACT=1,1", response, sizeof(response), MODEM_PDP_TIMEOUT_MS);
        if (err != ESP_OK || !modem_response_is_ok(response)) {
            result->timing.pdp_active_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
            result->code = COMM_DIAG_PDP_FAILED;
            goto cleanup;
        }
    }

    err = modem_send_command("AT+CGPADDR=1", response, sizeof(response), 5000);
    result->timing.pdp_active_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    if (err != ESP_OK || !modem_response_has_ip(response)) {
        result->code = COMM_DIAG_NO_IP;
        goto cleanup;
    }
    result->pdp_active = true;
    modem_copy_cgpaddr_ip(response, result->ip_addr, sizeof(result->ip_addr));

    // ---------------------------------------------------------
    // Stage 6: MQTT
    // ---------------------------------------------------------
    stage_start_us = esp_timer_get_time();

    err = modem_mqtt_open(response, sizeof(response));
    result->timing.mqtt_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    result->mqtt_open_result = modem_parse_qmtopen_result(response);
    if (err != ESP_OK || result->mqtt_open_result != 0) {
        result->code = COMM_DIAG_MQTT_OPEN_FAILED;
        goto cleanup;
    }
    result->mqtt_opened = true;

    err = modem_mqtt_connect(response, sizeof(response));
    result->timing.mqtt_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    result->mqtt_conn_retcode = modem_parse_qmtconn_retcode(response);
    if (err != ESP_OK || result->mqtt_conn_retcode != 0) {
        result->code = COMM_DIAG_MQTT_CONNECT_FAILED;
        goto cleanup;
    }
    result->mqtt_connected = true;
    result->code = COMM_DIAG_OK;
    result->timing.total_ms = (uint32_t)((esp_timer_get_time() - total_start_us) / 1000LL);
    comm_diag_log_result(result);
    report_logged = true;

    vTaskDelay(pdMS_TO_TICKS(MODEM_MQTT_HOLD_MS));

    (void)modem_mqtt_disconnect(response, sizeof(response));
    (void)modem_mqtt_close(response, sizeof(response));

cleanup:
    if (!report_logged) {
        result->timing.total_ms = (uint32_t)((esp_timer_get_time() - total_start_us) / 1000LL);
        comm_diag_log_result(result);
    }

    // ESP_LOGW(TAG, "starting to shutdown 4G module gracefully.");
    comm_diag_log_gpio_snapshot("before_shutdown");
    (void)modem_shutdown_gracefully(at_ready);
    (void)modem_power_disable();
    modem_uart_deinit(&ctx);
}

void comm_diag_log_result(const comm_diag_result_t *result)
{
    if (result == NULL) {
        return;
    }

    ESP_LOGI(TAG, "4G Module Startup Timing Report");
    ESP_LOGI(TAG, "Result: %s", comm_diag_code_to_str(result->code));
    ESP_LOGI(TAG, "---------------------------------------");
    ESP_LOGI(TAG, "1. Power Enable : %lu ms", (unsigned long)result->timing.power_on_ms);
    ESP_LOGI(TAG, "2. AT Handshake : %lu ms", (unsigned long)result->timing.boot_ms);
    ESP_LOGI(TAG, "3. SIM Ready    : %lu ms", (unsigned long)result->timing.sim_ready_ms);
    ESP_LOGI(TAG, "4. Network Reg  : %lu ms", (unsigned long)result->timing.network_attach_ms);
    ESP_LOGI(TAG, "5. PDP/IP       : %lu ms  (active=%s ip=%s)",
             (unsigned long)result->timing.pdp_active_ms,
             result->pdp_active ? "true" : "false",
             result->ip_addr[0] != '\0' ? result->ip_addr : "none");
    ESP_LOGI(TAG, "6. MQTT         : %lu ms  (opened=%s connected=%s open_result=%d retcode=%d)",
             (unsigned long)result->timing.mqtt_ms,
             result->mqtt_opened ? "true" : "false",
             result->mqtt_connected ? "true" : "false",
             result->mqtt_open_result,
             result->mqtt_conn_retcode);
    ESP_LOGI(TAG, "---------------------------------------");
    ESP_LOGI(TAG, "Total Time      : %lu ms", (unsigned long)result->timing.total_ms);
    ESP_LOGI(TAG, "---------------------------------------");

}
