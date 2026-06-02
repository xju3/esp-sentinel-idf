#include "bsp_4g.h"
#include "bsp_board.h"
#include "board_config.h"
#include "config_manager.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_defaults.h"
#include "esp_netif_ppp.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <sys/param.h>
#include <stdbool.h>
#include "freertos/event_groups.h"

#define PPP_VERBOSE 0 // 设为 1 可开启调试日志

#ifndef SN
#define SN "0"
#endif

// ============== Board pin aliases ==============
#define MODEM_UART_RX_PIN      BOARD_GPIO_4G_UART_RX
#define MODEM_UART_TX_PIN      BOARD_GPIO_4G_UART_TX
#define MODEM_PWR_EN_PIN       BOARD_GPIO_4G_PWR
#define MODEM_PWRKEY_PIN       BOARD_GPIO_4G_PWRKEY
#define MODEM_STATUS_PIN       BOARD_GPIO_4G_STATUS
#define MODEM_NET_STATUS_PIN   BOARD_GPIO_4G_NET_STATUS
#define UART_PORT_NUM UART_NUM_1
#define MODEM_UART_BAUD_RATE 115200
#define MODEM_UART_RX_BUF_SIZE 2048
#define MODEM_UART_TX_BUF_SIZE 1024
#define BUF_SIZE (1024)
#define MODEM_RESP_BUF_SIZE 1024
#define MODEM_POWER_ENABLE_LEVEL 1
#define MODEM_POWER_DISABLE_LEVEL 0
#define MODEM_POWER_SETTLE_MS 100
#define MODEM_PULSE_PWRKEY_MS 600
#define MODEM_BOOT_TIMEOUT_MS 15000
#define MODEM_SIM_TIMEOUT_MS 10000
#define MODEM_REG_TIMEOUT_MS 60000
#define MODEM_ATTACH_TIMEOUT_MS 30000
#define MODEM_PDP_TIMEOUT_MS 30000
#define MODEM_SHUTDOWN_TIMEOUT_MS 65000
#define MODEM_MQTT_OPEN_TIMEOUT_MS 30000
#define MODEM_MQTT_CONNECT_TIMEOUT_MS 30000
#define MODEM_MQTT_PUBLISH_TIMEOUT_MS 30000
#define MODEM_REG_POLL_MS 200
#define MODEM_SYNC_AT_CMD "AT"
#define PPP_IP_INFO_TIMEOUT_MS 3000
#define PPP_IP_INFO_POLL_MS 100
#define PPP_POST_CONNECT_STABILIZE_MS 500

static const char *TAG = "ppp_4g";

// PPP/LwIP state
static bool s_event_loop_initialized = false;
static bool s_ppp_handlers_registered = false;
static esp_netif_t *s_ppp_netif = NULL;
static EventGroupHandle_t s_ppp_event_group = NULL;
static TaskHandle_t s_ppp_rx_task = NULL;
static volatile bool s_ppp_rx_task_running = false;
static bool s_ppp_session_started = false;
static bool s_uart_driver_installed = false;
static bool s_at_ready = false;
static bool s_module_mqtt_connected = false;
static char s_modem_response[MODEM_RESP_BUF_SIZE];

#define PPP_GOT_IP_BIT BIT0
#define PPP_FAILED_BIT BIT1

typedef enum {
    PPP_4G_DIAG_OK = 0,
    PPP_4G_DIAG_POWER_ON_FAILED,
    PPP_4G_DIAG_AT_NO_RESPONSE,
    PPP_4G_DIAG_SIM_NOT_READY,
    PPP_4G_DIAG_NOT_REGISTERED,
    PPP_4G_DIAG_ATTACH_FAILED,
    PPP_4G_DIAG_PDP_FAILED,
    PPP_4G_DIAG_NO_IP,
    PPP_4G_DIAG_MQTT_OPEN_FAILED,
    PPP_4G_DIAG_MQTT_CONNECT_FAILED,
    PPP_4G_DIAG_MQTT_PUBLISH_FAILED,
    PPP_4G_DIAG_PPP_DIAL_FAILED,
    PPP_4G_DIAG_PPP_CONNECT_FAILED,
    PPP_4G_DIAG_IO_ERROR,
} ppp_4g_diag_code_t;

typedef struct {
    uint32_t power_on_ms;
    uint32_t boot_ms;
    uint32_t sim_ready_ms;
    uint32_t network_attach_ms;
    uint32_t pdp_active_ms;
    uint32_t mqtt_ms;
    uint32_t ppp_ms;
    uint32_t total_ms;
} ppp_4g_diag_timing_t;

typedef struct {
    ppp_4g_diag_code_t code;
    ppp_4g_diag_timing_t timing;
    bool sim_ready;
    bool registered;
    bool attached;
    bool pdp_active;
    bool mqtt_opened;
    bool mqtt_connected;
    bool ppp_attempted;
    bool ppp_connected;
    int mqtt_open_result;
    int mqtt_conn_retcode;
    char ip_addr[48];
} ppp_4g_diag_result_t;

static int64_t deadline_after_ms(uint32_t timeout_ms)
{
    return esp_timer_get_time() + ((int64_t)timeout_ms * 1000LL);
}

static bool response_has_token(const char *response, const char *token)
{
    return response != NULL && token != NULL && strstr(response, token) != NULL;
}

static bool response_find_pattern_offset(const char *response,
                                         size_t response_len,
                                         const char *pattern,
                                         size_t *offset)
{
    if (response == NULL || pattern == NULL) {
        return false;
    }

    size_t pattern_len = strlen(pattern);
    if (pattern_len == 0 || response_len < pattern_len) {
        return false;
    }

    for (size_t i = 0; i <= response_len - pattern_len; ++i) {
        if (memcmp(response + i, pattern, pattern_len) == 0) {
            if (offset != NULL) {
                *offset = i;
            }
            return true;
        }
    }
    return false;
}

static bool modem_response_is_ok(const char *response)
{
    return response_has_token(response, "\r\nOK\r\n") ||
           response_has_token(response, "\nOK\r\n");
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

static const char *ppp_4g_diag_code_to_str(ppp_4g_diag_code_t code)
{
    switch (code) {
    case PPP_4G_DIAG_OK:
        return "ok";
    case PPP_4G_DIAG_POWER_ON_FAILED:
        return "power_on_failed";
    case PPP_4G_DIAG_AT_NO_RESPONSE:
        return "at_no_response";
    case PPP_4G_DIAG_SIM_NOT_READY:
        return "sim_not_ready";
    case PPP_4G_DIAG_NOT_REGISTERED:
        return "not_registered";
    case PPP_4G_DIAG_ATTACH_FAILED:
        return "attach_failed";
    case PPP_4G_DIAG_PDP_FAILED:
        return "pdp_failed";
    case PPP_4G_DIAG_NO_IP:
        return "no_ip";
    case PPP_4G_DIAG_MQTT_OPEN_FAILED:
        return "mqtt_open_failed";
    case PPP_4G_DIAG_MQTT_CONNECT_FAILED:
        return "mqtt_connect_failed";
    case PPP_4G_DIAG_MQTT_PUBLISH_FAILED:
        return "mqtt_publish_failed";
    case PPP_4G_DIAG_PPP_DIAL_FAILED:
        return "ppp_dial_failed";
    case PPP_4G_DIAG_PPP_CONNECT_FAILED:
        return "ppp_connect_failed";
    case PPP_4G_DIAG_IO_ERROR:
        return "io_error";
    default:
        return "unknown";
    }
}

static void ppp_4g_log_result(const ppp_4g_diag_result_t *result)
{
    if (result == NULL) {
        return;
    }

    ESP_LOGI(TAG, "4G Module Startup Timing Report");
    ESP_LOGI(TAG, "Result: %s", ppp_4g_diag_code_to_str(result->code));
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
    if (result->ppp_attempted) {
        ESP_LOGI(TAG, "7. PPP          : %lu ms  (connected=%s)",
                 (unsigned long)result->timing.ppp_ms,
                 result->ppp_connected ? "true" : "false");
    }
    ESP_LOGI(TAG, "---------------------------------------");
    ESP_LOGI(TAG, "Total Time      : %lu ms", (unsigned long)result->timing.total_ms);
    ESP_LOGI(TAG, "---------------------------------------");
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

static const char *modem_mqtt_host(void)
{
    const char *host = g_user_config.host;
    if (host == NULL || host[0] == '\0') {
        return BOARD_4G_MQTT_HOST;
    }

    const char *scheme = strstr(host, "://");
    return scheme != NULL ? scheme + 3 : host;
}

static void modem_copy_mqtt_host(char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) {
        return;
    }

    const char *host = modem_mqtt_host();
    size_t len = 0;
    while (host[len] != '\0' && host[len] != ':' && host[len] != '/' && len + 1 < out_size) {
        out[len] = host[len];
        len++;
    }
    out[len] = '\0';
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

static int modem_parse_qmtpub_result(const char *response)
{
    int connect_id = -1;
    int msg_id = -1;
    int result = -1;
    const char *line = response != NULL ? strstr(response, "+QMTPUBEX:") : NULL;
    if (line != NULL && sscanf(line, "+QMTPUBEX: %d,%d,%d", &connect_id, &msg_id, &result) == 3) {
        return result;
    }
    line = response != NULL ? strstr(response, "+QMTPUB:") : NULL;
    if (line != NULL && sscanf(line, "+QMTPUB: %d,%d,%d", &connect_id, &msg_id, &result) == 3) {
        return result;
    }
    return -1;
}

static esp_err_t modem_gpio_init(void)
{
    const gpio_config_t power_cfg = {
        .pin_bit_mask = 1ULL << MODEM_PWR_EN_PIN,
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
        .pin_bit_mask = (1ULL << MODEM_PWRKEY_PIN),
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
        .pin_bit_mask = (1ULL << MODEM_STATUS_PIN) | (1ULL << MODEM_NET_STATUS_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&input_cfg);
    if (err != ESP_OK) {
        return err;
    }

    (void)gpio_set_level(MODEM_PWRKEY_PIN, 1);
    return ESP_OK;
}

static esp_err_t modem_uart_init(void)
{
    if (s_uart_driver_installed) {
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

    esp_err_t err = uart_driver_install(UART_PORT_NUM,
                                        MODEM_UART_RX_BUF_SIZE,
                                        MODEM_UART_TX_BUF_SIZE,
                                        0,
                                        NULL,
                                        0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = uart_param_config(UART_PORT_NUM, &config);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_set_pin(UART_PORT_NUM,
                       MODEM_UART_TX_PIN,
                       MODEM_UART_RX_PIN,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_flush_input(UART_PORT_NUM);
    if (err != ESP_OK) {
        return err;
    }

    s_uart_driver_installed = true;
    return ESP_OK;
}

static void modem_uart_deinit(void)
{
    if (s_uart_driver_installed) {
        (void)uart_driver_delete(UART_PORT_NUM);
        s_uart_driver_installed = false;
    }
}

static esp_err_t modem_power_enable(void)
{
    esp_err_t err = gpio_set_level(MODEM_PWR_EN_PIN, MODEM_POWER_ENABLE_LEVEL);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(MODEM_POWER_SETTLE_MS));
    }
    return err;
}

static esp_err_t modem_power_disable(void)
{
    return gpio_set_level(MODEM_PWR_EN_PIN, MODEM_POWER_DISABLE_LEVEL);
}

static int modem_status_level(void)
{
    return gpio_get_level(MODEM_STATUS_PIN);
}

static bool modem_status_is_on(void)
{
    return modem_status_level() == 1;
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
        int read_len = uart_read_bytes(UART_PORT_NUM,
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
        int read_len = uart_read_bytes(UART_PORT_NUM,
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

        size_t pattern_offset = 0;
        if (response_find_pattern_offset(response, used, pattern, &pattern_offset)) {
            if (pattern_offset > 0) {
                size_t remaining = used - pattern_offset;
                memmove(response, response + pattern_offset, remaining);
                used = remaining;
                response[used] = '\0';
            }
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

    (void)uart_flush_input(UART_PORT_NUM);
    int written = uart_write_bytes(UART_PORT_NUM, cmd, (size_t)strlen(cmd));
    if (written < 0) {
        return ESP_FAIL;
    }
    written = uart_write_bytes(UART_PORT_NUM, "\r\n", 2);
    if (written < 0) {
        return ESP_FAIL;
    }
    (void)uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(1000));

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
    char *response = s_modem_response;
    int64_t deadline = deadline_after_ms(MODEM_BOOT_TIMEOUT_MS);
    while (esp_timer_get_time() < deadline) {
        esp_err_t err = modem_send_command(MODEM_SYNC_AT_CMD, response, MODEM_RESP_BUF_SIZE, 200);
        if (err == ESP_OK && modem_response_is_ok(response)) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_ERR_TIMEOUT;
}

static void modem_disable_echo(void)
{
    char *response = s_modem_response;
    (void)modem_send_command("ATE0", response, MODEM_RESP_BUF_SIZE, 1000);
}

static esp_err_t modem_shutdown_gracefully(bool at_ready)
{
    if (at_ready) {
        (void)uart_flush_input(UART_PORT_NUM);
        static const char shutdown_cmd[] = "AT+QPOWD=1\r\n";
        int written = uart_write_bytes(UART_PORT_NUM, shutdown_cmd, sizeof(shutdown_cmd) - 1);
        if (written >= 0) {
            (void)uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(1000));
            if (modem_wait_for_status_level(0, MODEM_SHUTDOWN_TIMEOUT_MS) == ESP_OK) {
                ESP_LOGI(TAG, "Module gracefully powered down.");
                return ESP_OK;
            }
            ESP_LOGW(TAG, "Graceful shutdown timeout via STATUS pin.");
        }
    }

    if (modem_status_is_on()) {
        ESP_LOGW(TAG, "Forcing shutdown via PWRKEY...");
        (void)modem_pulse_low_active_line(MODEM_PWRKEY_PIN, 700);
        (void)modem_wait_for_status_level(0, 5000);
    }

    return ESP_OK;
}

static esp_err_t modem_prepare_packet_service(ppp_4g_diag_result_t *result)
{
    char *response = s_modem_response;
    int64_t stage_start_us = esp_timer_get_time();

    esp_err_t err = modem_sync();
    if (result != NULL) {
        result->timing.boot_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    }
    if (err != ESP_OK) {
        if (result != NULL) {
            result->code = PPP_4G_DIAG_AT_NO_RESPONSE;
        }
        return err;
    }
    s_at_ready = true;
    modem_disable_echo();

    stage_start_us = esp_timer_get_time();
    bool sim_ready = false;
    int64_t cpin_deadline = deadline_after_ms(MODEM_SIM_TIMEOUT_MS);
    while (esp_timer_get_time() < cpin_deadline) {
        if (modem_send_command("AT+CPIN?", response, MODEM_RESP_BUF_SIZE, 1000) == ESP_OK) {
            if (response_has_token(response, "+CPIN: READY")) {
                sim_ready = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (result != NULL) {
        result->timing.sim_ready_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    }
    if (!sim_ready) {
        if (result != NULL) {
            result->code = PPP_4G_DIAG_SIM_NOT_READY;
        }
        return ESP_ERR_TIMEOUT;
    }
    if (result != NULL) {
        result->sim_ready = true;
    }

    stage_start_us = esp_timer_get_time();
    if (modem_send_command("AT+CFUN?", response, MODEM_RESP_BUF_SIZE, 1000) != ESP_OK ||
        !response_has_token(response, "+CFUN: 1")) {
        (void)modem_send_command("AT+CFUN=1", response, MODEM_RESP_BUF_SIZE, 2000);
    }

    bool registered = false;
    int64_t reg_deadline = deadline_after_ms(MODEM_REG_TIMEOUT_MS);
    while (esp_timer_get_time() < reg_deadline) {
        if (modem_send_command("AT+CEREG?", response, MODEM_RESP_BUF_SIZE, 1000) == ESP_OK) {
            if (modem_response_is_registered(response)) {
                registered = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(MODEM_REG_POLL_MS));
    }
    if (result != NULL) {
        result->timing.network_attach_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    }
    if (!registered) {
        if (result != NULL) {
            result->code = PPP_4G_DIAG_NOT_REGISTERED;
        }
        return ESP_ERR_TIMEOUT;
    }
    if (result != NULL) {
        result->registered = true;
    }

    if (!modem_attached(response, MODEM_RESP_BUF_SIZE)) {
        err = modem_send_command("AT+CGATT=1", response, MODEM_RESP_BUF_SIZE, MODEM_ATTACH_TIMEOUT_MS);
        if (err != ESP_OK || !modem_response_is_ok(response)) {
            if (result != NULL) {
                result->timing.network_attach_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
                result->code = PPP_4G_DIAG_ATTACH_FAILED;
            }
            return err != ESP_OK ? err : ESP_FAIL;
        }
    }
    if (result != NULL) {
        result->attached = true;
        result->timing.network_attach_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    }

    stage_start_us = esp_timer_get_time();
    if (!modem_pdp_active(response, MODEM_RESP_BUF_SIZE)) {
        err = modem_send_command("AT+CGACT=1,1", response, MODEM_RESP_BUF_SIZE, MODEM_PDP_TIMEOUT_MS);
        if (err != ESP_OK || !modem_response_is_ok(response)) {
            if (result != NULL) {
                result->timing.pdp_active_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
                result->code = PPP_4G_DIAG_PDP_FAILED;
            }
            return err != ESP_OK ? err : ESP_FAIL;
        }
    }

    err = modem_send_command("AT+CGPADDR=1", response, MODEM_RESP_BUF_SIZE, 5000);
    if (result != NULL) {
        result->timing.pdp_active_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    }
    if (err != ESP_OK || !modem_response_has_ip(response)) {
        if (result != NULL) {
            result->code = PPP_4G_DIAG_NO_IP;
        }
        return err != ESP_OK ? err : ESP_FAIL;
    }
    if (result != NULL) {
        result->pdp_active = true;
        modem_copy_cgpaddr_ip(response, result->ip_addr, sizeof(result->ip_addr));
    }

    ESP_LOGI(TAG, "4G packet service is ready; switching to PPP dial.");
    return ESP_OK;
}

static esp_err_t modem_mqtt_open(char *response, size_t response_size)
{
    char host[96];
    char cmd[160];
    modem_copy_mqtt_host(host, sizeof(host));

    int len = snprintf(cmd,
                       sizeof(cmd),
                       "AT+QMTOPEN=0,\"%s\",%d",
                       host,
                       BOARD_4G_MQTT_PORT);
    if (len < 0 || (size_t)len >= sizeof(cmd)) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t err = modem_send_command(cmd, response, response_size, 10000);
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
    char cmd[160];
    int len = snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\"", SN);
    if (len < 0 || (size_t)len >= sizeof(cmd)) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t err = modem_send_command(cmd, response, response_size, 10000);
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

static esp_err_t modem_mqtt_connect_session(ppp_4g_diag_result_t *result)
{
    char *response = s_modem_response;
    int64_t stage_start_us = esp_timer_get_time();

    esp_err_t err = modem_mqtt_open(response, MODEM_RESP_BUF_SIZE);
    if (result != NULL) {
        result->timing.mqtt_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
        result->mqtt_open_result = modem_parse_qmtopen_result(response);
    }
    if (err != ESP_OK || modem_parse_qmtopen_result(response) != 0) {
        if (result != NULL) {
            result->code = PPP_4G_DIAG_MQTT_OPEN_FAILED;
        }
        return err != ESP_OK ? err : ESP_FAIL;
    }
    if (result != NULL) {
        result->mqtt_opened = true;
    }

    err = modem_mqtt_connect(response, MODEM_RESP_BUF_SIZE);
    if (result != NULL) {
        result->timing.mqtt_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
        result->mqtt_conn_retcode = modem_parse_qmtconn_retcode(response);
    }
    if (err != ESP_OK || modem_parse_qmtconn_retcode(response) != 0) {
        if (result != NULL) {
            result->code = PPP_4G_DIAG_MQTT_CONNECT_FAILED;
        }
        return err != ESP_OK ? err : ESP_FAIL;
    }

    s_module_mqtt_connected = true;
    if (result != NULL) {
        result->mqtt_connected = true;
        result->code = PPP_4G_DIAG_OK;
    }
    return ESP_OK;
}

static esp_err_t modem_mqtt_publish_binary(const char *topic, const uint8_t *data, size_t len)
{
    if (topic == NULL || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_module_mqtt_connected) {
        return ESP_ERR_INVALID_STATE;
    }

    char *response = s_modem_response;
    char cmd[192];
    int cmd_len = snprintf(cmd,
                           sizeof(cmd),
                           "AT+QMTPUBEX=0,1,1,0,\"%s\",%u",
                           topic,
                           (unsigned)len);
    if (cmd_len < 0 || (size_t)cmd_len >= sizeof(cmd)) {
        return ESP_ERR_INVALID_SIZE;
    }

    (void)uart_flush_input(UART_PORT_NUM);
    int written = uart_write_bytes(UART_PORT_NUM, cmd, (size_t)cmd_len);
    if (written < 0) {
        return ESP_FAIL;
    }
    written = uart_write_bytes(UART_PORT_NUM, "\r\n", 2);
    if (written < 0) {
        return ESP_FAIL;
    }
    (void)uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(1000));

    esp_err_t err = modem_read_until_pattern(response, MODEM_RESP_BUF_SIZE, ">", 5000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "QMTPUBEX prompt failed: %s", response[0] != '\0' ? response : "(none)");
        return err;
    }

    written = uart_write_bytes(UART_PORT_NUM, data, len);
    if (written < 0 || (size_t)written != len) {
        return ESP_FAIL;
    }
    static const uint8_t end_marker = 0x1A;
    written = uart_write_bytes(UART_PORT_NUM, &end_marker, sizeof(end_marker));
    if (written < 0) {
        return ESP_FAIL;
    }
    (void)uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(5000));

    err = modem_read_until_pattern(response, MODEM_RESP_BUF_SIZE, "+QMTPUB", MODEM_MQTT_PUBLISH_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "QMTPUBEX result wait failed: %s", esp_err_to_name(err));
        return err;
    }

    int pub_result = modem_parse_qmtpub_result(response);
    if (pub_result != 0) {
        ESP_LOGE(TAG, "QMTPUBEX failed result=%d", pub_result);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void modem_exit_data_mode(void)
{
    if (!s_uart_driver_installed) {
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    (void)uart_write_bytes(UART_PORT_NUM, "+++", 3);
    (void)uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
    (void)uart_flush_input(UART_PORT_NUM);
}

static esp_err_t ppp_uart_transmit(void *h, void *buffer, size_t len)
{
    (void)h;
    if (len == 0 || buffer == NULL)
    {
        return ESP_OK;
    }
    uart_write_bytes(UART_PORT_NUM, buffer, len);
    return ESP_OK;
}

static esp_netif_driver_ifconfig_t s_ppp_driver_cfg = {
    .handle = (void *)1, // non-NULL handle
    .transmit = ppp_uart_transmit,
};

static void ppp_status_event_handler(void *arg, esp_event_base_t event_base,
                                     int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;
    if (event_base != NETIF_PPP_STATUS)
    {
        return;
    }
    if (event_id >= NETIF_PPP_INTERNAL_ERR_OFFSET)
    {
        ESP_LOGW(TAG, "PPP status error: %ld", event_id - NETIF_PPP_INTERNAL_ERR_OFFSET);
    }
}

static void ppp_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    if (event_id == IP_EVENT_PPP_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        if (event->esp_netif != s_ppp_netif)
        {
            return;
        }
        xEventGroupSetBits(s_ppp_event_group, PPP_GOT_IP_BIT);
    }
    else if (event_id == IP_EVENT_PPP_LOST_IP)
    {
        (void)event_data;
        ESP_LOGW(TAG, "PPP Lost IP");
        xEventGroupSetBits(s_ppp_event_group, PPP_FAILED_BIT);
    }
}

static void ppp_uart_rx_task(void *args)
{
    (void)args;
    uint8_t *buffer = (uint8_t *)malloc(BUF_SIZE);
    if (!buffer)
    {
        ESP_LOGE(TAG, "PPP RX buffer alloc failed");
        s_ppp_rx_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    while (s_ppp_rx_task_running)
    {
        int len = uart_read_bytes(UART_PORT_NUM, buffer, BUF_SIZE, pdMS_TO_TICKS(1000));
        if (len > 0 && s_ppp_netif)
        {
            esp_netif_receive(s_ppp_netif, buffer, len, NULL);
        }
    }

    free(buffer);
    s_ppp_rx_task = NULL;
    vTaskDelete(NULL);
}

static void ppp_reset_event_bits(void)
{
    if (s_ppp_event_group != NULL)
    {
        xEventGroupClearBits(s_ppp_event_group, PPP_GOT_IP_BIT | PPP_FAILED_BIT);
    }
}

static void ppp_stop_session(void)
{
    if (s_ppp_netif == NULL || !s_ppp_session_started)
    {
        ppp_reset_event_bits();
        return;
    }

    esp_netif_action_disconnected(s_ppp_netif, 0, 0, 0);
    esp_netif_action_stop(s_ppp_netif, 0, 0, 0);
    s_ppp_session_started = false;
    ppp_reset_event_bits();
}

static void ppp_cleanup_failed_init(bool power_enabled)
{
    ppp_stop_session();

    if (s_ppp_rx_task_running)
    {
        s_ppp_rx_task_running = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (power_enabled)
    {
        (void)modem_shutdown_gracefully(s_at_ready);
        (void)modem_power_disable();
    }

    modem_uart_deinit();
    s_at_ready = false;
}

static void ppp_destroy_netif(void)
{
    if (s_ppp_netif != NULL)
    {
        esp_netif_destroy(s_ppp_netif);
        s_ppp_netif = NULL;
    }
    s_ppp_session_started = false;
    ppp_reset_event_bits();
}

static esp_err_t ppp_wait_for_ip_ready(void)
{
    esp_netif_ip_info_t ip_info = {0};
    const TickType_t step_ticks = pdMS_TO_TICKS(PPP_IP_INFO_POLL_MS);
    const TickType_t timeout_ticks = pdMS_TO_TICKS(PPP_IP_INFO_TIMEOUT_MS);
    const TickType_t start_ticks = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_ticks) < timeout_ticks)
    {
        if (s_ppp_netif != NULL &&
            esp_netif_get_ip_info(s_ppp_netif, &ip_info) == ESP_OK &&
            ip_info.ip.addr != 0U)
        {
            return ESP_OK;
        }
        vTaskDelay(step_ticks);
    }

    ESP_LOGW(TAG, "PPP reported connected but IP info did not become ready in time");
    return ESP_ERR_TIMEOUT;
}

static esp_err_t ppp_stack_init(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        return err;
    }

    if (!s_event_loop_initialized)
    {
        err = esp_event_loop_create_default();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
            return err;
        }
        s_event_loop_initialized = true;
    }

    if (!s_ppp_event_group)
    {
        s_ppp_event_group = xEventGroupCreate();
        if (!s_ppp_event_group)
        {
            ESP_LOGE(TAG, "PPP event group create failed");
            return ESP_FAIL;
        }
    }

    if (!s_ppp_handlers_registered)
    {
        err = esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, ppp_ip_event_handler, NULL);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "IP event handler register failed: %s", esp_err_to_name(err));
            return err;
        }
        err = esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, ppp_status_event_handler, NULL);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "PPP event handler register failed: %s", esp_err_to_name(err));
            return err;
        }
        s_ppp_handlers_registered = true;
    }

    if (!s_ppp_netif)
    {
        esp_netif_inherent_config_t base_cfg = ESP_NETIF_INHERENT_DEFAULT_PPP();
        base_cfg.if_desc = "pppos_client";
        esp_netif_config_t cfg = {
            .base = &base_cfg,
            .driver = &s_ppp_driver_cfg,
            .stack = ESP_NETIF_NETSTACK_DEFAULT_PPP,
        };
        s_ppp_netif = esp_netif_new(&cfg);
        if (!s_ppp_netif)
        {
            ESP_LOGE(TAG, "Failed to create PPP netif");
            return ESP_FAIL;
        }
        esp_netif_set_default_netif(s_ppp_netif);
    }

    return ESP_OK;
}

static esp_err_t ppp_start_and_wait_ip(ppp_4g_diag_result_t *result)
{
    char response_buffer[256];
    int64_t stage_start_us = esp_timer_get_time();
    if (result != NULL) {
        result->ppp_attempted = true;
    }

    (void)uart_flush_input(UART_PORT_NUM);
    static const char dial_cmd[] = "ATD*99***1#\r\n";
    int written = uart_write_bytes(UART_PORT_NUM, dial_cmd, sizeof(dial_cmd) - 1);
    esp_err_t err = ESP_OK;
    if (written < 0) {
        err = ESP_FAIL;
    } else {
        err = uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(1000));
    }
    if (err == ESP_OK) {
        err = modem_read_until_pattern(response_buffer, sizeof(response_buffer), "CONNECT", 30000);
    }
    if (err != ESP_OK)
    {
        if (result != NULL) {
            result->timing.ppp_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
            result->code = PPP_4G_DIAG_PPP_DIAL_FAILED;
        }
        ESP_LOGE(TAG, "Failed to enter PPP data mode");
        ESP_LOGE(TAG, "PPP dial response: %s", response_buffer[0] != '\0' ? response_buffer : "(none)");
        return ESP_FAIL;
    }

    if (!s_ppp_rx_task_running)
    {
        s_ppp_rx_task_running = true;
        if (xTaskCreate(ppp_uart_rx_task, "ppp_rx", 4096, NULL, 5, &s_ppp_rx_task) != pdTRUE)
        {
            s_ppp_rx_task_running = false;
            if (result != NULL) {
                result->timing.ppp_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
                result->code = PPP_4G_DIAG_PPP_CONNECT_FAILED;
            }
            ESP_LOGE(TAG, "Failed to create PPP RX task");
            return ESP_FAIL;
        }
    }

    if (s_ppp_session_started)
    {
        ESP_LOGW(TAG, "PPP session still active, forcing stop before restart");
        ppp_stop_session();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ppp_reset_event_bits();
    esp_netif_action_start(s_ppp_netif, 0, 0, 0);
    s_ppp_session_started = true;
    esp_netif_action_connected(s_ppp_netif, 0, 0, 0);

    EventBits_t bits = xEventGroupWaitBits(s_ppp_event_group,
                                           PPP_GOT_IP_BIT | PPP_FAILED_BIT,
                                           pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(60000));
    if (bits & PPP_GOT_IP_BIT)
    {
        esp_err_t ip_ready_err = ppp_wait_for_ip_ready();
        if (ip_ready_err != ESP_OK)
        {
            ppp_stop_session();
            if (result != NULL) {
                result->timing.ppp_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
                result->code = PPP_4G_DIAG_PPP_CONNECT_FAILED;
            }
            return ip_ready_err;
        }

        vTaskDelay(pdMS_TO_TICKS(PPP_POST_CONNECT_STABILIZE_MS));
        if (result != NULL) {
            result->timing.ppp_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
            result->ppp_connected = true;
            result->code = PPP_4G_DIAG_OK;
        }
        return ESP_OK;
    }

    ESP_LOGE(TAG, "PPP connect timeout/failed");
    ppp_stop_session();
    if (result != NULL) {
        result->timing.ppp_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
        result->code = PPP_4G_DIAG_PPP_CONNECT_FAILED;
    }
    return ESP_FAIL;
}

esp_err_t init_ppp_4g(cb_communication_channel_established cb)
{
    esp_err_t err = ESP_OK;
    bool power_enabled = false;
    bool report_logged = false;
    ppp_4g_diag_result_t result = {
        .code = PPP_4G_DIAG_IO_ERROR,
    };
    int64_t total_start_us = esp_timer_get_time();
    int64_t stage_start_us = total_start_us;

    err = modem_gpio_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "4G GPIO init failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    err = modem_uart_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "4G UART init failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    (void)modem_release_low_active_line(MODEM_PWRKEY_PIN);
    (void)gpio_set_level(MODEM_PWRKEY_PIN, 1);
    (void)modem_power_disable();
    vTaskDelay(pdMS_TO_TICKS(100));

    err = modem_power_enable();
    result.timing.power_on_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    if (err != ESP_OK)
    {
        result.code = PPP_4G_DIAG_POWER_ON_FAILED;
        ESP_LOGE(TAG, "4G power on failed: %s", esp_err_to_name(err));
        goto cleanup;
    }
    power_enabled = true;

    err = modem_pulse_low_active_line(MODEM_PWRKEY_PIN, MODEM_PULSE_PWRKEY_MS);
    if (err != ESP_OK)
    {
        result.code = PPP_4G_DIAG_POWER_ON_FAILED;
        ESP_LOGE(TAG, "4G PWRKEY pulse failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    err = modem_prepare_packet_service(&result);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "4G packet service prepare failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    err = ppp_stack_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "  ✗ Initialization Failed!");
        ESP_LOGE(TAG, "========================================");
        goto cleanup;
    }

    err = ppp_start_and_wait_ip(&result);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "  ✗ Initialization Failed!");
        ESP_LOGE(TAG, "========================================");
        goto cleanup;
    }

    result.timing.total_ms = (uint32_t)((esp_timer_get_time() - total_start_us) / 1000LL);
    ppp_4g_log_result(&result);
    report_logged = true;

    if (cb)
    {
        cb();
    }

    return ESP_OK;

cleanup:
    if (!report_logged) {
        result.timing.total_ms = (uint32_t)((esp_timer_get_time() - total_start_us) / 1000LL);
        ppp_4g_log_result(&result);
    }
    ppp_cleanup_failed_init(power_enabled);
    return err;
}

esp_err_t init_4g_mqtt(cb_communication_channel_established cb)
{
    esp_err_t err = ESP_OK;
    bool power_enabled = false;
    bool report_logged = false;
    ppp_4g_diag_result_t result = {
        .code = PPP_4G_DIAG_IO_ERROR,
    };
    int64_t total_start_us = esp_timer_get_time();
    int64_t stage_start_us = total_start_us;

    if (s_module_mqtt_connected) {
        if (cb != NULL) {
            cb();
        }
        return ESP_OK;
    }

    err = modem_gpio_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "4G GPIO init failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    err = modem_uart_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "4G UART init failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    (void)modem_release_low_active_line(MODEM_PWRKEY_PIN);
    (void)gpio_set_level(MODEM_PWRKEY_PIN, 1);
    (void)modem_power_disable();
    vTaskDelay(pdMS_TO_TICKS(100));

    err = modem_power_enable();
    result.timing.power_on_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    if (err != ESP_OK) {
        result.code = PPP_4G_DIAG_POWER_ON_FAILED;
        ESP_LOGE(TAG, "4G power on failed: %s", esp_err_to_name(err));
        goto cleanup;
    }
    power_enabled = true;

    err = modem_pulse_low_active_line(MODEM_PWRKEY_PIN, MODEM_PULSE_PWRKEY_MS);
    if (err != ESP_OK) {
        result.code = PPP_4G_DIAG_POWER_ON_FAILED;
        ESP_LOGE(TAG, "4G PWRKEY pulse failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    err = modem_prepare_packet_service(&result);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "4G packet service prepare failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    err = modem_mqtt_connect_session(&result);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "4G module MQTT connect failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    result.timing.total_ms = (uint32_t)((esp_timer_get_time() - total_start_us) / 1000LL);
    ppp_4g_log_result(&result);
    report_logged = true;

    if (cb != NULL) {
        cb();
    }
    return ESP_OK;

cleanup:
    if (!report_logged) {
        result.timing.total_ms = (uint32_t)((esp_timer_get_time() - total_start_us) / 1000LL);
        ppp_4g_log_result(&result);
    }
    if (power_enabled) {
        char *response = s_modem_response;
        if (s_module_mqtt_connected) {
            (void)modem_mqtt_disconnect(response, MODEM_RESP_BUF_SIZE);
            (void)modem_mqtt_close(response, MODEM_RESP_BUF_SIZE);
            s_module_mqtt_connected = false;
        }
    }
    ppp_cleanup_failed_init(power_enabled);
    return err;
}

esp_err_t bsp_4g_mqtt_publish(const char *topic, const uint8_t *data, size_t len)
{
    return modem_mqtt_publish_binary(topic, data, len);
}

esp_err_t shutdown_4g_mqtt(void)
{
    char *response = s_modem_response;

    if (s_module_mqtt_connected) {
        (void)modem_mqtt_disconnect(response, MODEM_RESP_BUF_SIZE);
        (void)modem_mqtt_close(response, MODEM_RESP_BUF_SIZE);
        s_module_mqtt_connected = false;
    }

    (void)modem_shutdown_gracefully(s_at_ready);
    (void)modem_power_disable();
    modem_uart_deinit();
    s_at_ready = false;
    return ESP_OK;
}

esp_err_t shutdown_ppp_4g(void)
{
    ppp_stop_session();

    if (s_ppp_rx_task_running)
    {
        s_ppp_rx_task_running = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    modem_exit_data_mode();

    (void)modem_shutdown_gracefully(s_at_ready);

    // 关闭电源使能
    (void)modem_power_disable();

    // 删除 UART 驱动
    modem_uart_deinit();
    s_at_ready = false;

    ppp_destroy_netif();

    return ESP_OK;
}
