#include "bsp_4g.h"

#include <stdbool.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "config_manager.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "supercap_manager.h"

#define MODEM_TAG "ec800z_4g"

#define MODEM_UART_PORT UART_NUM_1
#define MODEM_UART_BAUD_RATE 115200
#define MODEM_UART_RX_BUF_SIZE 2048
#define MODEM_UART_TX_BUF_SIZE 1024
#define MODEM_RESP_BUF_SIZE 2048

#define MODEM_GPIO_PWR GPIO_NUM_13
#define MODEM_GPIO_PWRKEY GPIO_NUM_14
#define MODEM_GPIO_UART_TX GPIO_NUM_17
#define MODEM_GPIO_UART_RX GPIO_NUM_18
#define MODEM_GPIO_STATUS GPIO_NUM_21
#define MODEM_GPIO_NET_STATUS GPIO_NUM_47
#define MODEM_GPIO_RESET_N GPIO_NUM_48

#define MODEM_POWER_ENABLE_LEVEL 1
#define MODEM_POWER_DISABLE_LEVEL 0
#define MODEM_PULSE_PWRKEY_MS 600
#define MODEM_POWER_SETTLE_MS 100
#define MODEM_BOOT_TIMEOUT_MS 15000
#define MODEM_SIM_READY_TIMEOUT_MS 10000
#define MODEM_STATUS_LOW_WINDOW_OBSERVE_MS 3000
#define MODEM_STATUS_LOW_SYNC_TIMEOUT_MS 600
#define MODEM_STATUS_POLL_MS 20
#define MODEM_REG_TIMEOUT_MS 60000
#define MODEM_ATTACH_TIMEOUT_MS 30000
#define MODEM_PDP_TIMEOUT_MS 30000
#define MODEM_MQTT_OPEN_TIMEOUT_MS 75000
#define MODEM_MQTT_CONN_TIMEOUT_MS 20000
#define MODEM_SHUTDOWN_TIMEOUT_MS 65000
#define MODEM_CMD_GUARD_MS 150
#define MODEM_MAX_PUBLISH_LEN 1500
#define MODEM_DEFAULT_KEEPALIVE_SEC 60

static SemaphoreHandle_t s_modem_lock = NULL;
static bool s_uart_ready = false;
static bool s_mqtt_connected = false;
static bool s_power_enabled = false;
static int32_t s_next_msg_id = 1;
static char s_modem_response[MODEM_RESP_BUF_SIZE];
static char s_modem_command[256];

void mqtt_proxy_notify_ready(void);
void mqtt_proxy_notify_published(int32_t msg_id);
void mqtt_proxy_notify_disconnected(void);
void mqtt_proxy_notify_error(void);

static int64_t deadline_after_ms(uint32_t timeout_ms)
{
    return esp_timer_get_time() + ((int64_t)timeout_ms * 1000LL);
}

static bool response_has_token(const char *response, const char *token)
{
    return response != NULL && token != NULL && strstr(response, token) != NULL;
}

static void trim_ascii(char *text)
{
    if (text == NULL)
    {
        return;
    }

    char *start = text;
    while (*start != '\0' && isspace((unsigned char)*start))
    {
        ++start;
    }

    char *end = start + strlen(start);
    while (end > start && isspace((unsigned char)end[-1]))
    {
        --end;
    }
    *end = '\0';

    if (start != text)
    {
        memmove(text, start, (size_t)(end - start) + 1);
    }
}

static bool extract_prefixed_value(const char *response,
                                   const char *prefix,
                                   char *out,
                                   size_t out_size)
{
    if (response == NULL || prefix == NULL || out == NULL || out_size == 0)
    {
        return false;
    }

    const char *line = strstr(response, prefix);
    if (line == NULL)
    {
        return false;
    }

    line += strlen(prefix);
    while (*line == ' ' || *line == '\t')
    {
        ++line;
    }

    size_t len = strcspn(line, "\r\n");
    if (len >= out_size)
    {
        len = out_size - 1;
    }
    memcpy(out, line, len);
    out[len] = '\0';
    trim_ascii(out);
    return out[0] != '\0';
}

static bool extract_first_payload_line(const char *response, char *out, size_t out_size)
{
    if (response == NULL || out == NULL || out_size == 0)
    {
        return false;
    }

    const char *cursor = response;
    while (*cursor != '\0')
    {
        const char *line_end = strpbrk(cursor, "\r\n");
        size_t len = line_end == NULL ? strlen(cursor) : (size_t)(line_end - cursor);
        if (len > 0)
        {
            if (!(len == 2 && cursor[0] == 'O' && cursor[1] == 'K') &&
                strstr(cursor, "ERROR") != cursor &&
                strstr(cursor, "AT+") != cursor &&
                strstr(cursor, "AT") != cursor)
            {
                if (len >= out_size)
                {
                    len = out_size - 1;
                }
                memcpy(out, cursor, len);
                out[len] = '\0';
                trim_ascii(out);
                return out[0] != '\0';
            }
        }

        if (line_end == NULL)
        {
            break;
        }
        cursor = line_end;
        while (*cursor == '\r' || *cursor == '\n')
        {
            ++cursor;
        }
    }

    return false;
}

static bool modem_response_is_ok(const char *response)
{
    return response_has_token(response, "\r\nOK\r\n") || response_has_token(response, "\nOK\r\n");
}

static esp_err_t modem_ensure_lock(void)
{
    if (s_modem_lock != NULL)
    {
        return ESP_OK;
    }

    s_modem_lock = xSemaphoreCreateMutex();
    return s_modem_lock != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

static void modem_flush_input(void)
{
    if (s_uart_ready)
    {
        uart_flush_input(MODEM_UART_PORT);
    }
}

static esp_err_t modem_uart_init(void)
{
    if (s_uart_ready)
    {
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
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    err = uart_param_config(MODEM_UART_PORT, &config);
    if (err != ESP_OK)
    {
        return err;
    }

    err = uart_set_pin(MODEM_UART_PORT,
                       MODEM_GPIO_UART_TX,
                       MODEM_GPIO_UART_RX,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        return err;
    }

    s_uart_ready = true;
    modem_flush_input();
    return ESP_OK;
}

static void modem_uart_deinit(void)
{
    if (!s_uart_ready)
    {
        return;
    }

    (void)uart_driver_delete(MODEM_UART_PORT);
    s_uart_ready = false;
}

static esp_err_t modem_gpio_init(void)
{
    const gpio_config_t power_cfg = {
        .pin_bit_mask = 1ULL << MODEM_GPIO_PWR,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&power_cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    const gpio_config_t ctrl_cfg = {
        .pin_bit_mask = (1ULL << MODEM_GPIO_PWRKEY) | (1ULL << MODEM_GPIO_RESET_N),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&ctrl_cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    const gpio_config_t status_cfg = {
        .pin_bit_mask = (1ULL << MODEM_GPIO_STATUS) | (1ULL << MODEM_GPIO_NET_STATUS),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&status_cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    (void)gpio_set_level(MODEM_GPIO_PWRKEY, 1);
    (void)gpio_set_level(MODEM_GPIO_RESET_N, 1);
    (void)gpio_set_level(MODEM_GPIO_PWR, MODEM_POWER_DISABLE_LEVEL);
    return ESP_OK;
}

static esp_err_t modem_power_enable(void)
{
    esp_err_t err = gpio_set_level(MODEM_GPIO_PWR, MODEM_POWER_ENABLE_LEVEL);
    if (err == ESP_OK)
    {
        s_power_enabled = true;
        vTaskDelay(pdMS_TO_TICKS(MODEM_POWER_SETTLE_MS));
    }
    return err;
}

static esp_err_t modem_power_disable(void)
{
    s_power_enabled = false;
    return gpio_set_level(MODEM_GPIO_PWR, MODEM_POWER_DISABLE_LEVEL);
}

static esp_err_t modem_release_control_lines(void)
{
    esp_err_t err = gpio_set_level(MODEM_GPIO_PWRKEY, 1);
    if (err != ESP_OK)
    {
        return err;
    }
    return gpio_set_level(MODEM_GPIO_RESET_N, 1);
}

static int modem_status_level(void)
{
    return gpio_get_level(MODEM_GPIO_STATUS);
}

static bool modem_status_is_on(void)
{
    return modem_status_level() == 0;
}

static esp_err_t modem_wait_for_status_level(int expected_level, uint32_t timeout_ms)
{
    int64_t deadline = deadline_after_ms(timeout_ms);
    while (esp_timer_get_time() < deadline)
    {
        if (modem_status_level() == expected_level)
        {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t modem_pulse_pwrkey(uint32_t pulse_ms)
{
    esp_err_t err = gpio_set_level(MODEM_GPIO_PWRKEY, 0);
    if (err != ESP_OK)
    {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(pulse_ms));
    err = gpio_set_level(MODEM_GPIO_PWRKEY, 1);
    if (err == ESP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(MODEM_CMD_GUARD_MS));
    }
    return err;
}

static esp_err_t modem_read_response(char *response, size_t response_size, uint32_t timeout_ms)
{
    if (response == NULL || response_size == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t used = 0;
    response[0] = '\0';

    int64_t deadline = deadline_after_ms(timeout_ms);
    while (esp_timer_get_time() < deadline)
    {
        uint8_t rx_buf[128];
        int read_len = uart_read_bytes(MODEM_UART_PORT,
                                       rx_buf,
                                       sizeof(rx_buf),
                                       pdMS_TO_TICKS(100));
        if (read_len <= 0)
        {
            continue;
        }

        size_t copy_len = (size_t)read_len;
        if (used + copy_len >= response_size)
        {
            copy_len = response_size - used - 1;
        }

        memcpy(response + used, rx_buf, copy_len);
        used += copy_len;
        response[used] = '\0';

        if (response_has_token(response, "\r\nOK\r\n") ||
            response_has_token(response, "\r\nERROR\r\n") ||
            response_has_token(response, "+CME ERROR:") ||
            response_has_token(response, "POWERED DOWN"))
        {
            return ESP_OK;
        }
    }

    return used > 0 ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t modem_read_until_pattern(char *response,
                                          size_t response_size,
                                          const char *pattern,
                                          uint32_t timeout_ms)
{
    if (response == NULL || response_size == 0 || pattern == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t used = 0;
    response[0] = '\0';

    int64_t deadline = deadline_after_ms(timeout_ms);
    while (esp_timer_get_time() < deadline)
    {
        uint8_t rx_buf[128];
        int read_len = uart_read_bytes(MODEM_UART_PORT,
                                       rx_buf,
                                       sizeof(rx_buf),
                                       pdMS_TO_TICKS(100));
        if (read_len <= 0)
        {
            continue;
        }

        size_t copy_len = (size_t)read_len;
        if (used + copy_len >= response_size)
        {
            copy_len = response_size - used - 1;
        }

        memcpy(response + used, rx_buf, copy_len);
        used += copy_len;
        response[used] = '\0';

        if (strstr(response, pattern) != NULL)
        {
            return ESP_OK;
        }
    }

    return used > 0 ? ESP_ERR_NOT_FOUND : ESP_ERR_TIMEOUT;
}

static esp_err_t modem_wait_for_prompt(uint32_t timeout_ms)
{
    s_modem_response[0] = '\0';
    return modem_read_until_pattern(s_modem_response, sizeof(s_modem_response), ">", timeout_ms);
}

static esp_err_t modem_send_command(const char *cmd,
                                    char *response,
                                    size_t response_size,
                                    uint32_t timeout_ms)
{
    if (cmd == NULL || response == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    modem_flush_input();
    int written = uart_write_bytes(MODEM_UART_PORT, cmd, strlen(cmd));
    if (written < 0)
    {
        return ESP_FAIL;
    }
    written = uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);
    if (written < 0)
    {
        return ESP_FAIL;
    }
    (void)uart_wait_tx_done(MODEM_UART_PORT, pdMS_TO_TICKS(1000));

    return modem_read_response(response, response_size, timeout_ms);
}

static esp_err_t modem_send_command_expect_ok(const char *cmd,
                                              char *response,
                                              size_t response_size,
                                              uint32_t timeout_ms)
{
    esp_err_t err = modem_send_command(cmd, response, response_size, timeout_ms);
    if (err != ESP_OK)
    {
        return err;
    }
    return modem_response_is_ok(response) ? ESP_OK : ESP_FAIL;
}

static esp_err_t modem_sync(void)
{
    int64_t deadline = deadline_after_ms(MODEM_BOOT_TIMEOUT_MS);

    while (esp_timer_get_time() < deadline)
    {
        s_modem_response[0] = '\0';
        esp_err_t err = modem_send_command("AT", s_modem_response, sizeof(s_modem_response), 1000);
        if (err == ESP_OK && modem_response_is_ok(s_modem_response))
        {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t modem_sync_during_status_low_window(bool *window_seen, uint32_t *window_duration_ms)
{
    if (window_seen != NULL)
    {
        *window_seen = false;
    }
    if (window_duration_ms != NULL)
    {
        *window_duration_ms = 0;
    }

    int64_t observe_deadline = deadline_after_ms(MODEM_STATUS_LOW_WINDOW_OBSERVE_MS);
    while (esp_timer_get_time() < observe_deadline)
    {
        if (modem_status_level() != 0)
        {
            vTaskDelay(pdMS_TO_TICKS(MODEM_STATUS_POLL_MS));
            continue;
        }

        if (window_seen != NULL)
        {
            *window_seen = true;
        }

        int64_t window_start_us = esp_timer_get_time();
        int64_t sync_deadline = deadline_after_ms(MODEM_STATUS_LOW_SYNC_TIMEOUT_MS);
        while (esp_timer_get_time() < sync_deadline && modem_status_level() == 0)
        {
            s_modem_response[0] = '\0';
            esp_err_t err = modem_send_command("AT", s_modem_response, sizeof(s_modem_response), 200);
            if (err == ESP_OK && modem_response_is_ok(s_modem_response))
            {
                if (window_duration_ms != NULL)
                {
                    *window_duration_ms = (uint32_t)((esp_timer_get_time() - window_start_us) / 1000LL);
                }
                return ESP_OK;
            }
            vTaskDelay(pdMS_TO_TICKS(MODEM_STATUS_POLL_MS));
        }

        if (window_duration_ms != NULL)
        {
            *window_duration_ms = (uint32_t)((esp_timer_get_time() - window_start_us) / 1000LL);
        }
        return ESP_ERR_TIMEOUT;
    }

    return ESP_ERR_NOT_FOUND;
}

static bool parse_cereg_stat(const char *response, int *stat)
{
    const char *prefix = strstr(response, "+CEREG:");
    if (prefix == NULL || stat == NULL)
    {
        return false;
    }

    int first = 0;
    int second = 0;
    int matched = sscanf(prefix, "+CEREG: %d,%d", &first, &second);
    if (matched == 2)
    {
        *stat = second;
        return true;
    }
    if (matched == 1)
    {
        *stat = first;
        return true;
    }
    return false;
}

static bool parse_cgatt_state(const char *response, int *state)
{
    const char *prefix = strstr(response, "+CGATT:");
    if (prefix == NULL || state == NULL)
    {
        return false;
    }

    return sscanf(prefix, "+CGATT: %d", state) == 1;
}

static bool parse_ip_addr(const char *response, char *out, size_t out_size)
{
    const char *prefix = strstr(response, "+CGPADDR:");
    if (prefix == NULL || out == NULL || out_size == 0)
    {
        return false;
    }

    const char *quote = strchr(prefix, '"');
    if (quote == NULL)
    {
        return false;
    }

    ++quote;
    const char *quote_end = strchr(quote, '"');
    if (quote_end == NULL)
    {
        return false;
    }

    size_t len = (size_t)(quote_end - quote);
    if (len >= out_size)
    {
        len = out_size - 1;
    }
    memcpy(out, quote, len);
    out[len] = '\0';
    return true;
}

static bool modem_ip_addr_is_usable(const char *ip_addr)
{
    if (ip_addr == NULL || ip_addr[0] == '\0')
    {
        return false;
    }

    if (strcmp(ip_addr, "0.0.0.0") == 0)
    {
        return false;
    }

    return true;
}

static bool parse_qmtopen_result(const char *response, int *client_idx, int *result_code)
{
    const char *prefix = strstr(response, "+QMTOPEN:");
    if (prefix == NULL || client_idx == NULL || result_code == NULL)
    {
        return false;
    }

    return sscanf(prefix, "+QMTOPEN: %d,%d", client_idx, result_code) == 2;
}

static bool parse_qmtconn_result(const char *response,
                                 int *client_idx,
                                 int *conn_result,
                                 int *retcode)
{
    const char *prefix = strstr(response, "+QMTCONN:");
    if (prefix == NULL || client_idx == NULL || conn_result == NULL || retcode == NULL)
    {
        return false;
    }

    return sscanf(prefix, "+QMTCONN: %d,%d,%d", client_idx, conn_result, retcode) == 3;
}

static bool parse_qmtpubex_result(const char *response,
                                  int *client_idx,
                                  int *msg_id,
                                  int *result_code)
{
    const char *prefix = strstr(response, "+QMTPUBEX:");
    if (prefix == NULL || client_idx == NULL || msg_id == NULL || result_code == NULL)
    {
        return false;
    }

    return sscanf(prefix, "+QMTPUBEX: %d,%d,%d", client_idx, msg_id, result_code) == 3;
}

static esp_err_t parse_broker_endpoint(const char *raw_host,
                                       char *host_out,
                                       size_t host_out_size,
                                       int *port_out)
{
    if (raw_host == NULL || host_out == NULL || host_out_size == 0 || port_out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const char *host = raw_host;
    if (strncmp(host, "mqtt://", 7) == 0)
    {
        host += 7;
    }

    size_t host_len = strcspn(host, ":/");
    if (host_len == 0 || host_len >= host_out_size)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(host_out, host, host_len);
    host_out[host_len] = '\0';

    *port_out = 1883;
    if (host[host_len] == ':')
    {
        int parsed_port = atoi(host + host_len + 1);
        if (parsed_port <= 0 || parsed_port > 65535)
        {
            return ESP_ERR_INVALID_ARG;
        }
        *port_out = parsed_port;
    }

    return ESP_OK;
}

static esp_err_t modem_wait_for_sim_ready(void)
{
    s_modem_response[0] = '\0';
    if (modem_send_command("AT+QINISTAT", s_modem_response, sizeof(s_modem_response), 1000) == ESP_OK)
    {
        char qinistat[16] = {0};
        if (extract_prefixed_value(s_modem_response, "+QINISTAT:", qinistat, sizeof(qinistat)))
        {
            ESP_LOGI(MODEM_TAG, "USIM init state: %s", qinistat);
        }
    }

    int64_t deadline = deadline_after_ms(MODEM_SIM_READY_TIMEOUT_MS);

    while (esp_timer_get_time() < deadline)
    {
        s_modem_response[0] = '\0';
        esp_err_t err = modem_send_command("AT+CPIN?", s_modem_response, sizeof(s_modem_response), 3000);
        if (err == ESP_OK && response_has_token(s_modem_response, "+CPIN: READY"))
        {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t modem_wait_for_registration(void)
{
    int64_t deadline = deadline_after_ms(MODEM_REG_TIMEOUT_MS);
    int last_stat = -1;

    while (esp_timer_get_time() < deadline)
    {
        s_modem_response[0] = '\0';
        esp_err_t err = modem_send_command("AT+CEREG?", s_modem_response, sizeof(s_modem_response), 1000);
        int stat = 0;
        if (err == ESP_OK && parse_cereg_stat(s_modem_response, &stat) && (stat == 1 || stat == 5))
        {
            ESP_LOGI(MODEM_TAG, "EPS registration ready, CEREG=%d", stat);
            return ESP_OK;
        }
        if (err == ESP_OK && parse_cereg_stat(s_modem_response, &stat) && stat != last_stat)
        {
            last_stat = stat;
            ESP_LOGI(MODEM_TAG, "Waiting for EPS registration, CEREG=%d", stat);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t modem_wait_for_attach(void)
{
    s_modem_response[0] = '\0';
    esp_err_t err = modem_send_command_expect_ok("AT+CGATT=1", s_modem_response, sizeof(s_modem_response), 5000);
    if (err != ESP_OK)
    {
        return err;
    }

    int64_t deadline = deadline_after_ms(MODEM_ATTACH_TIMEOUT_MS);
    int last_state = -1;
    while (esp_timer_get_time() < deadline)
    {
        int state = 0;
        s_modem_response[0] = '\0';
        err = modem_send_command("AT+CGATT?", s_modem_response, sizeof(s_modem_response), 1000);
        if (err == ESP_OK && parse_cgatt_state(s_modem_response, &state) && state == 1)
        {
            ESP_LOGI(MODEM_TAG, "PS attach ready, CGATT=%d", state);
            return ESP_OK;
        }
        if (err == ESP_OK && parse_cgatt_state(s_modem_response, &state) && state != last_state)
        {
            last_state = state;
            ESP_LOGI(MODEM_TAG, "Waiting for PS attach, CGATT=%d", state);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t modem_activate_pdp(char *ip_addr, size_t ip_addr_size)
{
    s_modem_response[0] = '\0';
    esp_err_t err = modem_send_command_expect_ok("AT+CGDCONT=1,\"IP\",\"\"",
                                                 s_modem_response,
                                                 sizeof(s_modem_response),
                                                 3000);
    if (err != ESP_OK)
    {
        return err;
    }

    s_modem_response[0] = '\0';
    err = modem_send_command_expect_ok("AT+CGACT=1,1", s_modem_response, sizeof(s_modem_response), 10000);
    if (err != ESP_OK)
    {
        return err;
    }

    int64_t deadline = deadline_after_ms(MODEM_PDP_TIMEOUT_MS);
    while (esp_timer_get_time() < deadline)
    {
        s_modem_response[0] = '\0';
        err = modem_send_command("AT+CGPADDR=1", s_modem_response, sizeof(s_modem_response), 1000);
        if (err == ESP_OK &&
            parse_ip_addr(s_modem_response, ip_addr, ip_addr_size) &&
            modem_ip_addr_is_usable(ip_addr))
        {
            ESP_LOGI(MODEM_TAG, "PDP address acquired: %s", ip_addr);
            return ESP_OK;
        }
        if (err == ESP_OK && parse_ip_addr(s_modem_response, ip_addr, ip_addr_size))
        {
            ESP_LOGW(MODEM_TAG, "PDP address not usable yet: %s", ip_addr);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t modem_open_mqtt_network(const char *host, int port)
{
    s_modem_response[0] = '\0';
    esp_err_t err = modem_send_command_expect_ok("AT+QMTCFG=\"VERSION\",0,4",
                                                 s_modem_response,
                                                 sizeof(s_modem_response),
                                                 1000);
    if (err != ESP_OK)
    {
        return err;
    }

    snprintf(s_modem_command,
             sizeof(s_modem_command),
             "AT+QMTCFG=\"KEEPALIVE\",0,%d",
             MODEM_DEFAULT_KEEPALIVE_SEC);
    s_modem_response[0] = '\0';
    err = modem_send_command_expect_ok(s_modem_command, s_modem_response, sizeof(s_modem_response), 1000);
    if (err != ESP_OK)
    {
        return err;
    }

    snprintf(s_modem_command, sizeof(s_modem_command), "AT+QMTOPEN=0,\"%s\",%d", host, port);
    s_modem_response[0] = '\0';
    err = modem_send_command_expect_ok(s_modem_command, s_modem_response, sizeof(s_modem_response), 1000);
    if (err != ESP_OK)
    {
        return err;
    }

    s_modem_response[0] = '\0';
    err = modem_read_until_pattern(s_modem_response, sizeof(s_modem_response), "+QMTOPEN:", MODEM_MQTT_OPEN_TIMEOUT_MS);
    if (err != ESP_OK)
    {
        return err;
    }

    int client_idx = -1;
    int result_code = -1;
    if (!parse_qmtopen_result(s_modem_response, &client_idx, &result_code))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return (client_idx == 0 && result_code == 0) ? ESP_OK : ESP_FAIL;
}

static esp_err_t modem_connect_mqtt_client(const char *client_id)
{
    snprintf(s_modem_command, sizeof(s_modem_command), "AT+QMTCONN=0,\"%s\"", client_id);
    s_modem_response[0] = '\0';
    esp_err_t err = modem_send_command_expect_ok(s_modem_command, s_modem_response, sizeof(s_modem_response), 1000);
    if (err != ESP_OK)
    {
        return err;
    }

    s_modem_response[0] = '\0';
    err = modem_read_until_pattern(s_modem_response, sizeof(s_modem_response), "+QMTCONN:", MODEM_MQTT_CONN_TIMEOUT_MS);
    if (err != ESP_OK)
    {
        return err;
    }

    int client_idx = -1;
    int conn_result = -1;
    int retcode = -1;
    if (!parse_qmtconn_result(s_modem_response, &client_idx, &conn_result, &retcode))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return (client_idx == 0 && conn_result == 0 && retcode == 0) ? ESP_OK : ESP_FAIL;
}

static void modem_log_identity(void)
{
    char value[128] = {0};

    s_modem_response[0] = '\0';
    if (modem_send_command("AT+CGMM", s_modem_response, sizeof(s_modem_response), 1000) == ESP_OK)
    {
        if (extract_first_payload_line(s_modem_response, value, sizeof(value)))
        {
            ESP_LOGI(MODEM_TAG, "module model: %s", value);
        }
    }
    s_modem_response[0] = '\0';
    if (modem_send_command("AT+QCCID", s_modem_response, sizeof(s_modem_response), 1000) == ESP_OK)
    {
        if (extract_prefixed_value(s_modem_response, "+QCCID:", value, sizeof(value)))
        {
            ESP_LOGI(MODEM_TAG, "SIM ICCID: %s", value);
        }
    }
    s_modem_response[0] = '\0';
    if (modem_send_command("AT+QNWINFO", s_modem_response, sizeof(s_modem_response), 1000) == ESP_OK)
    {
        if (extract_prefixed_value(s_modem_response, "+QNWINFO:", value, sizeof(value)))
        {
            ESP_LOGI(MODEM_TAG, "network info: %s", value);
        }
        else
        {
            ESP_LOGI(MODEM_TAG, "network info unavailable before registration");
        }
    }
}

static esp_err_t modem_disconnect_mqtt(void)
{
    if (!s_uart_ready)
    {
        return ESP_OK;
    }

    ESP_LOGI(MODEM_TAG, "Disconnecting EC800Z MQTT session");

    s_modem_response[0] = '\0';
    (void)modem_send_command("AT+QMTDISC=0", s_modem_response, sizeof(s_modem_response), 1000);
    s_modem_response[0] = '\0';
    (void)modem_read_until_pattern(s_modem_response, sizeof(s_modem_response), "+QMTDISC:", 5000);
    s_modem_response[0] = '\0';
    (void)modem_send_command("AT+QMTCLOSE=0", s_modem_response, sizeof(s_modem_response), 1000);
    s_modem_response[0] = '\0';
    (void)modem_read_until_pattern(s_modem_response, sizeof(s_modem_response), "+QMTCLOSE:", 5000);
    return ESP_OK;
}

static esp_err_t modem_shutdown_gracefully(bool at_ready)
{
    if (at_ready && s_uart_ready)
    {
        ESP_LOGI(MODEM_TAG, "Requesting graceful EC800Z shutdown via AT+QPOWD");
        modem_flush_input();
        static const char shutdown_cmd[] = "AT+QPOWD=1\r\n";
        int written = uart_write_bytes(MODEM_UART_PORT, shutdown_cmd, sizeof(shutdown_cmd) - 1);
        if (written >= 0)
        {
            (void)uart_wait_tx_done(MODEM_UART_PORT, pdMS_TO_TICKS(1000));
            s_modem_response[0] = '\0';
            (void)modem_read_response(s_modem_response, sizeof(s_modem_response), 1000);
            if (response_has_token(s_modem_response, "POWERED DOWN"))
            {
                ESP_LOGI(MODEM_TAG, "EC800Z reported POWERED DOWN");
                return ESP_OK;
            }

            if (modem_response_is_ok(s_modem_response))
            {
                ESP_LOGI(MODEM_TAG, "EC800Z accepted QPOWD, waiting for POWERED DOWN");
                s_modem_response[0] = '\0';
                if (modem_read_until_pattern(s_modem_response,
                                             sizeof(s_modem_response),
                                             "POWERED DOWN",
                                             MODEM_SHUTDOWN_TIMEOUT_MS) == ESP_OK)
                {
                    ESP_LOGI(MODEM_TAG, "EC800Z reported POWERED DOWN");
                    return ESP_OK;
                }
            }

            if (modem_wait_for_status_level(1, 5000) == ESP_OK)
            {
                ESP_LOGI(MODEM_TAG, "EC800Z STATUS indicates shutdown complete");
                return ESP_OK;
            }
        }
    }

    ESP_LOGW(MODEM_TAG, "Graceful shutdown did not complete, pulsing PWRKEY fallback");
    return modem_pulse_pwrkey(650);
}

esp_err_t init_ppp_4g(cb_communication_channel_established cb)
{
    esp_err_t err = modem_ensure_lock();
    if (err != ESP_OK)
    {
        return err;
    }

    if (xSemaphoreTake(s_modem_lock, pdMS_TO_TICKS(120000)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    if (s_mqtt_connected)
    {
        xSemaphoreGive(s_modem_lock);
        mqtt_proxy_notify_ready();
        if (cb)
        {
            cb();
        }
        return ESP_OK;
    }

    bool supercap_ready = false;
    if (CONFIG_SENTINEL_4G_SKIP_PRECHARGE)
    {
        ESP_LOGW(MODEM_TAG, "Skipping supercap precharge before 4G bring-up by configuration");
        supercap_ready = false;
    }
    else
    {
        err = supercap_prepare_for_4g(true, &supercap_ready);
    }
    if (err != ESP_OK)
    {
        xSemaphoreGive(s_modem_lock);
        return err;
    }
    if (!CONFIG_SENTINEL_4G_SKIP_PRECHARGE && supercap_ready)
    {
        ESP_LOGI(MODEM_TAG, "Supercap precharge ready, continuing 4G bring-up");
    }
    else if (!CONFIG_SENTINEL_4G_SKIP_PRECHARGE)
    {
        ESP_LOGW(MODEM_TAG, "Supercap precharge completed without reaching supply gate, continuing 4G bring-up");
    }

    bool at_ready = false;
    bool low_window_seen = false;
    bool low_window_sync = false;
    uint32_t low_window_duration_ms = 0;
    char ip_addr[32] = {0};
    char broker_host[LEN_MAX_HOST] = {0};
    int broker_port = 1883;

    err = modem_gpio_init();
    if (err != ESP_OK)
    {
        goto fail;
    }

    err = modem_uart_init();
    if (err != ESP_OK)
    {
        goto fail;
    }

    err = modem_release_control_lines();
    if (err != ESP_OK)
    {
        goto fail;
    }

    err = modem_power_enable();
    if (err != ESP_OK)
    {
        goto fail;
    }

    err = modem_sync_during_status_low_window(&low_window_seen, &low_window_duration_ms);
    if (err == ESP_OK)
    {
        low_window_sync = true;
    }
    else
    {
        err = modem_sync();
    }

    if (err != ESP_OK)
    {
        err = modem_pulse_pwrkey(MODEM_PULSE_PWRKEY_MS);
        if (err != ESP_OK)
        {
            goto fail;
        }

        err = modem_sync_during_status_low_window(&low_window_seen, &low_window_duration_ms);
        if (err == ESP_OK)
        {
            low_window_sync = true;
        }
        else
        {
            err = modem_sync();
        }
        if (err != ESP_OK)
        {
            goto fail;
        }
    }
    at_ready = true;
    ESP_LOGI(MODEM_TAG,
             "boot handshake complete: status_level=%d status_on=%s low_window_seen=%s low_window_sync=%s low_window_ms=%u",
             modem_status_level(),
             modem_status_is_on() ? "true" : "false",
             low_window_seen ? "true" : "false",
             low_window_sync ? "true" : "false",
             low_window_duration_ms);

    {
        s_modem_response[0] = '\0';
        (void)modem_send_command_expect_ok("ATE0", s_modem_response, sizeof(s_modem_response), 1000);
        s_modem_response[0] = '\0';
        (void)modem_send_command_expect_ok("AT+CMEE=2", s_modem_response, sizeof(s_modem_response), 1000);
    }

    err = modem_wait_for_sim_ready();
    if (err != ESP_OK)
    {
        goto fail;
    }

    modem_log_identity();

    ESP_LOGI(MODEM_TAG, "Waiting for cellular registration...");
    err = modem_wait_for_registration();
    if (err != ESP_OK)
    {
        goto fail;
    }

    ESP_LOGI(MODEM_TAG, "Waiting for PS attach...");
    err = modem_wait_for_attach();
    if (err != ESP_OK)
    {
        goto fail;
    }

    ESP_LOGI(MODEM_TAG, "Activating PDP context and waiting for IP address...");
    err = modem_activate_pdp(ip_addr, sizeof(ip_addr));
    if (err != ESP_OK)
    {
        goto fail;
    }
    ESP_LOGI(MODEM_TAG, "4G initialization reached IP-ready state, IP=%s", ip_addr);

    if (g_user_config.host[0] == '\0')
    {
        err = ESP_ERR_INVALID_STATE;
        goto fail;
    }

    err = parse_broker_endpoint(g_user_config.host, broker_host, sizeof(broker_host), &broker_port);
    if (err != ESP_OK)
    {
        goto fail;
    }

    err = modem_open_mqtt_network(broker_host, broker_port);
    if (err != ESP_OK)
    {
        goto fail;
    }
    ESP_LOGI(MODEM_TAG, "MQTT transport opened via %s:%d", broker_host, broker_port);

    err = modem_connect_mqtt_client(g_user_config.device_id[0] != '\0' ?
                                        g_user_config.device_id :
                                        "sentinel-4g");
    if (err != ESP_OK)
    {
        goto fail;
    }
    ESP_LOGI(MODEM_TAG, "MQTT connected, 4G initialization complete");

    s_mqtt_connected = true;
    xSemaphoreGive(s_modem_lock);

    mqtt_proxy_notify_ready();
    if (cb)
    {
        cb();
    }
    return ESP_OK;

fail:
    ESP_LOGE(MODEM_TAG, "EC800Z bring-up failed: %s", esp_err_to_name(err));
    mqtt_proxy_notify_error();
    s_mqtt_connected = false;
    (void)modem_disconnect_mqtt();
    (void)modem_shutdown_gracefully(at_ready);
    if (s_power_enabled)
    {
        (void)modem_power_disable();
    }
    modem_uart_deinit();
    xSemaphoreGive(s_modem_lock);
    return err;
}

esp_err_t shutdown_ppp_4g(void)
{
    esp_err_t err = modem_ensure_lock();
    if (err != ESP_OK)
    {
        return err;
    }

    if (xSemaphoreTake(s_modem_lock, pdMS_TO_TICKS(120000)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(MODEM_TAG, "Shutting down EC800Z transport and powering module off");

    (void)modem_disconnect_mqtt();
    (void)modem_shutdown_gracefully(s_uart_ready);
    if (s_power_enabled)
    {
        (void)modem_power_disable();
        ESP_LOGI(MODEM_TAG, "EC800Z power rail disabled");
    }
    modem_uart_deinit();
    s_mqtt_connected = false;
    xSemaphoreGive(s_modem_lock);

    ESP_LOGI(MODEM_TAG, "EC800Z shutdown sequence complete");
    mqtt_proxy_notify_disconnected();
    return ESP_OK;
}

int32_t bsp_4g_mqtt_publish(const char *topic, const uint8_t *data, size_t len, int qos, bool retain)
{
    if (topic == NULL || data == NULL || len == 0 || len > MODEM_MAX_PUBLISH_LEN)
    {
        return -1;
    }

    esp_err_t err = modem_ensure_lock();
    if (err != ESP_OK)
    {
        return -1;
    }

    if (xSemaphoreTake(s_modem_lock, pdMS_TO_TICKS(120000)) != pdTRUE)
    {
        return -1;
    }

    if (!s_mqtt_connected || !s_uart_ready)
    {
        xSemaphoreGive(s_modem_lock);
        return -1;
    }

    int32_t msg_id = s_next_msg_id++;
    if (s_next_msg_id > 65535)
    {
        s_next_msg_id = 1;
    }

    snprintf(s_modem_command,
             sizeof(s_modem_command),
             "AT+QMTPUBEX=0,%ld,%d,%d,\"%s\",%u",
             (long)msg_id,
             qos,
             retain ? 1 : 0,
             topic,
             (unsigned)len);

    modem_flush_input();
    int written = uart_write_bytes(MODEM_UART_PORT, s_modem_command, strlen(s_modem_command));
    if (written < 0)
    {
        xSemaphoreGive(s_modem_lock);
        return -1;
    }
    written = uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);
    if (written < 0)
    {
        xSemaphoreGive(s_modem_lock);
        return -1;
    }
    (void)uart_wait_tx_done(MODEM_UART_PORT, pdMS_TO_TICKS(1000));

    err = modem_wait_for_prompt(3000);
    if (err != ESP_OK)
    {
        s_mqtt_connected = false;
        xSemaphoreGive(s_modem_lock);
        return -1;
    }

    written = uart_write_bytes(MODEM_UART_PORT, data, len);
    if (written < 0 || (size_t)written != len)
    {
        s_mqtt_connected = false;
        xSemaphoreGive(s_modem_lock);
        return -1;
    }
    (void)uart_wait_tx_done(MODEM_UART_PORT, pdMS_TO_TICKS(3000));

    s_modem_response[0] = '\0';
    err = modem_read_until_pattern(s_modem_response, sizeof(s_modem_response), "+QMTPUBEX:", 15000);
    if (err != ESP_OK)
    {
        s_mqtt_connected = false;
        xSemaphoreGive(s_modem_lock);
        return -1;
    }

    int client_idx = -1;
    int ack_msg_id = -1;
    int result_code = -1;
    if (!parse_qmtpubex_result(s_modem_response, &client_idx, &ack_msg_id, &result_code) ||
        client_idx != 0 || ack_msg_id != msg_id || result_code != 0)
    {
        s_mqtt_connected = false;
        xSemaphoreGive(s_modem_lock);
        return -1;
    }

    xSemaphoreGive(s_modem_lock);
    mqtt_proxy_notify_published(msg_id);
    return msg_id;
}
