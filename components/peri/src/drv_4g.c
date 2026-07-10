#include "drv_4g.h"
#include "bsp_board.h"
#include "board_config.h"
#include "config_manager.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include <stdint.h>
#include <stdlib.h>
#include <sys/param.h>
#include <stdbool.h>
#include "esp_ota_ops.h"
#include <time.h>
#include <sys/time.h>
#include "logger.h"

#ifndef SN
#define SN "0"
#endif

// ============== Board pin aliases ==============
#define MODEM_UART_RX_PIN DRV_4G_PIN_UART_RX
#define MODEM_UART_TX_PIN DRV_4G_PIN_UART_TX
#define MODEM_PWR_EN_PIN DRV_4G_PIN_PWR
#define MODEM_PWRKEY_PIN DRV_4G_PIN_PWRKEY
#define MODEM_STATUS_PIN DRV_4G_PIN_STATUS
#define MODEM_NET_STATUS_PIN DRV_4G_PIN_NET_STATUS
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
#define MODEM_REG_POLL_MS 200
#define MODEM_SYNC_AT_CMD "AT"
#define MODEM_HTTP_POST_CONNECT_TIMEOUT_MS 125000
#define MODEM_HTTP_RESP_BUF_SIZE 4096

// UART & AT state
static bool s_uart_driver_installed = false;
static bool s_at_ready = false;
static bool s_module_network_ready = false;
static char s_modem_response[MODEM_RESP_BUF_SIZE];
static volatile bool s_at_cmd_active = false;
static SemaphoreHandle_t s_at_mutex = NULL;

static int64_t days_from_civil(int year, int month, int day)
{
    year -= month <= 2;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yoe = (unsigned)(year - era * 400);
    const unsigned month_adjusted = (unsigned)(month + (month > 2 ? -3 : 9));
    const unsigned doy = (153 * month_adjusted + 2) / 5 + (unsigned)day - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

static time_t utc_epoch_from_modem_time(
    int year,
    int month,
    int day,
    int hour,
    int min,
    int sec)
{
    const int full_year = 2000 + year;
    const int64_t days = days_from_civil(full_year, month, day);
    const int64_t utc_seconds =
        days * 86400 + (int64_t)hour * 3600 + (int64_t)min * 60 + sec;
    return (time_t)utc_seconds;
}

static void ensure_at_mutex(void)
{
    if (s_at_mutex == NULL)
    {
        s_at_mutex = xSemaphoreCreateMutex();
    }
}

typedef enum
{
    PPP_4G_DIAG_OK = 0,
    PPP_4G_DIAG_POWER_ON_FAILED,
    PPP_4G_DIAG_AT_NO_RESPONSE,
    PPP_4G_DIAG_SIM_NOT_READY,
    PPP_4G_DIAG_NOT_REGISTERED,
    PPP_4G_DIAG_ATTACH_FAILED,
    PPP_4G_DIAG_PDP_FAILED,
    PPP_4G_DIAG_NO_IP,
    PPP_4G_DIAG_IO_ERROR,
} ppp_4g_diag_code_t;

typedef struct
{
    uint32_t power_on_ms;
    uint32_t boot_ms;
    uint32_t sim_ready_ms;
    uint32_t network_attach_ms;
    uint32_t pdp_active_ms;
    uint32_t ppp_ms;
    uint32_t total_ms;
} ppp_4g_diag_timing_t;

typedef struct
{
    ppp_4g_diag_code_t code;
    ppp_4g_diag_timing_t timing;
    bool sim_ready;
    bool registered;
    bool attached;
    bool pdp_active;
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
    if (response == NULL || pattern == NULL)
    {
        return false;
    }

    size_t pattern_len = strlen(pattern);
    if (pattern_len == 0 || response_len < pattern_len)
    {
        return false;
    }

    for (size_t i = 0; i <= response_len - pattern_len; ++i)
    {
        if (memcmp(response + i, pattern, pattern_len) == 0)
        {
            if (offset != NULL)
            {
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
    switch (code)
    {
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
    case PPP_4G_DIAG_IO_ERROR:
        return "io_error";
    default:
        return "unknown";
    }
}

static void ppp_4g_log_result(const ppp_4g_diag_result_t *result)
{
    if (result == NULL)
    {
        return;
    }

    // LOG_DEBUGF("4G Module Startup Timing Report");
    LOG_DEBUGF("Result: %s", ppp_4g_diag_code_to_str(result->code));
    LOG_DEBUG("---------------------------------------");
    LOG_DEBUGF("1. Power Enable : %lu ms", (unsigned long)result->timing.power_on_ms);
    LOG_DEBUGF("2. AT Handshake : %lu ms", (unsigned long)result->timing.boot_ms);
    LOG_DEBUGF("3. SIM Ready    : %lu ms", (unsigned long)result->timing.sim_ready_ms);
    LOG_DEBUGF("4. Network Reg  : %lu ms", (unsigned long)result->timing.network_attach_ms);
    LOG_DEBUGF("5. PDP/IP       : %lu ms  (active=%s ip=%s)",
             (unsigned long)result->timing.pdp_active_ms,
             result->pdp_active ? "true" : "false",
             result->ip_addr[0] != '\0' ? result->ip_addr : "none");
    LOG_DEBUG("6. Transport    : HTTP/network-only");
    LOG_DEBUG("---------------------------------------");
    LOG_DEBUGF("Total Time      : %lu ms", (unsigned long)result->timing.total_ms);
    LOG_DEBUG("---------------------------------------");
}

static void modem_copy_cgpaddr_ip(const char *response, char *ip_addr, size_t ip_addr_size)
{
    if (response == NULL || ip_addr == NULL || ip_addr_size == 0)
    {
        return;
    }

    const char *line = strstr(response, "+CGPADDR:");
    if (line == NULL)
    {
        return;
    }

    const char *comma = strchr(line, ',');
    if (comma == NULL)
    {
        return;
    }

    const char *start = comma + 1;
    while (*start == ' ' || *start == '"')
    {
        ++start;
    }

    size_t len = 0;
    while (start[len] != '\0' &&
           start[len] != '"' &&
           start[len] != '\r' &&
           start[len] != '\n' &&
           start[len] != ',')
    {
        ++len;
    }

    if (len == 0 || len >= ip_addr_size)
    {
        return;
    }
    memcpy(ip_addr, start, len);
    ip_addr[len] = '\0';
}

static esp_err_t modem_gpio_init(void)
{
    // 如果之前休眠时锁定了引脚状态，下一次初始化前必须先解除锁定
    gpio_hold_dis(MODEM_PWR_EN_PIN);

    const gpio_config_t power_cfg = {
        .pin_bit_mask = 1ULL << MODEM_PWR_EN_PIN,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&power_cfg);
    if (err != ESP_OK)
    {
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
    if (err != ESP_OK)
    {
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
    if (err != ESP_OK)
    {
        return err;
    }

    (void)gpio_set_level(MODEM_PWRKEY_PIN, 1);
    return ESP_OK;
}

static esp_err_t modem_read_response(char *response, size_t response_size, uint32_t timeout_ms);
static esp_err_t modem_uart_init(void);

// ================= 新增：通过 4G 基站或内部 NTP 获取时间并同步给 ESP32 =================
esp_err_t bsp_4g_sync_time(void)
{
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);

    char response[MODEM_RESP_BUF_SIZE];
    esp_err_t err = ESP_FAIL;
    s_at_cmd_active = true;

    err = modem_gpio_init();
    if (err != ESP_OK)
    {
        LOG_ERRORF("4G GPIO init failed before time sync: %s", esp_err_to_name(err));
        goto cleanup;
    }

    err = modem_uart_init();
    if (err != ESP_OK)
    {
        LOG_ERRORF("4G UART init failed before time sync: %s", esp_err_to_name(err));
        goto cleanup;
    }

    // 发送 AT+CCLK? 查询模块当前时间 (格式: +CCLK: "24/06/15,12:30:45+32")
    uart_flush_input(UART_PORT_NUM);
    uart_write_bytes(UART_PORT_NUM, "AT+CCLK?\r\n", 10);
    err = modem_read_response(response, sizeof(response), 2000);

    if (err == ESP_OK)
    {
        int year, month, day, hour, min, sec;
        int tz_quarters = 0;
        char *line = strstr(response, "+CCLK: \"");
        int parsed = 0;
        if (line)
        {
            parsed = sscanf(
                line,
                "+CCLK: \"%d/%d/%d,%d:%d:%d%d",
                &year,
                &month,
                &day,
                &hour,
                &min,
                &sec,
                &tz_quarters);
        }

        if (parsed >= 6)
        {
            if (year >= 24 && month >= 1 && month <= 12 && day >= 1 && day <= 31 &&
                hour >= 0 && hour <= 23 && min >= 0 && min <= 59 && sec >= 0 && sec <= 60)
            { // 确保时间大于 2024 年，排除模块自身的默认初始时间 1980/2004 等
                time_t t = utc_epoch_from_modem_time(
                    year,
                    month,
                    day,
                    hour,
                    min,
                    sec);

                struct timeval tv = {.tv_sec = t, .tv_usec = 0};
                settimeofday(&tv, NULL); // 强制修改 ESP32 的硬件 RTC 系统时间

                LOG_DEBUGF(
                    "Time synced from 4G Base Station: 20%02d-%02d-%02d %02d:%02d:%02d tz_quarters=%d",
                    year,
                    month,
                    day,
                    hour,
                    min,
                    sec,
                    parsed >= 7 ? tz_quarters : 0);
                err = ESP_OK;
            }
            else
            {
                LOG_WARNF(
                    "4G time not updated yet or invalid (20%02d-%02d-%02d %02d:%02d:%02d). Retry needed.",
                    year,
                    month,
                    day,
                    hour,
                    min,
                    sec);
                err = ESP_FAIL;
            }
        }
    }

cleanup:
    s_at_cmd_active = false;
    xSemaphoreGive(s_at_mutex);
    return err;
}
static esp_err_t modem_uart_init(void)
{
    if (s_uart_driver_installed)
    {
        return ESP_OK;
    }

    if (uart_is_driver_installed(UART_PORT_NUM))
    {
        (void)uart_driver_delete(UART_PORT_NUM);
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
    if (err != ESP_OK)
    {
        return err;
    }

    err = uart_param_config(UART_PORT_NUM, &config);
    if (err != ESP_OK)
    {
        return err;
    }

    err = uart_set_pin(UART_PORT_NUM,
                       MODEM_UART_TX_PIN,
                       MODEM_UART_RX_PIN,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        return err;
    }

    err = uart_flush_input(UART_PORT_NUM);
    if (err != ESP_OK)
    {
        return err;
    }

    s_uart_driver_installed = true;
    return ESP_OK;
}

static void modem_uart_deinit(void)
{
    if (s_uart_driver_installed)
    {
        (void)uart_driver_delete(UART_PORT_NUM);
        s_uart_driver_installed = false;
    }
}

static esp_err_t modem_power_enable(void)
{
    esp_err_t err = gpio_set_level(MODEM_PWR_EN_PIN, MODEM_POWER_ENABLE_LEVEL);
    if (err == ESP_OK)
    {
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
    if (err != ESP_OK)
    {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(pulse_ms));
    return gpio_set_level(pin, 1);
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
        int read_len = uart_read_bytes(UART_PORT_NUM,
                                       rx_buf,
                                       sizeof(rx_buf),
                                       pdMS_TO_TICKS(100));
        if (read_len > 0)
        {
            size_t copy_len = (size_t)read_len;
            // 在 modem_read_until_pattern 的截断处改为：
            if (used + copy_len >= response_size)
            {
                // 保留后半段，丢弃前半段（pattern 不可能跨越太长距离）
                size_t keep = response_size / 2;
                memmove(response, response + used - keep, keep);
                used = keep;
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
        int read_len = uart_read_bytes(UART_PORT_NUM,
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

        size_t pattern_offset = 0;
        if (response_find_pattern_offset(response, used, pattern, &pattern_offset))
        {
            if (pattern_offset > 0)
            {
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
    if (output == NULL || output_size == 0)
    {
        return;
    }

    size_t out = 0;
    output[0] = '\0';
    if (input == NULL)
    {
        return;
    }

    for (size_t i = 0; input[i] != '\0' && out + 1 < output_size; ++i)
    {
        unsigned char ch = (unsigned char)input[i];
        const char *escaped = NULL;
        char hex[5] = {0};

        switch (ch)
        {
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
            if (!isprint(ch))
            {
                snprintf(hex, sizeof(hex), "\\x%02X", ch);
                escaped = hex;
            }
            break;
        }

        if (escaped != NULL)
        {
            size_t escaped_len = strlen(escaped);
            if (out + escaped_len >= output_size)
            {
                break;
            }
            memcpy(output + out, escaped, escaped_len);
            out += escaped_len;
        }
        else
        {
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
    if (cmd == NULL || response == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

#if defined(AT) && AT == 1
    char tx_visible[160];
    char tx_frame[128];
    int tx_frame_len = snprintf(tx_frame, sizeof(tx_frame), "%s\r\n", cmd);
    if (tx_frame_len < 0)
    {
        return ESP_FAIL;
    }
    format_visible_bytes(tx_frame, tx_visible, sizeof(tx_visible));
    LOG_DEBUGF(">>> UART TX AT frame=\"%s\" cmd=\"%s\" len=%d", tx_visible, cmd, tx_frame_len);
#endif

    s_at_cmd_active = true;

    (void)uart_flush_input(UART_PORT_NUM);
    int written = uart_write_bytes(UART_PORT_NUM, cmd, (size_t)strlen(cmd));
    if (written < 0)
    {
        return ESP_FAIL;
    }
    written = uart_write_bytes(UART_PORT_NUM, "\r\n", 2);
    if (written < 0)
    {
        s_at_cmd_active = false;
        return ESP_FAIL;
    }
    (void)uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(1000));

    esp_err_t err = modem_read_response(response, response_size, timeout_ms);
    s_at_cmd_active = false;
#if defined(AT) && AT == 1
    if (err == ESP_OK)
    {
        LOG_DEBUGF("<<< AT RX:\n%s", response);
    }
    else
    {
        LOG_WARNF("<<< AT RX: (timeout/error=%d)", err);
    }
#endif
    return err;
}

static bool modem_attached(char *response, size_t response_size)
{
    if (modem_send_command("AT+CGATT?", response, response_size, 1000) != ESP_OK)
    {
        return false;
    }
    return response_has_token(response, "+CGATT: 1");
}

static bool modem_pdp_active(char *response, size_t response_size)
{
    if (modem_send_command("AT+CGACT?", response, response_size, 1000) != ESP_OK)
    {
        return false;
    }
    return response_has_token(response, "+CGACT: 1,1");
}

static esp_err_t modem_sync(void)
{
    char *response = s_modem_response;
    int64_t deadline = deadline_after_ms(MODEM_BOOT_TIMEOUT_MS);
    while (esp_timer_get_time() < deadline)
    {
        esp_err_t err = modem_send_command(MODEM_SYNC_AT_CMD, response, MODEM_RESP_BUF_SIZE, 200);
        if (err == ESP_OK && modem_response_is_ok(response))
        {
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
    if (at_ready)
    {
        (void)uart_flush_input(UART_PORT_NUM);
        static const char shutdown_cmd[] = "AT+QPOWD=1\r\n";
        int written = uart_write_bytes(UART_PORT_NUM, shutdown_cmd, sizeof(shutdown_cmd) - 1);
        if (written >= 0)
        {
            (void)uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(1000));
            if (modem_wait_for_status_level(0, MODEM_SHUTDOWN_TIMEOUT_MS) == ESP_OK)
            {
                LOG_DEBUGF("Module gracefully powered down.");
                return ESP_OK;
            }
            LOG_WARNF("Graceful shutdown timeout via STATUS pin.");
        }
    }

    if (modem_status_is_on())
    {
        LOG_WARNF("Forcing shutdown via PWRKEY...");
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
    if (result != NULL)
    {
        result->timing.boot_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    }
    if (err != ESP_OK)
    {
        if (result != NULL)
        {
            result->code = PPP_4G_DIAG_AT_NO_RESPONSE;
        }
        return err;
    }
    s_at_ready = true;
    modem_disable_echo();

    stage_start_us = esp_timer_get_time();
    bool sim_ready = false;
    int64_t cpin_deadline = deadline_after_ms(MODEM_SIM_TIMEOUT_MS);
    while (esp_timer_get_time() < cpin_deadline)
    {
        if (modem_send_command("AT+CPIN?", response, MODEM_RESP_BUF_SIZE, 1000) == ESP_OK)
        {
            if (response_has_token(response, "+CPIN: READY"))
            {
                sim_ready = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (result != NULL)
    {
        result->timing.sim_ready_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    }
    if (!sim_ready)
    {
        if (result != NULL)
        {
            result->code = PPP_4G_DIAG_SIM_NOT_READY;
        }
        return ESP_ERR_TIMEOUT;
    }
    if (result != NULL)
    {
        result->sim_ready = true;
    }

    stage_start_us = esp_timer_get_time();
    if (modem_send_command("AT+CFUN?", response, MODEM_RESP_BUF_SIZE, 1000) != ESP_OK ||
        !response_has_token(response, "+CFUN: 1"))
    {
        (void)modem_send_command("AT+CFUN=1", response, MODEM_RESP_BUF_SIZE, 2000);
    }

    bool registered = false;
    int64_t reg_deadline = deadline_after_ms(MODEM_REG_TIMEOUT_MS);
    while (esp_timer_get_time() < reg_deadline)
    {
        if (modem_send_command("AT+CEREG?", response, MODEM_RESP_BUF_SIZE, 1000) == ESP_OK)
        {
            if (modem_response_is_registered(response))
            {
                registered = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(MODEM_REG_POLL_MS));
    }
    if (result != NULL)
    {
        result->timing.network_attach_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    }
    if (!registered)
    {
        if (result != NULL)
        {
            result->code = PPP_4G_DIAG_NOT_REGISTERED;
        }
        return ESP_ERR_TIMEOUT;
    }
    if (result != NULL)
    {
        result->registered = true;
    }

    bool attached = modem_attached(response, MODEM_RESP_BUF_SIZE);
    int64_t attach_deadline = deadline_after_ms(MODEM_ATTACH_TIMEOUT_MS);
    while (!attached && esp_timer_get_time() < attach_deadline)
    {
        err = modem_send_command("AT+CGATT=1", response, MODEM_RESP_BUF_SIZE, MODEM_ATTACH_TIMEOUT_MS);
        if (err == ESP_OK && modem_response_is_ok(response))
        {
            attached = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        attached = modem_attached(response, MODEM_RESP_BUF_SIZE);
    }

    if (!attached)
    {
        if (result != NULL)
        {
            result->timing.network_attach_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
            result->code = PPP_4G_DIAG_ATTACH_FAILED;
        }
        return err != ESP_OK ? err : ESP_FAIL;
    }
    if (result != NULL)
    {
        result->attached = true;
        result->timing.network_attach_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    }

    stage_start_us = esp_timer_get_time();
    if (!modem_pdp_active(response, MODEM_RESP_BUF_SIZE))
    {
        err = modem_send_command("AT+CGACT=1,1", response, MODEM_RESP_BUF_SIZE, MODEM_PDP_TIMEOUT_MS);
        if (err != ESP_OK || !modem_response_is_ok(response))
        {
            if (result != NULL)
            {
                result->timing.pdp_active_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
                result->code = PPP_4G_DIAG_PDP_FAILED;
            }
            return err != ESP_OK ? err : ESP_FAIL;
        }
    }

    err = modem_send_command("AT+CGPADDR=1", response, MODEM_RESP_BUF_SIZE, 5000);
    if (result != NULL)
    {
        result->timing.pdp_active_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
    }
    if (err != ESP_OK || !modem_response_has_ip(response))
    {
        if (result != NULL)
        {
            result->code = PPP_4G_DIAG_NO_IP;
        }
        return err != ESP_OK ? err : ESP_FAIL;
    }
    if (result != NULL)
    {
        result->pdp_active = true;
        modem_copy_cgpaddr_ip(response, result->ip_addr, sizeof(result->ip_addr));
    }

    // LOG_DEBUGF("4G packet service is ready.");
    return ESP_OK;
}

static esp_err_t init_4g_network_internal(cb_communication_channel_established cb)
{
    esp_err_t err = ESP_OK;
    bool report_logged = false;
    ppp_4g_diag_result_t result = {
        .code = PPP_4G_DIAG_IO_ERROR,
    };
    int64_t total_start_us = esp_timer_get_time();
    int64_t stage_start_us = total_start_us;

    if (s_module_network_ready)
    {
        if (cb != NULL)
        {
            cb();
        }
        return ESP_OK;
    }

    err = modem_gpio_init();
    if (err != ESP_OK)
    {
        LOG_ERRORF("4G GPIO init failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    err = modem_uart_init();
    if (err != ESP_OK)
    {
        LOG_ERRORF("4G UART init failed: %s", esp_err_to_name(err));
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
        LOG_ERRORF("4G power on failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    err = modem_pulse_low_active_line(MODEM_PWRKEY_PIN, MODEM_PULSE_PWRKEY_MS);
    if (err != ESP_OK)
    {
        result.code = PPP_4G_DIAG_POWER_ON_FAILED;
        LOG_ERRORF("4G PWRKEY pulse failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    err = modem_prepare_packet_service(&result);
    if (err != ESP_OK)
    {
        LOG_ERRORF("4G packet service prepare failed: %s", esp_err_to_name(err));
        goto cleanup;
    }
    s_module_network_ready = true;

    result.code = PPP_4G_DIAG_OK;

    result.timing.total_ms = (uint32_t)((esp_timer_get_time() - total_start_us) / 1000LL);
    ppp_4g_log_result(&result);
    report_logged = true;

    if (cb != NULL)
    {
        cb();
    }
    return ESP_OK;

cleanup:
    if (!report_logged)
    {
        result.timing.total_ms = (uint32_t)((esp_timer_get_time() - total_start_us) / 1000LL);
        ppp_4g_log_result(&result);
    }
    s_module_network_ready = false;
    (void)modem_shutdown_gracefully(s_at_ready);
    (void)modem_power_disable();
    modem_uart_deinit();
    s_at_ready = false;
    return err;
}

esp_err_t init_4g_network(cb_communication_channel_established cb)
{
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);
    esp_err_t err = init_4g_network_internal(cb);
    xSemaphoreGive(s_at_mutex);
    return err;
}

static esp_err_t shutdown_4g_network_internal(void)
{
    (void)modem_shutdown_gracefully(s_at_ready);
    (void)modem_power_disable();
    modem_uart_deinit();

    // 重置相关引脚为默认高阻态，防止深度睡眠期间对已断电的 4G 模块产生电流倒灌泄漏
    gpio_reset_pin(MODEM_UART_TX_PIN);
    gpio_reset_pin(MODEM_UART_RX_PIN);
    gpio_reset_pin(MODEM_PWRKEY_PIN);
    gpio_reset_pin(MODEM_STATUS_PIN);
    gpio_reset_pin(MODEM_NET_STATUS_PIN);

    // 锁定电源使能引脚，使其在进入深度睡眠后依然强制输出断电状态（低电平），
    // 防止悬空导致外部电源开关管(LDO/MOSFET)微导通，彻底消除随机漏电。
    gpio_hold_en(MODEM_PWR_EN_PIN);

    // 开启全局深睡 GPIO 保持，确保上面的 hold_en 在 Deep Sleep 期间依然生效
    gpio_deep_sleep_hold_en();

    s_at_ready = false;
    s_module_network_ready = false;
    return ESP_OK;
}

esp_err_t shutdown_4g_network(void)
{
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);
    esp_err_t err = shutdown_4g_network_internal();
    xSemaphoreGive(s_at_mutex);
    return err;
}

// 流式安全读取直到 CONNECT\r\n（防止把固件二进制头部误吃掉）
static esp_err_t wait_for_connect_stream_safe(uint32_t timeout_ms)
{
    const char *target = "CONNECT\r\n";
    int match_idx = 0;
    int target_len = strlen(target);
    int64_t deadline = esp_timer_get_time() + timeout_ms * 1000LL;

    while (esp_timer_get_time() < deadline)
    {
        uint8_t c;
        int len = uart_read_bytes(UART_PORT_NUM, &c, 1, pdMS_TO_TICKS(10));
        if (len > 0)
        {
            if (c == target[match_idx])
            {
                match_idx++;
                if (match_idx == target_len)
                    return ESP_OK;
            }
            else
            {
                if (c == target[0])
                    match_idx = 1;
                else
                    match_idx = 0;
            }
        }
    }
    return ESP_ERR_TIMEOUT;
}

static void append_uart_response_until_line_end(char *response, size_t response_size, uint32_t timeout_ms)
{
    if (response == NULL || response_size == 0)
        return;

    int64_t deadline = deadline_after_ms(timeout_ms);
    while (esp_timer_get_time() < deadline)
    {
        size_t used = strlen(response);
        if (used > 0 && response[used - 1] == '\n')
            return;
        if (used >= response_size - 1)
            return;

        int read_len = uart_read_bytes(UART_PORT_NUM,
                                       response + used,
                                       response_size - used - 1,
                                       pdMS_TO_TICKS(20));
        if (read_len > 0)
        {
            response[used + read_len] = '\0';
        }
    }
}

static esp_err_t bsp_4g_http_get_internal(const char *url, char **out_response)
{
    if (!url || !out_response)
        return ESP_ERR_INVALID_ARG;
    *out_response = NULL;
    if (!s_at_ready)
        return ESP_ERR_INVALID_STATE;

    char cmd[128];
    char *response = calloc(1, MODEM_HTTP_RESP_BUF_SIZE);
    if (!response)
        return ESP_ERR_NO_MEM;
    int url_len = strlen(url);
    esp_err_t err = ESP_OK;

    err = modem_send_command("AT+QHTTPCFG=\"requestheader\",0", response, MODEM_HTTP_RESP_BUF_SIZE, 2000);
    if (err != ESP_OK || !modem_response_is_ok(response))
    {
        err = err != ESP_OK ? err : ESP_FAIL;
        goto cleanup;
    }

    err = modem_send_command("AT+QHTTPCFG=\"responseheader\",0", response, MODEM_HTTP_RESP_BUF_SIZE, 2000);
    if (err != ESP_OK || !modem_response_is_ok(response))
    {
        err = err != ESP_OK ? err : ESP_FAIL;
        goto cleanup;
    }

    // 互斥锁定：HTTP 事务期间独占串口数据
    s_at_cmd_active = true;

    // 1. 设置 URL 长度
    snprintf(cmd, sizeof(cmd), "AT+QHTTPURL=%d,80\r\n", url_len);
    uart_flush_input(UART_PORT_NUM);
    uart_write_bytes(UART_PORT_NUM, cmd, strlen(cmd));
    err = modem_read_until_pattern(response, MODEM_HTTP_RESP_BUF_SIZE, "CONNECT", 5000);
    if (err != ESP_OK)
    {
        goto cleanup;
    }

    // 2. 发送实际 URL
    uart_write_bytes(UART_PORT_NUM, url, url_len);
    err = modem_read_response(response, MODEM_HTTP_RESP_BUF_SIZE, 5000);
    if (err != ESP_OK || !modem_response_is_ok(response))
    {
        err = err != ESP_OK ? err : ESP_FAIL;
        goto cleanup;
    }

    // 3. 触发模块发起底层 HTTP GET 请求
    uart_write_bytes(UART_PORT_NUM, "AT+QHTTPGET=80\r\n", 16);
    // 请求可能耗时很长，给 40 秒超时
    err = modem_read_until_pattern(response, MODEM_HTTP_RESP_BUF_SIZE, "+QHTTPGET:", 40000);
    if (err != ESP_OK)
    {
        goto cleanup;
    }
    append_uart_response_until_line_end(response, MODEM_HTTP_RESP_BUF_SIZE, 1000);

    // 解析 QHTTPGET: err, status, len
    int qerr = -1, qstatus = -1, qlen = 0;
    const char *httpget_line = strstr(response, "+QHTTPGET:");
    if (httpget_line && sscanf(httpget_line, "+QHTTPGET: %d,%d,%d", &qerr, &qstatus, &qlen) == 3)
    {
        if (qerr == 0 && qlen > 0)
        {
            // 4. 从模块内部提取 JSON 数据 (即便不是 200 也要读出来看看报错详情)
            uart_write_bytes(UART_PORT_NUM, "AT+QHTTPREAD=80\r\n", 17);
            err = wait_for_connect_stream_safe(5000);
            if (err == ESP_OK)
            {
                char *body = calloc(1, qlen + 1);
                int received = 0;
                int64_t start_us = esp_timer_get_time();
                while (body && received < qlen)
                {
                    int r = uart_read_bytes(UART_PORT_NUM, body + received, qlen - received, pdMS_TO_TICKS(100));
                    if (r > 0)
                        received += r;
                    if ((esp_timer_get_time() - start_us) > 15000000)
                        break; // 防止死等，15秒超时
                }
                if (received == qlen)
                {
                    *out_response = body;
                    // 读取完毕后模块通常还会吐出 OK 和 +QHTTPREAD:0，这里主动读取清空
                    modem_read_response(response, MODEM_HTTP_RESP_BUF_SIZE, 3000);
                    err = ESP_OK;
                }
                else
                {
                    free(body);
                    err = ESP_FAIL;
                }

                if (qstatus != 200 && received == qlen && body)
                {
                    LOG_WARNF("HTTP GET returned %d. Response body: %s", qstatus, body);
                    if (*out_response != body) {
                        free(body);
                    }
                    err = ESP_FAIL;
                }
            }
        }
        else if (qerr == 0 && qstatus == 200)
        {
            // content_len == 0
            err = ESP_OK;
        }
        else
        {
            LOG_WARNF("HTTP GET failed: AT_err=%d, HTTP_status=%d, content_len=%d", qerr, qstatus, qlen);
            err = ESP_FAIL;
        }
    }
    else
    {
        LOG_WARNF("Unexpected QHTTPGET response: %s", response);
        err = ESP_FAIL;
    }

cleanup:
    free(response);
    s_at_cmd_active = false;
    return err;
}

static esp_err_t bsp_4g_ota_download_and_write_internal(const char *url, int fw_size, const char *access_key, esp_ota_handle_t update_handle)
{
    if (!url || fw_size <= 0)
        return ESP_ERR_INVALID_ARG;
    if (!s_at_ready)
        return ESP_ERR_INVALID_STATE;

    // 解析出 host 和 path 用于手动构造 Header
    const char *proto_end = strstr(url, "://");
    const char *host_start = proto_end ? proto_end + 3 : url;
    const char *path_start = strchr(host_start, '/');
    char host[128] = {0};
    char path[256] = {0};

    if (path_start)
    {
        int host_len = path_start - host_start;
        if (host_len > 127)
            host_len = 127;
        strncpy(host, host_start, host_len);
        strncpy(path, path_start, sizeof(path) - 1);
    }
    else
    {
        strncpy(host, host_start, sizeof(host) - 1);
        strcpy(path, "/");
    }

    s_at_cmd_active = true;
    char cmd[256];
    char response[MODEM_RESP_BUF_SIZE];
    esp_err_t err = ESP_OK;

    // 1. 临时开启 AT+QHTTP 自定义 Header 能力
    modem_send_command("AT+QHTTPCFG=\"requestheader\",1", response, sizeof(response), 2000);

    char *ota_buf = malloc(4096);
    if (!ota_buf)
    {
        s_at_cmd_active = false;
        return ESP_ERR_NO_MEM;
    }

    int offset = 0;
    int chunk_size = 4096;
    int retry_count = 0;

    LOG_DEBUGF("Starting 4G Chunked OTA from MinIO: %s", host);

    while (offset < fw_size)
    {
        int end = offset + chunk_size - 1;
        if (end >= fw_size)
            end = fw_size - 1;
        int expect_len = end - offset + 1;

        // 设置目标 URL
        snprintf(cmd, sizeof(cmd), "AT+QHTTPURL=%d,80\r\n", (int)strlen(url));
        uart_flush_input(UART_PORT_NUM);
        uart_write_bytes(UART_PORT_NUM, cmd, strlen(cmd));
        err = modem_read_until_pattern(response, sizeof(response), "CONNECT", 5000);
        if (err != ESP_OK)
        {
            if (++retry_count > 3)
                break;
            continue;
        }

        uart_write_bytes(UART_PORT_NUM, url, strlen(url));
        err = modem_read_response(response, sizeof(response), 5000);
        if (err != ESP_OK || !modem_response_is_ok(response))
        {
            if (++retry_count > 3)
                break;
            continue;
        }

        // 处理鉴权 Header
        char auth_header[128] = {0};
        if (access_key && access_key[0] != '\0')
        {
            snprintf(auth_header, sizeof(auth_header), "Authorization: %s\r\n", access_key);
        }

        // 构造含有 Range 的 HTTP GET 请求头
        char req_header[512];
        int req_len = snprintf(req_header, sizeof(req_header),
                               "GET %s HTTP/1.1\r\n"
                               "Host: %s\r\n"
                               "%s"
                               "Range: bytes=%d-%d\r\n"
                               "Connection: keep-alive\r\n\r\n",
                               path, host, auth_header, offset, end);

        snprintf(cmd, sizeof(cmd), "AT+QHTTPGET=80,%d\r\n", req_len);
        uart_write_bytes(UART_PORT_NUM, cmd, strlen(cmd));
        err = modem_read_until_pattern(response, sizeof(response), "CONNECT", 5000);
        if (err != ESP_OK)
        {
            if (++retry_count > 3)
                break;
            continue;
        }

        uart_write_bytes(UART_PORT_NUM, req_header, req_len);

        // 等待 MinIO 响应 206 Partial Content (或者200)
        err = modem_read_until_pattern(response, sizeof(response), "+QHTTPGET:", 20000);
        if (err != ESP_OK)
        {
            if (++retry_count > 3)
                break;
            continue;
        }

        int64_t qdeadline = esp_timer_get_time() + 1000000;
        while (strchr(response, '\n') == NULL && esp_timer_get_time() < qdeadline)
        {
            int rlen = strlen(response);
            int r = uart_read_bytes(UART_PORT_NUM, response + rlen, MODEM_RESP_BUF_SIZE - rlen - 1, pdMS_TO_TICKS(10));
            if (r > 0)
                response[rlen + r] = '\0';
        }

        int qerr = -1, qstatus = -1, qlen = 0;
        char *line = strstr(response, "+QHTTPGET:");
        if (line && sscanf(line, "+QHTTPGET: %d,%d,%d", &qerr, &qstatus, &qlen) >= 2)
        {
            if (qerr != 0 || (qstatus != 206 && qstatus != 200))
            {
                LOG_ERRORF("MinIO range req rejected: err=%d, status=%d", qerr, qstatus);
                err = ESP_FAIL;
                if (++retry_count > 3)
                    break;
                continue;
            }
        }

        // 准备读取本块二进制数据
        uart_write_bytes(UART_PORT_NUM, "AT+QHTTPREAD=80\r\n", 17);
        err = wait_for_connect_stream_safe(5000);
        if (err == ESP_OK)
        {
            int received = 0;
            int64_t start_us = esp_timer_get_time();
            while (received < expect_len)
            {
                int r = uart_read_bytes(UART_PORT_NUM, ota_buf + received, expect_len - received, pdMS_TO_TICKS(100));
                if (r > 0)
                    received += r;
                if ((esp_timer_get_time() - start_us) > 15000000)
                {
                    err = ESP_ERR_TIMEOUT;
                    break;
                }
            }
            if (received == expect_len)
            {
                // ★ 极其关键的一步：边读边写进入 Flash
                if (esp_ota_write(update_handle, ota_buf, expect_len) != ESP_OK)
                {
                    LOG_ERROR("OTA Write to Flash failed");
                    err = ESP_FAIL;
                    break;
                }
                offset += expect_len;
                retry_count = 0;
                LOG_DEBUGF("OTA Progress: %d / %d bytes (%.1f%%)", offset, fw_size, (float)offset * 100.0 / fw_size);

                // 清除剩余的 OK 回复
                modem_read_response(response, sizeof(response), 2000);
            }
            else
            {
                err = ESP_FAIL;
                if (++retry_count > 3)
                    break;
            }
        }
        else
        {
            if (++retry_count > 3)
                break;
        }
    }

    free(ota_buf);
    // 恢复标准 Header 设置，以免影响后续的其他普通网络请求
    modem_send_command("AT+QHTTPCFG=\"requestheader\",0", response, sizeof(response), 2000);
    s_at_cmd_active = false;

    return (offset >= fw_size) ? ESP_OK : ESP_FAIL;
}

static esp_err_t bsp_4g_http_write_json_internal(const char *method,
                                                 const char *url,
                                                 const char *payload,
                                                 char **out_response)
{
    if (!method || !url || !payload)
        return ESP_ERR_INVALID_ARG;
    if (out_response)
        *out_response = NULL;
    if (!s_at_ready)
        return ESP_ERR_INVALID_STATE;

    const char *proto_end = strstr(url, "://");
    const char *host_start = proto_end ? proto_end + 3 : url;
    const char *path_start = strchr(host_start, '/');
    char host[128] = {0};
    char path[256] = {0};

    if (path_start)
    {
        int host_len = path_start - host_start;
        if (host_len > 127)
            host_len = 127;
        strncpy(host, host_start, host_len);
        strncpy(path, path_start, sizeof(path) - 1);
    }
    else
    {
        strncpy(host, host_start, sizeof(host) - 1);
        strcpy(path, "/");
    }

    s_at_cmd_active = true;
    char cmd[64];
    char *response = calloc(1, MODEM_RESP_BUF_SIZE);
    char *req_header = malloc(512);
    esp_err_t err = ESP_OK;
    if (!response || !req_header)
    {
        free(response);
        free(req_header);
        s_at_cmd_active = false;
        return ESP_ERR_NO_MEM;
    }

    err = modem_send_command("AT+QHTTPCFG=\"requestheader\",1", response, MODEM_RESP_BUF_SIZE, 2000);
    if (err != ESP_OK || !modem_response_is_ok(response))
    {
        LOG_WARNF("HTTP %s requestheader config failed: %s", method, response[0] ? response : esp_err_to_name(err));
        err = err != ESP_OK ? err : ESP_FAIL;
        goto cleanup;
    }
    s_at_cmd_active = true;

    snprintf(cmd, sizeof(cmd), "AT+QHTTPURL=%d,80\r\n", (int)strlen(url));
    uart_flush_input(UART_PORT_NUM);
    uart_write_bytes(UART_PORT_NUM, cmd, strlen(cmd));
    (void)uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(1000));
    if (modem_read_until_pattern(response, MODEM_RESP_BUF_SIZE, "CONNECT", 5000) == ESP_OK)
    {
        uart_write_bytes(UART_PORT_NUM, url, strlen(url));
        (void)uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(1000));
        err = modem_read_response(response, MODEM_RESP_BUF_SIZE, 5000);
        if (err != ESP_OK || !modem_response_is_ok(response))
        {
            LOG_WARNF("HTTP %s URL set failed: %s", method, response[0] ? response : esp_err_to_name(err));
            err = err != ESP_OK ? err : ESP_FAIL;
            goto cleanup;
        }
    }
    else
    {
        LOG_WARNF("HTTP %s URL prompt failed: %s", method, response[0] ? response : "(none)");
        err = ESP_FAIL;
        goto cleanup;
    }

    int payload_len = strlen(payload);
    int req_len = snprintf(req_header, 512,
                           "%s %s HTTP/1.1\r\n"
                           "Host: %s\r\n"
                           "User-Agent: Sentinel/1.0\r\n"
                           "Accept: */*\r\n"
                           "Content-Type: application/json\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n\r\n",
                           method, path, host, payload_len);
    if (req_len < 0 || req_len >= 512)
    {
        err = ESP_ERR_INVALID_SIZE;
        goto cleanup;
    }

    int total_len = req_len + payload_len;
    snprintf(cmd, sizeof(cmd), "AT+QHTTPPOST=%d,80,80\r\n", total_len);
    uart_write_bytes(UART_PORT_NUM, cmd, strlen(cmd));
    (void)uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(1000));

    if (modem_read_until_pattern(response, MODEM_RESP_BUF_SIZE, "CONNECT", MODEM_HTTP_POST_CONNECT_TIMEOUT_MS) == ESP_OK)
    {
        uart_write_bytes(UART_PORT_NUM, req_header, req_len);
        uart_write_bytes(UART_PORT_NUM, payload, payload_len);
        (void)uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(5000));

        err = modem_read_until_pattern(response, MODEM_RESP_BUF_SIZE, "+QHTTPPOST:", 15000);
        if (err == ESP_OK)
        {
            int64_t qdeadline = esp_timer_get_time() + 1000000;
            while (strchr(response, '\n') == NULL && esp_timer_get_time() < qdeadline)
            {
                int rlen = strlen(response);
                if (rlen >= MODEM_RESP_BUF_SIZE - 1)
                    break;
                int r = uart_read_bytes(UART_PORT_NUM,
                                        response + rlen,
                                        MODEM_RESP_BUF_SIZE - rlen - 1,
                                        pdMS_TO_TICKS(10));
                if (r > 0)
                    response[rlen + r] = '\0';
            }

            int qerr = -1, qstatus = -1, qlen = 0;
            char *line = strstr(response, "+QHTTPPOST:");
            if (line && sscanf(line, "+QHTTPPOST: %d,%d,%d", &qerr, &qstatus, &qlen) >= 2)
            {
                if (qerr == 0 && qlen > 0)
                {
                    err = ESP_OK;
                    if (out_response || qstatus != 200)
                    {
                        uart_write_bytes(UART_PORT_NUM, "AT+QHTTPREAD=80\r\n", 17);
                        err = wait_for_connect_stream_safe(5000);
                        if (err == ESP_OK)
                        {
                            char *body = calloc(1, qlen + 1);
                            int received = 0;
                            int64_t start_us = esp_timer_get_time();
                            while (body && received < qlen)
                            {
                                int r = uart_read_bytes(UART_PORT_NUM,
                                                        body + received,
                                                        qlen - received,
                                                        pdMS_TO_TICKS(100));
                                if (r > 0)
                                    received += r;
                                if ((esp_timer_get_time() - start_us) > 15000000)
                                    break;
                            }
                            if (body && received == qlen)
                            {
                                err = ESP_OK;
                            }
                            else
                            {
                                free(body);
                                body = NULL;
                                err = ESP_FAIL;
                            }

                            if (body)
                            {
                                if (qstatus != 200 && qstatus != 201)
                                {
                                    LOG_WARNF("HTTP POST returned %d. Response body: %s", qstatus, body);
                                    if (out_response) *out_response = body;
                                    else free(body);
                                    err = ESP_FAIL;
                                }
                                else
                                {
                                    if (out_response) *out_response = body;
                                    else free(body);
                                    err = ESP_OK;
                                }
                            }
                            modem_read_response(response, MODEM_RESP_BUF_SIZE, 3000);
                        }
                        else
                        {
                            err = ESP_FAIL;
                        }
                    }
                }
                else if (qerr == 0 && (qstatus == 200 || qstatus == 201))
                {
                    err = ESP_OK;
                }
                else
                {
                    LOG_WARNF("HTTP %s failed: AT_err=%d, HTTP_status=%d, content_len=%d",
                              method,
                              qerr,
                              qstatus,
                              qlen);
                    err = ESP_FAIL;
                }
            }
            else
            {
                LOG_WARNF("Unexpected QHTTPPOST response: %s", response);
                err = ESP_FAIL;
            }
        }
        else
        {
            LOG_WARNF("HTTP %s result wait failed: %s", method, esp_err_to_name(err));
        }
    }
    else
    {
        LOG_WARNF("HTTP %s payload prompt failed: %s", method, response[0] ? response : "(none)");
        err = ESP_FAIL;
    }

cleanup:
    modem_send_command("AT+QHTTPCFG=\"requestheader\",0", response, MODEM_RESP_BUF_SIZE, 2000);
    free(req_header);
    free(response);
    s_at_cmd_active = false;
    return err;
}

static esp_err_t bsp_4g_http_post_json_internal(const char *url, const char *payload, char **out_response)
{
    return bsp_4g_http_write_json_internal("POST", url, payload, out_response);
}

static esp_err_t bsp_4g_http_put_internal(const char *url, const char *payload)
{
    return bsp_4g_http_write_json_internal("PUT", url, payload, NULL);
}

esp_err_t bsp_4g_ota_download_and_write(const char *url, int fw_size, const char *access_key, esp_ota_handle_t update_handle)
{
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);
    esp_err_t err = bsp_4g_ota_download_and_write_internal(url, fw_size, access_key, update_handle);
    xSemaphoreGive(s_at_mutex);
    return err;
}

esp_err_t bsp_4g_http_put(const char *url, const char *payload)
{
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);
    esp_err_t err = init_4g_network_internal(NULL);
    if (err == ESP_OK)
    {
        err = bsp_4g_http_put_internal(url, payload);
    }
    xSemaphoreGive(s_at_mutex);
    return err;
}

esp_err_t bsp_4g_http_post_json(const char *url, const char *payload, char **out_response)
{
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);
    esp_err_t err = init_4g_network_internal(NULL);
    if (err == ESP_OK)
    {
        err = bsp_4g_http_post_json_internal(url, payload, out_response);
    }
    xSemaphoreGive(s_at_mutex);
    return err;
}

esp_err_t bsp_4g_http_get(const char *url, char **out_response)
{
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);
    esp_err_t err = init_4g_network_internal(NULL);
    if (err == ESP_OK)
    {
        err = bsp_4g_http_get_internal(url, out_response);
    }
    xSemaphoreGive(s_at_mutex);
    return err;
}

esp_err_t bsp_4g_get_rssi(int *out_rssi)
{
    if (!out_rssi) return ESP_ERR_INVALID_ARG;
    
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);
    
    esp_err_t err = init_4g_network_internal(NULL);
    if (err == ESP_OK)
    {
        char *response = malloc(MODEM_RESP_BUF_SIZE);
        if (response)
        {
            err = modem_send_command("AT+CSQ", response, MODEM_RESP_BUF_SIZE, 1000);
            if (err == ESP_OK)
            {
                int rssi = 0, ber = 0;
                const char *p = strstr(response, "+CSQ:");
                if (p && sscanf(p, "+CSQ: %d,%d", &rssi, &ber) == 2)
                {
                    *out_rssi = rssi;
                }
                else
                {
                    err = ESP_FAIL;
                }
            }
            free(response);
        }
        else
        {
            err = ESP_ERR_NO_MEM;
        }
    }
    
    xSemaphoreGive(s_at_mutex);
    return err;
}
