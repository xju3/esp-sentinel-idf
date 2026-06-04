#include "bsp_4g.h"
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
#include <stdlib.h>
#include <sys/param.h>
#include <stdbool.h>
#include "esp_ota_ops.h"
#include <time.h>
#include <sys/time.h>

#ifndef SN
#define SN "0"
#endif

// ============== Board pin aliases ==============
#define MODEM_UART_RX_PIN BOARD_GPIO_4G_UART_RX
#define MODEM_UART_TX_PIN BOARD_GPIO_4G_UART_TX
#define MODEM_PWR_EN_PIN BOARD_GPIO_4G_PWR
#define MODEM_PWRKEY_PIN BOARD_GPIO_4G_PWRKEY
#define MODEM_STATUS_PIN BOARD_GPIO_4G_STATUS
#define MODEM_NET_STATUS_PIN BOARD_GPIO_4G_NET_STATUS
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
#define MODEM_HTTP_RESP_BUF_SIZE 4096

static const char *TAG = "ppp_4g";

// UART & AT state
static bool s_uart_driver_installed = false;
static bool s_at_ready = false;
static bool s_module_mqtt_connected = false;
static char s_modem_response[MODEM_RESP_BUF_SIZE];
static volatile bool s_at_cmd_active = false;
static TaskHandle_t s_urc_task_handle = NULL;
static SemaphoreHandle_t s_at_mutex = NULL;

static void ensure_at_mutex(void)
{
    if (s_at_mutex == NULL)
    {
        s_at_mutex = xSemaphoreCreateMutex();
    }
}

// URC Callback
typedef void (*bsp_4g_urc_cb_t)(int event_type, const char *topic, const char *payload, size_t len);
static bsp_4g_urc_cb_t s_urc_cb = NULL;

void bsp_4g_set_urc_cb(bsp_4g_urc_cb_t cb)
{
    s_urc_cb = cb;
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
    PPP_4G_DIAG_MQTT_OPEN_FAILED,
    PPP_4G_DIAG_MQTT_CONNECT_FAILED,
    PPP_4G_DIAG_MQTT_PUBLISH_FAILED,
    PPP_4G_DIAG_IO_ERROR,
} ppp_4g_diag_code_t;

typedef struct
{
    uint32_t power_on_ms;
    uint32_t boot_ms;
    uint32_t sim_ready_ms;
    uint32_t network_attach_ms;
    uint32_t pdp_active_ms;
    uint32_t mqtt_ms;
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
    bool mqtt_opened;
    bool mqtt_connected;
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
    case PPP_4G_DIAG_MQTT_OPEN_FAILED:
        return "mqtt_open_failed";
    case PPP_4G_DIAG_MQTT_CONNECT_FAILED:
        return "mqtt_connect_failed";
    case PPP_4G_DIAG_MQTT_PUBLISH_FAILED:
        return "mqtt_publish_failed";
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
    ESP_LOGI(TAG, "---------------------------------------");
    ESP_LOGI(TAG, "Total Time      : %lu ms", (unsigned long)result->timing.total_ms);
    ESP_LOGI(TAG, "---------------------------------------");
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

static const char *modem_mqtt_host(void)
{
    const char *host = g_user_config.host;
    if (host == NULL || host[0] == '\0')
    {
        return BOARD_4G_MQTT_HOST;
    }

    const char *scheme = strstr(host, "://");
    return scheme != NULL ? scheme + 3 : host;
}

static void modem_copy_mqtt_host(char *out, size_t out_size)
{
    if (out == NULL || out_size == 0)
    {
        return;
    }

    const char *host = modem_mqtt_host();
    size_t len = 0;
    while (host[len] != '\0' && host[len] != ':' && host[len] != '/' && len + 1 < out_size)
    {
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
    if (line != NULL && sscanf(line, "+QMTOPEN: %d,%d", &connect_id, &result) == 2)
    {
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
    if (line != NULL && sscanf(line, "+QMTCONN: %d,%d,%d", &connect_id, &result, &retcode) == 3)
    {
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
    if (line != NULL && sscanf(line, "+QMTPUBEX: %d,%d,%d", &connect_id, &msg_id, &result) == 3)
    {
        return result;
    }
    line = response != NULL ? strstr(response, "+QMTPUB:") : NULL;
    if (line != NULL && sscanf(line, "+QMTPUB: %d,%d,%d", &connect_id, &msg_id, &result) == 3)
    {
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

// ================= 新增：通过 4G 基站或内部 NTP 获取时间并同步给 ESP32 =================
esp_err_t bsp_4g_sync_time(void)
{
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);

    char response[MODEM_RESP_BUF_SIZE];
    esp_err_t err = ESP_FAIL;
    s_at_cmd_active = true;

    // 发送 AT+CCLK? 查询模块当前时间 (格式: +CCLK: "24/06/15,12:30:45+32")
    uart_flush_input(UART_PORT_NUM);
    uart_write_bytes(UART_PORT_NUM, "AT+CCLK?\r\n", 10);
    err = modem_read_response(response, sizeof(response), 2000);

    if (err == ESP_OK)
    {
        int year, month, day, hour, min, sec;
        char *line = strstr(response, "+CCLK: \"");
        // 解析时间，忽略末尾的时区标识
        if (line && sscanf(line, "+CCLK: \"%d/%d/%d,%d:%d:%d", &year, &month, &day, &hour, &min, &sec) >= 6)
        {
            if (year >= 24)
            { // 确保时间大于 2024 年，排除模块自身的默认初始时间 1980/2004 等
                struct tm tm_time = {0};
                tm_time.tm_year = year + 100; // AT 返回 24，代表 2024。tm_year 是从 1900 算起，所以 +100
                tm_time.tm_mon = month - 1;   // 月份 0-11
                tm_time.tm_mday = day;
                tm_time.tm_hour = hour;
                tm_time.tm_min = min;
                tm_time.tm_sec = sec;

                time_t t = mktime(&tm_time);

                struct timeval tv = {.tv_sec = t, .tv_usec = 0};
                settimeofday(&tv, NULL); // 强制修改 ESP32 的硬件 RTC 系统时间

                ESP_LOGI(TAG, "Time synced from 4G Base Station: 20%02d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, min, sec);
                err = ESP_OK;
            }
            else
            {
                ESP_LOGW(TAG, "4G time not updated yet (Year 20%02d). Retry needed.", year);
                err = ESP_FAIL;
            }
        }
    }

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
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
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

static void modem_handle_urc(const char *response)
{
    if (!s_urc_cb || !response)
        return;

    const char *recv = strstr(response, "+QMTRECV:");
    if (recv)
    {
        char topic[128] = {0};
        int client_id, msg_id;
        // 解析格式: +QMTRECV: 0,0,"topic","payload"
        if (sscanf(recv, "+QMTRECV: %d,%d,\"%127[^\"]\"", &client_id, &msg_id, topic) == 3)
        {
            const char *q1 = strchr(recv + 9, '"');
            if (q1)
            {
                const char *q2 = strchr(q1 + 1, '"');
                if (q2 && *(q2 + 1) == ',')
                {
                    const char *payload = q2 + 2;
                    size_t plen = strlen(payload);
                    // 剔除可能多余的换行符
                    while (plen > 0 && (payload[plen - 1] == '\r' || payload[plen - 1] == '\n'))
                        plen--;
                    s_urc_cb(0, topic, payload, plen);
                }
            }
        }
    }
    else if (strstr(response, "+QMTSTAT:"))
    {
        // MQTT 断开连接事件
        int client_id, err_code;
        if (sscanf(strstr(response, "+QMTSTAT:"), "+QMTSTAT: %d,%d", &client_id, &err_code) == 2)
        {
            s_urc_cb(1, NULL, NULL, err_code);
        }
    }
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

            modem_handle_urc(response); // 拦截 URC

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

        modem_handle_urc(response); // 拦截 URC

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
    ESP_LOGI(TAG, ">>> UART TX AT frame=\"%s\" cmd=\"%s\" len=%d", tx_visible, cmd, tx_frame_len);
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
        ESP_LOGI(TAG, "<<< AT RX:\n%s", response);
    }
    else
    {
        ESP_LOGW(TAG, "<<< AT RX: (timeout/error=%d)", err);
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
                ESP_LOGI(TAG, "Module gracefully powered down.");
                return ESP_OK;
            }
            ESP_LOGW(TAG, "Graceful shutdown timeout via STATUS pin.");
        }
    }

    if (modem_status_is_on())
    {
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

    if (!modem_attached(response, MODEM_RESP_BUF_SIZE))
    {
        err = modem_send_command("AT+CGATT=1", response, MODEM_RESP_BUF_SIZE, MODEM_ATTACH_TIMEOUT_MS);
        if (err != ESP_OK || !modem_response_is_ok(response))
        {
            if (result != NULL)
            {
                result->timing.network_attach_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
                result->code = PPP_4G_DIAG_ATTACH_FAILED;
            }
            return err != ESP_OK ? err : ESP_FAIL;
        }
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

    ESP_LOGI(TAG, "4G packet service is ready.");
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
    if (len < 0 || (size_t)len >= sizeof(cmd))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t err = modem_send_command(cmd, response, response_size, 10000);
    if (err != ESP_OK || !modem_response_is_ok(response))
    {
        return err != ESP_OK ? err : ESP_FAIL;
    }
    if (response_has_token(response, "+QMTOPEN:"))
    {
        return ESP_OK;
    }
    return modem_read_until_pattern(response, response_size, "+QMTOPEN:", MODEM_MQTT_OPEN_TIMEOUT_MS);
}

static esp_err_t modem_mqtt_connect(char *response, size_t response_size)
{
    char cmd[160];
    int len = snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\"", SN);
    if (len < 0 || (size_t)len >= sizeof(cmd))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t err = modem_send_command(cmd, response, response_size, 10000);
    if (err != ESP_OK || !modem_response_is_ok(response))
    {
        return err != ESP_OK ? err : ESP_FAIL;
    }
    if (response_has_token(response, "+QMTCONN:"))
    {
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
    if (result != NULL)
    {
        result->timing.mqtt_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
        result->mqtt_open_result = modem_parse_qmtopen_result(response);
    }
    if (err != ESP_OK || modem_parse_qmtopen_result(response) != 0)
    {
        if (result != NULL)
        {
            result->code = PPP_4G_DIAG_MQTT_OPEN_FAILED;
        }
        return err != ESP_OK ? err : ESP_FAIL;
    }
    if (result != NULL)
    {
        result->mqtt_opened = true;
    }

    err = modem_mqtt_connect(response, MODEM_RESP_BUF_SIZE);
    if (result != NULL)
    {
        result->timing.mqtt_ms = (uint32_t)((esp_timer_get_time() - stage_start_us) / 1000LL);
        result->mqtt_conn_retcode = modem_parse_qmtconn_retcode(response);
    }
    if (err != ESP_OK || modem_parse_qmtconn_retcode(response) != 0)
    {
        if (result != NULL)
        {
            result->code = PPP_4G_DIAG_MQTT_CONNECT_FAILED;
        }
        return err != ESP_OK ? err : ESP_FAIL;
    }

    s_module_mqtt_connected = true;
    if (result != NULL)
    {
        result->mqtt_connected = true;
        result->code = PPP_4G_DIAG_OK;
    }
    return ESP_OK;
}

static esp_err_t modem_mqtt_publish_binary(const char *topic, const uint8_t *data, size_t len)
{
    if (topic == NULL || data == NULL || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_module_mqtt_connected)
    {
        return ESP_ERR_INVALID_STATE;
    }

    char *response = s_modem_response;
    char cmd[192];
    int cmd_len = snprintf(cmd,
                           sizeof(cmd),
                           "AT+QMTPUBEX=0,1,1,0,\"%s\",%u",
                           topic,
                           (unsigned)len);
    if (cmd_len < 0 || (size_t)cmd_len >= sizeof(cmd))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    s_at_cmd_active = true;

    (void)uart_flush_input(UART_PORT_NUM);
    int written = uart_write_bytes(UART_PORT_NUM, cmd, (size_t)cmd_len);
    if (written < 0)
    {
        return ESP_FAIL;
    }
    written = uart_write_bytes(UART_PORT_NUM, "\r\n", 2);
    if (written < 0)
    {
        return ESP_FAIL;
    }
    (void)uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(1000));

    esp_err_t err = modem_read_until_pattern(response, MODEM_RESP_BUF_SIZE, ">", 5000);
    if (err != ESP_OK)
    {
        s_at_cmd_active = false;
        ESP_LOGE(TAG, "QMTPUBEX prompt failed: %s", response[0] != '\0' ? response : "(none)");
        return err;
    }

    written = uart_write_bytes(UART_PORT_NUM, data, len);
    if (written < 0 || (size_t)written != len)
    {
        return ESP_FAIL;
    }
    static const uint8_t end_marker = 0x1A;
    written = uart_write_bytes(UART_PORT_NUM, &end_marker, sizeof(end_marker));
    if (written < 0)
    {
        return ESP_FAIL;
    }
    (void)uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(5000));

    err = modem_read_until_pattern(response, MODEM_RESP_BUF_SIZE, "+QMTPUB", MODEM_MQTT_PUBLISH_TIMEOUT_MS);
    s_at_cmd_active = false;

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "QMTPUBEX result wait failed: %s", esp_err_to_name(err));
        return err;
    }

    int pub_result = modem_parse_qmtpub_result(response);
    if (pub_result != 0)
    {
        ESP_LOGE(TAG, "QMTPUBEX failed result=%d", pub_result);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void modem_urc_task(void *arg)
{
    static char line_buf[1024];
    static int line_pos = 0;
    uint8_t rx_byte;

    while (1)
    {
        if (s_module_mqtt_connected && !s_at_cmd_active)
        {
            // 逐字节读取或小块读取，拼接到 line_buf 中，防止断帧
            int len = uart_read_bytes(UART_PORT_NUM, &rx_byte, 1, pdMS_TO_TICKS(50));
            if (len > 0)
            {
                if (line_pos < sizeof(line_buf) - 1)
                {
                    line_buf[line_pos++] = (char)rx_byte;
                }
                // 遇到换行符，或者我们发现这是一个完整的 URC 响应时进行处理
                if (rx_byte == '\n' || line_pos >= sizeof(line_buf) - 1)
                {
                    line_buf[line_pos] = '\0';
                    if (line_pos > 2)
                    {
                        modem_handle_urc(line_buf); // 只有完整行才传给解析
                    }
                    line_pos = 0; // 清空缓存，准备迎接下一行
                }
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

static esp_err_t init_4g_mqtt_internal(cb_communication_channel_established cb)
{
    esp_err_t err = ESP_OK;
    bool power_enabled = false;
    bool report_logged = false;
    ppp_4g_diag_result_t result = {
        .code = PPP_4G_DIAG_IO_ERROR,
    };
    int64_t total_start_us = esp_timer_get_time();
    int64_t stage_start_us = total_start_us;

    if (s_module_mqtt_connected)
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

    err = modem_mqtt_connect_session(&result);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "4G module MQTT connect failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    result.timing.total_ms = (uint32_t)((esp_timer_get_time() - total_start_us) / 1000LL);
    ppp_4g_log_result(&result);
    report_logged = true;

    // 创建独立的常驻后台任务专门捕获并派发 URC
    if (s_urc_task_handle == NULL)
    {
        xTaskCreate(modem_urc_task, "4g_urc_task", 4096, NULL, 5, &s_urc_task_handle);
    }

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
    if (power_enabled)
    {
        char *response = s_modem_response;
        if (s_module_mqtt_connected)
        {
            (void)modem_mqtt_disconnect(response, MODEM_RESP_BUF_SIZE);
            (void)modem_mqtt_close(response, MODEM_RESP_BUF_SIZE);
            s_module_mqtt_connected = false;
        }
    }
    (void)modem_shutdown_gracefully(s_at_ready);
    (void)modem_power_disable();
    modem_uart_deinit();
    return err;
}

esp_err_t init_4g_mqtt(cb_communication_channel_established cb)
{
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);
    esp_err_t err = init_4g_mqtt_internal(cb);
    xSemaphoreGive(s_at_mutex);
    return err;
}

esp_err_t bsp_4g_mqtt_publish(const char *topic, const uint8_t *data, size_t len)
{
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);
    esp_err_t err = modem_mqtt_publish_binary(topic, data, len);
    xSemaphoreGive(s_at_mutex);
    return err;
}

static esp_err_t bsp_4g_mqtt_disconnect_internal(void)
{
    char response[MODEM_RESP_BUF_SIZE];
    if (!s_module_mqtt_connected)
    {
        return ESP_OK;
    }

    esp_err_t err = modem_mqtt_disconnect(response, sizeof(response));
    (void)modem_mqtt_close(response, sizeof(response));
    s_module_mqtt_connected = false;
    return err;
}

esp_err_t bsp_4g_mqtt_disconnect(void)
{
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);
    esp_err_t err = bsp_4g_mqtt_disconnect_internal();
    xSemaphoreGive(s_at_mutex);
    return err;
}

static esp_err_t shutdown_4g_mqtt_internal(void)
{
    if (s_module_mqtt_connected)
    {
        (void)bsp_4g_mqtt_disconnect_internal();
    }
    (void)modem_shutdown_gracefully(s_at_ready);
    (void)modem_power_disable();
    modem_uart_deinit();
    s_at_ready = false;
    return ESP_OK;
}

esp_err_t shutdown_4g_mqtt(void)
{
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);
    esp_err_t err = shutdown_4g_mqtt_internal();
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

    // 互斥锁定：防止 URC 监听任务在此期间抢夺串口数据
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
         err, modem_response_is_ok(response), response);
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
        if (qerr == 0 && qstatus == 200 && qlen > 0)
        {
            // 4. 从模块内部提取 JSON 数据
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
            }
        }
        else
        {
            ESP_LOGW(TAG, "HTTP GET failed: AT_err=%d, HTTP_status=%d, content_len=%d", qerr, qstatus, qlen);
            err = ESP_FAIL;
        }
    }
    else
    {
        ESP_LOGW(TAG, "Unexpected QHTTPGET response: %s", response);
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

    ESP_LOGI(TAG, "Starting 4G Chunked OTA from MinIO: %s", host);

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
                ESP_LOGE(TAG, "MinIO range req rejected: err=%d, status=%d", qerr, qstatus);
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
                    ESP_LOGE(TAG, "OTA Write to Flash failed");
                    err = ESP_FAIL;
                    break;
                }
                offset += expect_len;
                retry_count = 0;
                ESP_LOGI(TAG, "OTA Progress: %d / %d bytes (%.1f%%)", offset, fw_size, (float)offset * 100.0 / fw_size);

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

static esp_err_t bsp_4g_http_put_internal(const char *url, const char *payload)
{
    if (!url || !payload)
        return ESP_ERR_INVALID_ARG;
    if (!s_at_ready)
        return ESP_ERR_INVALID_STATE;

    // 同样由于 QHTTP 默认是 POST，要实现真正的 PUT，需自己手写 Header
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

    modem_send_command("AT+QHTTPCFG=\"requestheader\",1", response, sizeof(response), 2000);

    snprintf(cmd, sizeof(cmd), "AT+QHTTPURL=%d,80\r\n", (int)strlen(url));
    uart_flush_input(UART_PORT_NUM);
    uart_write_bytes(UART_PORT_NUM, cmd, strlen(cmd));
    if (modem_read_until_pattern(response, sizeof(response), "CONNECT", 5000) == ESP_OK)
    {
        uart_write_bytes(UART_PORT_NUM, url, strlen(url));
        modem_read_response(response, sizeof(response), 5000);
    }

    int payload_len = strlen(payload);
    char req_header[512];
    int req_len = snprintf(req_header, sizeof(req_header),
                           "PUT %s HTTP/1.1\r\n"
                           "Host: %s\r\n"
                           "Content-Type: application/json\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n\r\n",
                           path, host, payload_len);

    int total_len = req_len + payload_len;
    snprintf(cmd, sizeof(cmd), "AT+QHTTPPOST=80,%d,80\r\n", total_len);
    uart_write_bytes(UART_PORT_NUM, cmd, strlen(cmd));

    if (modem_read_until_pattern(response, sizeof(response), "CONNECT", 5000) == ESP_OK)
    {
        uart_write_bytes(UART_PORT_NUM, req_header, req_len);
        uart_write_bytes(UART_PORT_NUM, payload, payload_len);

        err = modem_read_until_pattern(response, sizeof(response), "+QHTTPPOST:", 15000);
        if (err == ESP_OK)
        {
            int64_t qdeadline = esp_timer_get_time() + 1000000;
            while (strchr(response, '\n') == NULL && esp_timer_get_time() < qdeadline)
            {
                int rlen = strlen(response);
                int r = uart_read_bytes(UART_PORT_NUM, response + rlen, sizeof(response) - rlen - 1, pdMS_TO_TICKS(10));
                if (r > 0)
                    response[rlen + r] = '\0';
            }

            int qerr = -1, qstatus = -1;
            char *line = strstr(response, "+QHTTPPOST:");
            if (line && sscanf(line, "+QHTTPPOST: %d,%d", &qerr, &qstatus) >= 2)
            {
                if (qerr == 0 && (qstatus >= 200 && qstatus < 300))
                    err = ESP_OK;
                else
                    err = ESP_FAIL;
            }
            else
            {
                err = ESP_FAIL;
            }
        }
    }
    else
    {
        err = ESP_FAIL;
    }

    modem_send_command("AT+QHTTPCFG=\"requestheader\",0", response, sizeof(response), 2000);
    s_at_cmd_active = false;
    return err;
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
    esp_err_t err = bsp_4g_http_put_internal(url, payload);
    xSemaphoreGive(s_at_mutex);
    return err;
}

esp_err_t bsp_4g_http_get(const char *url, char **out_response)
{
    ensure_at_mutex();
    xSemaphoreTake(s_at_mutex, portMAX_DELAY);
    esp_err_t err = bsp_4g_http_get_internal(url, out_response);
    xSemaphoreGive(s_at_mutex);
    return err;
}
