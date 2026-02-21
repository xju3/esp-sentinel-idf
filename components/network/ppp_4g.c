#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <sys/param.h> // for MIN

#define PIN_TX 18
#define PIN_RX 17
#define PIN_PWR 14

#define UART_PORT_NUM UART_NUM_1
#define UART_BAUD_RATE 115200
#define BUF_SIZE (1024)

static const char *TAG = "ppp_4g";

// Check if URC content implies the modem is ready for AT commands
static bool urc_indicates_ready(const uint8_t *data, int len) {
    // Look for common banners: "RDY", "ATREADY", "+CPIN: READY"
    if (len <= 0) return false;
    const char *p = (const char *)data;
    if (strstr(p, "RDY") || strstr(p, "ATREADY") || strstr(p, "+CPIN: READY")) return true;
    return false;
}

// Drain any pending UART bytes (e.g., URC/banners) and log them; report if they show readiness
static bool uart_drain_with_log(void) {
    uint8_t tmp[BUF_SIZE];
    bool ready = false;
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, tmp, BUF_SIZE, pdMS_TO_TICKS(20));
        if (len <= 0) break;
        ESP_LOG_BUFFER_HEXDUMP(TAG, tmp, len, ESP_LOG_INFO);
        if (!ready && urc_indicates_ready(tmp, len)) ready = true;
    }
    uart_flush_input(UART_PORT_NUM);
    return ready;
}

static void ppp_4g_power_on(void) {
    ESP_LOGI(TAG, "PWRKEY low pulse start");
    gpio_set_level(PIN_PWR, 0);
    vTaskDelay(pdMS_TO_TICKS(1500)); // some modules need >1s low
    ESP_LOGI(TAG, "PWRKEY releasing (set high)");
    gpio_set_level(PIN_PWR, 1);
    vTaskDelay(pdMS_TO_TICKS(8000)); // allow full boot/URC flush
}

static esp_err_t send_at_command(const char *command, const char *expected_response, char *response_buffer, size_t buffer_size, int timeout_ms) {
    // Clear stale bytes before issuing a new command to avoid mixing responses
    uart_drain_with_log();
    uart_write_bytes(UART_PORT_NUM, command, strlen(command));
    uart_write_bytes(UART_PORT_NUM, "\r\n", 2);

    memset(response_buffer, 0, buffer_size);
    int total_len = 0;
    int remaining = buffer_size - 1;

    int loops = timeout_ms / 100;
    // Read in chunks to allow early exit and avoid blocking for full timeout
    for (int i = 0; i < loops; i++) {
        int len = uart_read_bytes(UART_PORT_NUM, (uint8_t *)response_buffer + total_len, remaining, pdMS_TO_TICKS(100));
        if (len > 0) {
            total_len += len;
            remaining -= len;
            response_buffer[total_len] = 0;
            // Log raw bytes to diagnose baud/echo issues without consuming elsewhere
            ESP_LOG_BUFFER_HEXDUMP(TAG, (const uint8_t *)response_buffer + total_len - len, len, ESP_LOG_INFO);
            if (strstr(response_buffer, expected_response) != NULL) {
                ESP_LOGI(TAG, "Response: %s", response_buffer);
                return ESP_OK;
            }
        }
        if (remaining <= 0) break;
    }
    ESP_LOGW(TAG, "AT Command Timeout/Fail. Got: %s", response_buffer);
    return ESP_FAIL;
}

esp_err_t ppp_4g_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_PWR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    // PWRKEY idle is high; set a known level before any toggle to avoid holding it low
    gpio_set_level(PIN_PWR, 1);

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // No event queue/task to avoid consuming RX data that AT parser needs
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    char response_buffer[256];

    // 1. Power‑up the modem unless we actually receive a proper OK
    bool modem_ready = false;
    for (int i = 0; i < 5; i++) {
        if (send_at_command("AT", "OK", response_buffer, sizeof(response_buffer), 1000) == ESP_OK) {
            modem_ready = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (!modem_ready) {
        ESP_LOGI(TAG, "Modem not responding, toggling power...");
        ppp_4g_power_on();

        // 等待模组完全启动并打印完 URC (如 RDY, SMS DONE)，避免干扰后续 AT 指令
        vTaskDelay(pdMS_TO_TICKS(6000));
    }

    // 无论是否刚开机，都循环等待直到模组能稳定响应 AT OK 或出现 READY URC
    // Wait for startup (A7670C can take a few seconds to be ready for AT)
    int retries = 20;
    ESP_LOGI(TAG, "Waiting for modem to sync (AT command)...");
    while (retries-- > 0) {
        // If URC already tells us it's ready, skip sending AT
        if (uart_drain_with_log()) {
            ESP_LOGI(TAG, "Modem indicated ready via URC");
            break;
        }
        if (send_at_command("AT", "OK", response_buffer, sizeof(response_buffer), 1500) == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (retries <= 0) {
        ESP_LOGE(TAG, "Modem not responding to AT (Sync failed)");
        return ESP_FAIL;
    }

    // Disable echo to simplify parsing and avoid confusing partial matches
    if (send_at_command("ATE0", "OK", response_buffer, sizeof(response_buffer), 1000) != ESP_OK) {
        ESP_LOGW(TAG, "ATE0 failed; continuing but responses may include echo");
    }

    ESP_LOGI(TAG, "Module synchronized, checking SIM status...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Check SIM Card Status
    if (send_at_command("AT+CPIN?", "+CPIN: READY", response_buffer, sizeof(response_buffer), 5000) != ESP_OK) {
        ESP_LOGE(TAG, "SIM Card not detected or PIN required");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "SIM Card Ready");

    /*
    // Get operator name
    if (send_at_command("AT+COPS?", "+COPS:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK) {
        char *start = strstr(response_buffer, "\"");
        if (start != NULL) {
            start++;
            char *end = strstr(start, "\"");
            if (end != NULL) {
                *end = '\0';
                ESP_LOGI(TAG, "Operator: %s", start);
            }
        }
    }

    // Get signal strength
    if (send_at_command("AT+CSQ", "+CSQ:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK) {
        int rssi, ber;
        if (sscanf(response_buffer, "%*s %d,%d", &rssi, &ber) == 2) {
            ESP_LOGI(TAG, "Signal Strength: %d, BER: %d", rssi, ber);
        }
    }

    // Check network registration
    if (send_at_command("AT+CREG?", "+CREG:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK) {
        int n, stat;
        if (sscanf(response_buffer, "%*s %d,%d", &n, &stat) == 2) {
            ESP_LOGI(TAG, "Network Registration Status: %d", stat);
            if (stat == 1 || stat == 5) {
                ESP_LOGI(TAG, "Registered to the network");
            } else {
                ESP_LOGW(TAG, "Not registered to the network");
            }
        }
    }

    // Check Internet Connection via Ping
    ESP_LOGI(TAG, "Checking Internet Connection...");
    // 1. Configure APN (CMNET is default for China Mobile, adjust if needed)
    // Note: We ignore errors here as it might be already configured
    send_at_command("AT+CGDCONT=1,\"IP\",\"CMNET\"", "OK", response_buffer, sizeof(response_buffer), 2000);
    
    // 2. Activate PDP Context (1,1 = Context ID 1, State Active)
    send_at_command("AT+CGACT=1,1", "OK", response_buffer, sizeof(response_buffer), 10000);

    // 3. Ping 114.114.114.114 (Reliable public DNS in China)
    // AT+CPING="IP",1 (Ping once)
    // Success response contains "+CPING: 1" (Reply received)
    if (send_at_command("AT+CPING=\"114.114.114.114\",1", "+CPING: 1", response_buffer, sizeof(response_buffer), 10000) == ESP_OK) {
        ESP_LOGI(TAG, "Internet Access: SUCCESS (Ping 114.114.114.114 OK)");
    } else {
        ESP_LOGW(TAG, "Internet Access: FAILED (Ping timeout or error)");
    }
    */

    return ESP_OK;
}

esp_err_t ppp_4g_deinit(void) {
    ESP_ERROR_CHECK(uart_driver_delete(UART_PORT_NUM));
    return ESP_OK;
}
