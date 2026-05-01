#include "bsp_4g.h"
#include "bsp_board.h"
#include "bsp_power.h"
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
#include <sys/param.h>
#include <stdbool.h>
#include "freertos/event_groups.h"

#define PPP_VERBOSE 0 // 设为 1 可开启调试日志

// ============== Board pin aliases ==============
#define MODEM_UART_RX_PIN      BOARD_GPIO_4G_UART_RX
#define MODEM_UART_TX_PIN      BOARD_GPIO_4G_UART_TX
#define MODEM_PWR_EN_PIN       BOARD_GPIO_4G_PWR_EN
#define MODEM_PWRKEY_PIN       BOARD_GPIO_4G_PWRKEY
#define MODEM_STATUS_PIN       BOARD_GPIO_4G_STATUS
#define MODEM_NET_STATUS_PIN   BOARD_GPIO_4G_NET_STATUS
#define MODEM_RESET_N_PIN      BOARD_GPIO_4G_RESET_N

#define UART_PORT_NUM UART_NUM_1
#define BUF_SIZE (1024)
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

#define PPP_GOT_IP_BIT BIT0
#define PPP_FAILED_BIT BIT1

// 清空 UART 缓冲区并记录
static void uart_drain_with_log(void)
{
    uint8_t tmp[BUF_SIZE];
    int total = 0;
    while (1)
    {
        int len = uart_read_bytes(UART_PORT_NUM, tmp, BUF_SIZE, pdMS_TO_TICKS(50));
        if (len <= 0)
            break;
        total += len;
        if (PPP_VERBOSE)
        {
            ESP_LOGD(TAG, "Drained %d bytes:", len);
        }
    }
    if (PPP_VERBOSE && total > 0)
    {
        ESP_LOGD(TAG, "Total drained: %d bytes", total);
    }
    uart_flush_input(UART_PORT_NUM);
}

// 开机操作
static void ppp_4g_power_on(void)
{
    gpio_set_level(MODEM_PWRKEY_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1500));

    gpio_set_level(MODEM_PWRKEY_PIN, 1);

    // 稳定场景下缩短开机等待
    vTaskDelay(pdMS_TO_TICKS(1200));
}

// 发送 AT 命令
static esp_err_t send_at_command(const char *command, const char *expected_response,
                                 char *response_buffer, size_t buffer_size, int timeout_ms)
{
    // 清空缓冲区
    uart_drain_with_log();

    // 发送命令
    if (PPP_VERBOSE)
    {
        ESP_LOGD(TAG, "→ Sending: %s", command);
    }
    uart_write_bytes(UART_PORT_NUM, command, strlen(command));
    uart_write_bytes(UART_PORT_NUM, "\r\n", 2);

    memset(response_buffer, 0, buffer_size);
    int total_len = 0;
    int remaining = buffer_size - 1;

    int loops = timeout_ms / 100;
    for (int i = 0; i < loops; i++)
    {
        int len = uart_read_bytes(UART_PORT_NUM, (uint8_t *)response_buffer + total_len,
                                  remaining, pdMS_TO_TICKS(100));
        if (len > 0)
        {
            total_len += len;
            remaining -= len;
            response_buffer[total_len] = 0;

            // 打印接收到的数据
            if (PPP_VERBOSE)
            {
                ESP_LOGD(TAG, "← Received %d bytes:", len);
            }

            // 检查是否包含期望的响应
            if (strstr(response_buffer, expected_response) != NULL)
            {
                if (PPP_VERBOSE)
                {
                    ESP_LOGD(TAG, "✓ Got expected response: %s", expected_response);
                    ESP_LOGD(TAG, "  Full response: %s", response_buffer);
                }
                return ESP_OK;
            }
        }
        if (remaining <= 0)
            break;
    }
    if (PPP_VERBOSE)
    {
        ESP_LOGW(TAG, "✗ Timeout waiting for: %s", expected_response);
        ESP_LOGW(TAG, "  Got: %s", response_buffer);
    }
    return ESP_FAIL;
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

static void ppp_cleanup_failed_init(bool uart_driver_installed, bool power_enabled)
{
    ppp_stop_session();

    if (s_ppp_rx_task_running)
    {
        s_ppp_rx_task_running = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (uart_driver_installed)
    {
        (void)uart_driver_delete(UART_PORT_NUM);
    }

    if (power_enabled)
    {
        (void)bsp_power_4g_disable();
    }
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
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, ppp_ip_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, ppp_status_event_handler, NULL));
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

static esp_err_t ppp_start_and_wait_ip(void)
{
    char response_buffer[256];

    send_at_command("AT+CGATT=1", "OK", response_buffer, sizeof(response_buffer), 5000);

    if (send_at_command("ATD*99***1#", "CONNECT", response_buffer, sizeof(response_buffer), 30000) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enter PPP data mode");
        return ESP_FAIL;
    }

    if (!s_ppp_rx_task_running)
    {
        s_ppp_rx_task_running = true;
        if (xTaskCreate(ppp_uart_rx_task, "ppp_rx", 4096, NULL, 5, &s_ppp_rx_task) != pdTRUE)
        {
            s_ppp_rx_task_running = false;
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
            return ip_ready_err;
        }

        vTaskDelay(pdMS_TO_TICKS(PPP_POST_CONNECT_STABILIZE_MS));
        return ESP_OK;
    }

    ESP_LOGE(TAG, "PPP connect timeout/failed");
    ppp_stop_session();
    return ESP_FAIL;
}

// 尝试不同波特率
static esp_err_t detect_baudrate(uint32_t *detected_baud)
{
    const uint32_t target_baud = 115200;
    char response_buffer[256];

    uart_set_baudrate(UART_PORT_NUM, target_baud);
    vTaskDelay(pdMS_TO_TICKS(50));
    for (int retry = 0; retry < 2; retry++)
    {
        if (send_at_command("AT", "OK", response_buffer, sizeof(response_buffer), 1000) == ESP_OK)
        {
            *detected_baud = target_baud;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGE(TAG, "✗ No response at %d baud", target_baud);
    return ESP_FAIL;
}

esp_err_t init_ppp_4g(cb_communication_channel_established cb)
{
    esp_err_t err = ppp_stack_init();
    bool uart_driver_installed = false;
    bool power_enabled = false;
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "  ✗ Initialization Failed!");
        ESP_LOGE(TAG, "========================================");
        return err;
    }

    // ============== GPIO 配置 ==============
    // 配置 PWK 引脚
    gpio_config_t pwk_conf = {
        .pin_bit_mask = (1ULL << MODEM_PWRKEY_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&pwk_conf);
    gpio_set_level(MODEM_PWRKEY_PIN, 1); // PWRKEY 初始状态为高

    // 配置 4G 主电源使能引脚
    gpio_config_t pwr_en_conf = {
        .pin_bit_mask = (1ULL << MODEM_PWR_EN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&pwr_en_conf);
    gpio_set_level(MODEM_PWR_EN_PIN, 0); // 4G 主电源默认关闭

    // 板级占位：这些引脚在本阶段只收口定义，不改变现有 4G 时序逻辑。
    gpio_config_t modem_gpio_placeholder_conf = {
        .pin_bit_mask = (1ULL << MODEM_RESET_N_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&modem_gpio_placeholder_conf);
    gpio_set_level(MODEM_RESET_N_PIN, 1);

    gpio_config_t modem_status_conf = {
        .pin_bit_mask = (1ULL << MODEM_STATUS_PIN) | (1ULL << MODEM_NET_STATUS_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&modem_status_conf);

    err = bsp_power_prepare_4g_energy();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to prepare 4G power path: %s", esp_err_to_name(err));
        return err;
    }
    power_enabled = true;

    // ============== UART 配置 ==============
    uart_config_t uart_config = {
        .baud_rate = 115200, // 默认波特率，后续会尝试其他值
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    uart_driver_installed = true;
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, MODEM_UART_TX_PIN, MODEM_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    char response_buffer[512];

    // ============== Stage 2: 检查模组是否已开机 ==============
    bool module_on = false;
    uint32_t current_baud = 115200;

    // 先尝试当前波特率
    for (int i = 0; i < 3; i++)
    {
        if (send_at_command("AT", "OK", response_buffer, sizeof(response_buffer), 500) == ESP_OK)
        {
            module_on = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (!module_on)
    {
        ppp_4g_power_on();
        vTaskDelay(pdMS_TO_TICKS(1500));
        uart_drain_with_log();
    }

    // ============== Stage 3: 波特率检测 ==============
    if (detect_baudrate(&current_baud) != ESP_OK)
    {
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "  ✗ Initialization Failed!");
        ESP_LOGE(TAG, "========================================");
        err = ESP_FAIL;
        goto cleanup;
    }

    // ============== Stage 4: AT 同步 ==============
    int sync_retries = 2;
    while (sync_retries-- > 0)
    {
        if (send_at_command("AT", "OK", response_buffer, sizeof(response_buffer), 2000) == ESP_OK)
        {
            break;
        }
        ESP_LOGW(TAG, "  Retry %d/2...", 2 - sync_retries);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (sync_retries <= 0)
    {
        ESP_LOGE(TAG, "✗ AT sync failed");
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "  ✗ Initialization Failed!");
        ESP_LOGE(TAG, "========================================");
        err = ESP_FAIL;
        goto cleanup;
    }

    // ============== Stage 5: 关闭回显 ==============
    if (send_at_command("ATE0", "OK", response_buffer, sizeof(response_buffer), 2000) != ESP_OK)
    {
        ESP_LOGW(TAG, "✗ Failed to disable echo, continuing anyway");
    }

    // ============== Stage 6: 检查 SIM 卡 ==============
    vTaskDelay(pdMS_TO_TICKS(1000));

    int sim_retries = 2;
    while (sim_retries-- > 0)
    {
        if (send_at_command("AT+CPIN?", "+CPIN: READY", response_buffer,
                            sizeof(response_buffer), 3000) == ESP_OK)
        {
            break;
        }
        ESP_LOGW(TAG, "  SIM not ready, retry %d/2...", 2 - sim_retries);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (sim_retries <= 0)
    {
        ESP_LOGE(TAG, "✗ SIM Card not detected or not ready");
        ESP_LOGE(TAG, "  Please check:");
        ESP_LOGE(TAG, "  1. SIM card is properly inserted");
        ESP_LOGE(TAG, "  2. SIM card is activated and has credit");
        ESP_LOGE(TAG, "  3. SIM card supports 4G/LTE");
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "  ✗ Initialization Failed!");
        ESP_LOGE(TAG, "========================================");
        err = ESP_FAIL;
        goto cleanup;
    }

    // 读取 SIM 卡 ICCID
    if (send_at_command("AT+CCID", "+CCID:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK)
    {
        char *iccid_start = strstr(response_buffer, "+CCID:");
        if (iccid_start != NULL)
        {
            iccid_start += 7;
            char *newline = strstr(iccid_start, "\r");
            if (newline)
                *newline = '\0';
        }
    }

    // ============== Stage 7: 检查网络状态 ==============
    if (send_at_command("AT+COPS?", "+COPS:", response_buffer, sizeof(response_buffer), 3000) == ESP_OK)
    {
        char *start = strstr(response_buffer, "\"");
        if (start != NULL)
        {
            start++;
            char *end = strstr(start, "\"");
            if (end != NULL)
            {
                *end = '\0';
            }
        }
    }

    send_at_command("AT+CREG=1", "OK", response_buffer, sizeof(response_buffer), 2000);
    send_at_command("AT+CGDCONT=1,\"IP\",\"CMNET\"", "OK", response_buffer, sizeof(response_buffer), 3000);

    int reg_retries = 20; // 最多等待 20 秒
    bool registered_success = false;
    while (reg_retries-- > 0)
    {
        if (send_at_command("AT+CREG?", "+CREG:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK)
        {
            int n = 0, stat = 0;
            char *creg_start = strstr(response_buffer, "+CREG:");
            if (creg_start != NULL)
            {
                if (sscanf(creg_start, "+CREG: %d,%d", &n, &stat) == 2)
                {
                    if (stat == 1 || stat == 5)
                    {
                        registered_success = true;
                        break;
                    }
                }
            }
        }

        if (reg_retries > 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (!registered_success)
    {
        ESP_LOGW(TAG, "✗ Network registration timeout");
        ESP_LOGW(TAG, "  Note: Module may still work for some operations");
        if (send_at_command("AT+CEER", "+CEER:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK)
        {
            ESP_LOGW(TAG, "  Last error: %s", response_buffer);
        }

        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "  ✗ Initialization Failed!");
        ESP_LOGE(TAG, "========================================");
        err = ESP_FAIL;
        goto cleanup;
    }

    // ============== Stage 8: 启动 PPP 拨号 ==============
    err = ppp_start_and_wait_ip();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "  ✗ Initialization Failed!");
        ESP_LOGE(TAG, "========================================");
        goto cleanup;
    }

    if (cb)
    {
        cb();
    }

    return ESP_OK;

cleanup:
    ppp_cleanup_failed_init(uart_driver_installed, power_enabled);
    return err;
}

esp_err_t shutdown_ppp_4g(void)
{
    ppp_stop_session();

    if (s_ppp_rx_task_running)
    {
        s_ppp_rx_task_running = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 可选：发送关机命令
    char response_buffer[128];
    send_at_command("AT+CPOF", "OK", response_buffer, sizeof(response_buffer), 5000);

    // 关闭电源使能
    (void)bsp_power_4g_disable();

    // 删除 UART 驱动
    ESP_ERROR_CHECK(uart_driver_delete(UART_PORT_NUM));

    ppp_destroy_netif();

    return ESP_OK;
}
