#include "bsp_4g.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <sys/param.h>
#include <stdbool.h>

#define PPP_VERBOSE 0 // 设为 1 可开启调试日志

// ============== 引脚定义 ==============
#define PIN_TX GPIO_NUM_18  // ESP32-S3 TX -> 转接板 TX
#define PIN_RX GPIO_NUM_17  // ESP32-S3 RX -> 转接板 RX
#define PIN_PEN GPIO_NUM_13 // ESP32-S3 GPIO13 -> 转接板 PEN (Power Enable)
#define PIN_PWK GPIO_NUM_14 // ESP32-S3 GPIO14 -> 转接板 PWK (PWRKEY)

#define UART_PORT_NUM UART_NUM_1
#define BUF_SIZE (1024)

static const char *TAG = "ppp_4g";

// 初始化状态
typedef enum
{
    INIT_STAGE_START = 0,
    INIT_STAGE_POWER_ENABLE,
    INIT_STAGE_POWER_ON,
    INIT_STAGE_BAUD_DETECT,
    INIT_STAGE_AT_SYNC,
    INIT_STAGE_DISABLE_ECHO,
    INIT_STAGE_CHECK_SIM,
    INIT_STAGE_CHECK_NETWORK,
    INIT_STAGE_COMPLETE,
    INIT_STAGE_FAILED
} init_stage_t;

static void log_stage(init_stage_t stage)
{
    switch (stage)
    {
    case INIT_STAGE_START:
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  Stage 0: Initialization Started");
        ESP_LOGI(TAG, "========================================");
        break;
    case INIT_STAGE_POWER_ENABLE:
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  Stage 1: Enabling Power (PEN Pin)");
        ESP_LOGI(TAG, "========================================");
        break;
    case INIT_STAGE_POWER_ON:
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  Stage 2: Power On Module (PWK Pin)");
        ESP_LOGI(TAG, "========================================");
        break;
    case INIT_STAGE_BAUD_DETECT:
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  Stage 3: Detecting Baud Rate");
        ESP_LOGI(TAG, "========================================");
        break;
    case INIT_STAGE_AT_SYNC:
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  Stage 4: AT Command Synchronization");
        ESP_LOGI(TAG, "========================================");
        break;
    case INIT_STAGE_DISABLE_ECHO:
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  Stage 5: Disabling Echo");
        ESP_LOGI(TAG, "========================================");
        break;
    case INIT_STAGE_CHECK_SIM:
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  Stage 6: Checking SIM Card");
        ESP_LOGI(TAG, "========================================");
        break;
    case INIT_STAGE_CHECK_NETWORK:
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  Stage 7: Checking Network Status");
        ESP_LOGI(TAG, "========================================");
        break;
    case INIT_STAGE_COMPLETE:
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  ✓ Initialization Complete!");
        ESP_LOGI(TAG, "========================================");
        break;
    case INIT_STAGE_FAILED:
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "  ✗ Initialization Failed!");
        ESP_LOGE(TAG, "========================================");
        break;
    }
}

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
            ESP_LOGI(TAG, "Drained %d bytes:", len);
            ESP_LOG_BUFFER_HEXDUMP(TAG, tmp, len, ESP_LOG_INFO);
        }
    }
    if (PPP_VERBOSE && total > 0)
    {
        ESP_LOGI(TAG, "Total drained: %d bytes", total);
    }
    uart_flush_input(UART_PORT_NUM);
}

// 电源使能
static void ppp_4g_power_enable(void)
{
    ESP_LOGI(TAG, "→ Setting PEN pin HIGH (Enable power regulator)");
    gpio_set_level(PIN_PEN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "✓ Power regulator enabled");
}

// 开机操作
static void ppp_4g_power_on(void)
{
    ESP_LOGI(TAG, "→ Toggling PWK pin to power on module");
    ESP_LOGI(TAG, "  - Setting PWK LOW");
    gpio_set_level(PIN_PWK, 0);
    vTaskDelay(pdMS_TO_TICKS(1500));

    ESP_LOGI(TAG, "  - Setting PWK HIGH (Release)");
    gpio_set_level(PIN_PWK, 1);

    // A7670C 在带 SIM 卡时初始化更慢，提前预留更长启动窗口
    ESP_LOGI(TAG, "→ Waiting for module boot (5 seconds)...");
    for (int i = 5; i > 0; i--)
    {
        ESP_LOGI(TAG, "  %d...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "✓ Boot wait complete");
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
        ESP_LOGI(TAG, "→ Sending: %s", command);
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
                ESP_LOGI(TAG, "← Received %d bytes:", len);
                ESP_LOG_BUFFER_HEXDUMP(TAG, (const uint8_t *)response_buffer + total_len - len,
                                       len, ESP_LOG_INFO);
            }

            // 检查是否包含期望的响应
            if (strstr(response_buffer, expected_response) != NULL)
            {
                if (PPP_VERBOSE)
                {
                    ESP_LOGI(TAG, "✓ Got expected response: %s", expected_response);
                    ESP_LOGI(TAG, "  Full response: %s", response_buffer);
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

static const char *cnmp_mode_to_str(int mode)
{
    switch (mode)
    {
    case 2:
        return "AUTO (2G/3G/4G)";
    case 13:
        return "GSM only";
    case 14:
        return "WCDMA only";
    case 38:
        return "LTE only";
    default:
        return "Unknown";
    }
}

// 在成功注册网络后打印一次 4G 运行信息
static void log_4g_runtime_info(void)
{
    char response_buffer[512];

    const char *operator = "?";
    int rssi = 99, ber = 99, dbm = -999;
    int mode = -1, attached = -1;
    char cell_info[128] = {0};
    char phone_number[64] = {0};

    if (send_at_command("AT+COPS?", "+COPS:", response_buffer, sizeof(response_buffer), 3000) == ESP_OK)
    {
        char *name_start = strstr(response_buffer, "\"");
        if (name_start)
        {
            name_start++;
            char *name_end = strstr(name_start, "\"");
            if (name_end)
                *name_end = '\0';
            operator = name_start;
        }
    }

    if (send_at_command("AT+CSQ", "+CSQ:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK)
    {
        char *csq_start = strstr(response_buffer, "+CSQ:");
        if (csq_start && sscanf(csq_start, "+CSQ: %d,%d", &rssi, &ber) == 2)
        {
            if (rssi >= 0 && rssi <= 31)
                dbm = -113 + 2 * rssi;
        }
    }

    if (send_at_command("AT+CNMP?", "+CNMP:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK)
    {
        sscanf(response_buffer, "%*[^:]: %d", &mode);
    }

    if (send_at_command("AT+CPSI?", "+CPSI:", response_buffer, sizeof(response_buffer), 3000) == ESP_OK)
    {
        strncpy(cell_info, response_buffer, sizeof(cell_info) - 1);
    }

    if (send_at_command("AT+CGATT?", "+CGATT:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK)
    {
        sscanf(response_buffer, "%*[^:]: %d", &attached);
    }

    // SIM 号码 (部分运营商可能未写入或被禁用)
    if (send_at_command("AT+CNUM", "+CNUM:", response_buffer, sizeof(response_buffer), 3000) == ESP_OK)
    {
        // +CNUM: "Voice","13800138000",129,7,4
        char num[64] = {0};
        if (sscanf(response_buffer, "%*[^,],\"%63[^\"]", num) == 1)
        {
            strncpy(phone_number, num, sizeof(phone_number) - 1);
        }
    }

    ESP_LOGI(TAG, "4G: operator=%s, rssi=%d (%d dBm), ber=%d, mode=%s(%d), attached=%s",
             operator, rssi, dbm, ber, cnmp_mode_to_str(mode), mode,
             attached == 1 ? "yes" : (attached == 0 ? "no" : "?"));

    if (cell_info[0])
    {
        ESP_LOGI(TAG, "4G cell: %s", cell_info);
    }
    if (phone_number[0])
    {
        ESP_LOGI(TAG, "SIM MSISDN: %s", phone_number);
    }
    else
    {
        ESP_LOGI(TAG, "SIM MSISDN: unavailable");
    }
}

// 尝试不同波特率
static esp_err_t detect_baudrate(uint32_t *detected_baud)
{
    uint32_t baud_rates[] = {115200, 9600, 57600, 38400, 19200};
    int num_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);
    char response_buffer[256];

    for (int i = 0; i < num_rates; i++)
    {
        ESP_LOGI(TAG, "→ Trying baud rate: %d", baud_rates[i]);
        uart_set_baudrate(UART_PORT_NUM, baud_rates[i]);
        vTaskDelay(pdMS_TO_TICKS(100));

        // 尝试 5 次，SIM 初始化时模组响应会更慢
        for (int retry = 0; retry < 5; retry++)
        {
            if (send_at_command("AT", "OK", response_buffer, sizeof(response_buffer), 1500) == ESP_OK)
            {
                *detected_baud = baud_rates[i];
                ESP_LOGI(TAG, "✓ Baud rate detected: %d", baud_rates[i]);
                return ESP_OK;
            }
            vTaskDelay(pdMS_TO_TICKS(300));
        }

        ESP_LOGW(TAG, "✗ No response at %d baud", baud_rates[i]);
    }

    ESP_LOGE(TAG, "✗ Failed to detect baud rate");
    return ESP_FAIL;
}

esp_err_t ppp_4g_init(void)
{
    init_stage_t stage = INIT_STAGE_START;
    log_stage(stage);

    // ============== GPIO 配置 ==============
    ESP_LOGI(TAG, "→ Configuring GPIO pins");

    // 配置 PWK 引脚
    gpio_config_t pwk_conf = {
        .pin_bit_mask = (1ULL << PIN_PWK),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&pwk_conf);
    gpio_set_level(PIN_PWK, 1); // PWK 初始状态为高
    ESP_LOGI(TAG, "  ✓ PWK pin configured (GPIO%d)", PIN_PWK);

    // 配置 PEN 引脚
    gpio_config_t pen_conf = {
        .pin_bit_mask = (1ULL << PIN_PEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&pen_conf);
    gpio_set_level(PIN_PEN, 0); // PEN 初始状态为低
    ESP_LOGI(TAG, "  ✓ PEN pin configured (GPIO%d)", PIN_PEN);

    // ============== UART 配置 ==============
    ESP_LOGI(TAG, "→ Configuring UART");
    uart_config_t uart_config = {
        .baud_rate = 115200, // 默认波特率，后续会尝试其他值
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "  ✓ UART%d configured: TX=GPIO%d, RX=GPIO%d, Baud=%d",
             UART_PORT_NUM, PIN_TX, PIN_RX, uart_config.baud_rate);

    // 带 SIM 卡时模组会先做握卡流程，额外等待 2s 让串口稳定
    ESP_LOGI(TAG, "→ Extra settle wait for SIM init (2 seconds)...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    char response_buffer[512];

    // ============== Stage 1: 使能电源 ==============
    stage = INIT_STAGE_POWER_ENABLE;
    log_stage(stage);
    ppp_4g_power_enable();

    // ============== Stage 2: 检查模组是否已开机 ==============
    stage = INIT_STAGE_POWER_ON;
    log_stage(stage);

    ESP_LOGI(TAG, "→ Checking if module is already powered on...");
    bool module_on = false;
    uint32_t current_baud = 115200;

    // 先尝试当前波特率
    for (int i = 0; i < 3; i++)
    {
        if (send_at_command("AT", "OK", response_buffer, sizeof(response_buffer), 500) == ESP_OK)
        {
            module_on = true;
            ESP_LOGI(TAG, "✓ 4G Module already powered on and responding");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (!module_on)
    {
        ESP_LOGI(TAG, "✗ Module not responding, will toggle power");
        ppp_4g_power_on();

        // 开机后清空 URC 消息
        ESP_LOGI(TAG, "→ Draining boot messages (URC)...");
        vTaskDelay(pdMS_TO_TICKS(1500));
        uart_drain_with_log();
    }

    // ============== Stage 3: 波特率检测 ==============
    stage = INIT_STAGE_BAUD_DETECT;
    log_stage(stage);

    if (detect_baudrate(&current_baud) != ESP_OK)
    {
        stage = INIT_STAGE_FAILED;
        log_stage(stage);
        return ESP_FAIL;
    }

    // ============== Stage 4: AT 同步 ==============
    stage = INIT_STAGE_AT_SYNC;
    log_stage(stage);

    ESP_LOGI(TAG, "→ Synchronizing with module (sending AT)...");
    int sync_retries = 3;
    while (sync_retries-- > 0)
     {
        if (send_at_command("AT", "OK", response_buffer, sizeof(response_buffer), 2000) == ESP_OK)
        {
            ESP_LOGI(TAG, "✓ AT sync successful");
            break;
        }
        ESP_LOGW(TAG, "  Retry %d/10...", 10 - sync_retries);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (sync_retries <= 0)
    {
        ESP_LOGE(TAG, "✗ AT sync failed after 10 retries");
        stage = INIT_STAGE_FAILED;
        log_stage(stage);
        return ESP_FAIL;
    }

    // ============== Stage 5: 关闭回显 ==============
    stage = INIT_STAGE_DISABLE_ECHO;
    log_stage(stage);

    if (send_at_command("ATE0", "OK", response_buffer, sizeof(response_buffer), 2000) != ESP_OK)
    {
        ESP_LOGW(TAG, "✗ Failed to disable echo, continuing anyway");
    }
    else
    {
        ESP_LOGI(TAG, "✓ Echo disabled");
    }

    // ============== Stage 6: 检查 SIM 卡 ==============
    stage = INIT_STAGE_CHECK_SIM;
    log_stage(stage);

    ESP_LOGI(TAG, "→ Checking SIM card status...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    int sim_retries = 3;
    while (sim_retries-- > 0)
    {
        if (send_at_command("AT+CPIN?", "+CPIN: READY", response_buffer,
                            sizeof(response_buffer), 3000) == ESP_OK)
        {
            ESP_LOGI(TAG, "✓ SIM Card Ready");
            break;
        }
        ESP_LOGW(TAG, "  SIM not ready, retry %d/10...", 10 - sim_retries);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    if (sim_retries <= 0)
    {
        ESP_LOGE(TAG, "✗ SIM Card not detected or not ready");
        ESP_LOGE(TAG, "  Please check:");
        ESP_LOGE(TAG, "  1. SIM card is properly inserted");
        ESP_LOGE(TAG, "  2. SIM card is activated and has credit");
        ESP_LOGE(TAG, "  3. SIM card supports 4G/LTE");
        stage = INIT_STAGE_FAILED;
        log_stage(stage);
        return ESP_FAIL;
    }

    // 读取 SIM 卡 ICCID
    ESP_LOGI(TAG, "→ Reading SIM card ICCID...");
    if (send_at_command("AT+CCID", "+CCID:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK)
    {
        char *iccid_start = strstr(response_buffer, "+CCID:");
        if (iccid_start != NULL)
        {
            // 跳过 "+CCID: "
            iccid_start += 7;
            // 去掉换行
            char *newline = strstr(iccid_start, "\r");
            if (newline)
                *newline = '\0';
            ESP_LOGI(TAG, "✓ SIM ICCID: %s", iccid_start);
        }
    }

    // ============== Stage 7: 检查网络状态 ==============
    stage = INIT_STAGE_CHECK_NETWORK;
    log_stage(stage);

    // 获取运营商信息
    ESP_LOGI(TAG, "→ Getting operator information...");
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
                ESP_LOGI(TAG, "✓ Operator: %s", start);
            }
        }
    }

    bool antenna_suspect = false;

    // 获取信号强度（多次重试，避免刚开机误报 99）
    ESP_LOGI(TAG, "→ Getting signal strength...");
    int rssi = 99, ber = 99;
    bool got_signal = false;
    for (int attempt = 0; attempt < 5; attempt++)
    {
        if (send_at_command("AT+CSQ", "+CSQ:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK)
        {
            char *csq_start = strstr(response_buffer, "+CSQ:");
            if (csq_start != NULL && sscanf(csq_start, "+CSQ: %d,%d", &rssi, &ber) == 2)
            {
                ESP_LOGI(TAG, "  Attempt %d: RSSI=%d, BER=%d", attempt + 1, rssi, ber);
                if (rssi != 99)
                {
                    got_signal = true;
                    break;
                }
            }
        }
        ESP_LOGW(TAG, "  No valid signal yet, retrying...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    if (!got_signal && rssi == 99)
    {
        antenna_suspect = true; // 延后在注册结束时统一报警，避免重复刷屏
    }
    else if (rssi < 10)
    {
        ESP_LOGW(TAG, "  Warning: Weak signal (RSSI=%d)", rssi);
    }
    else
    {
        ESP_LOGI(TAG, "  Signal quality: Good (RSSI=%d)", rssi);
    }

    // 启用网络注册 URC
    ESP_LOGI(TAG, "→ Enabling network registration notifications...");
    send_at_command("AT+CREG=1", "OK", response_buffer, sizeof(response_buffer), 2000);

    // 检查当前网络模式
    ESP_LOGI(TAG, "→ Checking network mode...");
    if (send_at_command("AT+CNMP?", "+CNMP:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK)
    {
        ESP_LOGI(TAG, "  Current mode: %s", response_buffer);
    }

    // 中国移动常用 APN：CMNET（物联网卡可能为 CMIOT，可根据需要改）
    ESP_LOGI(TAG, "→ Setting PDP APN to CMNET...");
    send_at_command("AT+CGDCONT=1,\"IP\",\"CMNET\"", "OK", response_buffer, sizeof(response_buffer), 3000);

    // 设置为自动模式（2=自动，支持 2G/3G/4G）
    ESP_LOGI(TAG, "→ Setting network mode to AUTO...");
    send_at_command("AT+CNMP=2", "OK", response_buffer, sizeof(response_buffer), 2000);

    // 手动触发网络搜索
    ESP_LOGI(TAG, "→ Manually triggering network search...");
    send_at_command("AT+COPS=0", "OK", response_buffer, sizeof(response_buffer), 30000);

    // 检查网络注册状态
    ESP_LOGI(TAG, "→ Checking network registration...");
    int reg_retries = 45; // 最多等待 90 秒，部分网络初始注册较慢
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
                    if (stat == 1)
                    {
                        ESP_LOGI(TAG, "✓ Registered to home network");
                        registered_success = true;
                        break;
                    }
                    else if (stat == 5)
                    {
                        ESP_LOGI(TAG, "✓ Registered to roaming network");
                        registered_success = true;
                        break;
                    }
                }
            }
        }

        // LTE/EPS 注册结果（有些网络只在 CEREG 上报告）
        if (!registered_success &&
            send_at_command("AT+CEREG?", "+CEREG:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK)
        {
            int n = 0, stat = 0;
            char *cereg_start = strstr(response_buffer, "+CEREG:");
            if (cereg_start != NULL)
            {
                if (sscanf(cereg_start, "+CEREG: %d,%d", &n, &stat) == 2)
                {
                    if (stat == 1 || stat == 5)
                    {
                        ESP_LOGI(TAG, "✓ EPS Registered (CEREG stat=%d)", stat);
                        registered_success = true;
                        break;
                    }
                }
            }
        }

        // 中途仍未注册时，尝试切到 LTE only 再回自动，加速中国移动驻网
        if (!registered_success && reg_retries == 25)
        {
            ESP_LOGW(TAG, "  Long searching, switching to LTE only (CNMP=38)...");
            send_at_command("AT+CNMP=38", "OK", response_buffer, sizeof(response_buffer), 5000);
            send_at_command("AT+COPS=0", "OK", response_buffer, sizeof(response_buffer), 30000);
        }
        if (!registered_success && reg_retries == 15)
        {
            ESP_LOGW(TAG, "  Restoring AUTO search (CNMP=2) and retrying...");
            send_at_command("AT+CNMP=2", "OK", response_buffer, sizeof(response_buffer), 5000);
            send_at_command("AT+COPS=0", "OK", response_buffer, sizeof(response_buffer), 30000);
        }

        if (reg_retries > 0)
        {
            ESP_LOGI(TAG, "  Waiting for network registration (%d seconds left)...", reg_retries * 2);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    if (registered_success)
    {
        // 注册成功后打印详细的 4G 运行信息
        log_4g_runtime_info();
    }
    else
    {
        ESP_LOGW(TAG, "✗ Network registration timeout");
        ESP_LOGW(TAG, "  Note: Module may still work for some operations");
        // 输出最后一次附着失败原因，便于现场定位
        if (send_at_command("AT+CEER", "+CEER:", response_buffer, sizeof(response_buffer), 2000) == ESP_OK)
        {
            ESP_LOGW(TAG, "  Last error: %s", response_buffer);
        }

        if (antenna_suspect)
        {
            ESP_LOGE(TAG, "");
            ESP_LOGE(TAG, "================================================");
            ESP_LOGE(TAG, "  CRITICAL: NO ANTENNA OR NO SIGNAL DETECTED!");
            ESP_LOGE(TAG, "================================================");
            ESP_LOGE(TAG, "  If antenna is connected, wait in open area or check band settings.");
            ESP_LOGE(TAG, "  Check RF cable / antenna seating / supported bands.");
            ESP_LOGE(TAG, "================================================");
            ESP_LOGE(TAG, "");
        }
        else
        {
            ESP_LOGW(TAG, "  RSSI had valid reading earlier; timeout may be coverage/PLMN issue.");
        }
    }

    // ============== 初始化完成 ==============
    stage = INIT_STAGE_COMPLETE;
    log_stage(stage);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Module Information:");
    ESP_LOGI(TAG, "  - Baud Rate: %d", current_baud);
    ESP_LOGI(TAG, "  - UART: UART%d (TX=GPIO%d, RX=GPIO%d)", UART_PORT_NUM, PIN_TX, PIN_RX);
    ESP_LOGI(TAG, "  - Control: PWK=GPIO%d, PEN=GPIO%d", PIN_PWK, PIN_PEN);
    ESP_LOGI(TAG, "");

    return ESP_OK;
}

esp_err_t ppp_4g_deinit(void)
{
    ESP_LOGI(TAG, "→ Shutting down 4G module...");

    // 可选：发送关机命令
    char response_buffer[128];
    send_at_command("AT+CPOF", "OK", response_buffer, sizeof(response_buffer), 5000);

    // 关闭电源使能
    gpio_set_level(PIN_PEN, 0);
    ESP_LOGI(TAG, "  ✓ Power disabled");

    // 删除 UART 驱动
    ESP_ERROR_CHECK(uart_driver_delete(UART_PORT_NUM));
    ESP_LOGI(TAG, "✓ 4G module shutdown complete");

    return ESP_OK;
}
