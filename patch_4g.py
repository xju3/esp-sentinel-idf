import sys

with open('components/bsp/src/bsp_4g.c', 'r', encoding='utf-8') as f:
    content = f.read()

old_block_1 = """static esp_err_t ppp_start_and_wait_ip(void)
{
    char response_buffer[256];

    send_at_command("AT+CGATT=1", "OK", response_buffer, sizeof(response_buffer), 5000);

    if (send_at_command("ATD*99***1#", "CONNECT", response_buffer, sizeof(response_buffer), 30000) != ESP_OK)
    {"""

new_block_1 = """static esp_err_t ppp_start_and_wait_ip(void)
{
    char response_buffer[256];

    // 精简：通常驻网成功后自动附着，略过 AT+CGATT=1 节省时间
    if (send_at_command("ATD*99***1#", "CONNECT", response_buffer, sizeof(response_buffer), 30000) != ESP_OK)
    {"""

old_block_2 = """    // ============== Stage 2: 检查模组是否已开机 ==============
    bool module_on = false;
    uint32_t current_baud = 115200;

    // 先用快速 AT 测试一次
    if (send_at_command("AT", "OK", response_buffer, sizeof(response_buffer), 300) == ESP_OK)
    {
        module_on = true;
    }

    if (!module_on)
    {
        ppp_4g_power_on();
        
        // 快速轮询 AT 指令直到模组就绪，最多等待约 5 秒
        for (int i = 0; i < 25; i++)
        {
            if (send_at_command("AT", "OK", response_buffer, sizeof(response_buffer), 200) == ESP_OK)
            {
                module_on = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        uart_drain_with_log();
    }

    // ============== AT 同步 ==============
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

    // ============== Stage 7: 检查网络状态 ==============
    send_at_command("AT+CREG=1", "OK", response_buffer, sizeof(response_buffer), 2000);
    send_at_command("AT+CGDCONT=1,\\"IP\\",\\"CMNET\\"", "OK", response_buffer, sizeof(response_buffer), 3000);

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
    }"""

new_block_2 = """    // ============== Stage 2: 硬件状态引脚加速开机判定 ==============
    bool module_on = (gpio_get_level(MODEM_STATUS_PIN) == 1);

    if (!module_on)
    {
        if (PPP_VERBOSE) ESP_LOGI(TAG, "STATUS pin low, powering on module...");
        ppp_4g_power_on();
        
        // 使用 STATUS 引脚判断是否开机，最长等待 3 秒
        for (int i = 0; i < 30; i++)
        {
            if (gpio_get_level(MODEM_STATUS_PIN) == 1)
            {
                module_on = true;
                if (PPP_VERBOSE) ESP_LOGI(TAG, "STATUS pin high, module powered on.");
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    if (!module_on)
    {
        ESP_LOGW(TAG, "STATUS pin still low, fallback to AT test...");
    }

    // 快速 AT 同步，只要有回应就说明串口通了
    bool at_sync = false;
    for (int i = 0; i < 15; i++)
    {
        if (send_at_command("AT", "OK", response_buffer, sizeof(response_buffer), 300) == ESP_OK)
        {
            at_sync = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!at_sync)
    {
        ESP_LOGE(TAG, "✗ AT sync failed");
        err = ESP_FAIL;
        goto cleanup;
    }

    // ============== Stage 3: 精简配置与状态检查 ==============
    // 关闭回显 (缩短超时)
    send_at_command("ATE0", "OK", response_buffer, sizeof(response_buffer), 500);

    // 检查 SIM 卡，轮询等待，缩短单次延时，最多等3秒
    bool sim_ready = false;
    for (int i = 0; i < 15; i++)
    {
        if (send_at_command("AT+CPIN?", "+CPIN: READY", response_buffer, sizeof(response_buffer), 300) == ESP_OK)
        {
            sim_ready = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (!sim_ready)
    {
        ESP_LOGE(TAG, "✗ SIM Card not ready");
        err = ESP_FAIL;
        goto cleanup;
    }

    // 快速下发必要配置 (忽略错误，以防因为基带忙导致全流程失败)
    send_at_command("AT+CREG=1", "OK", response_buffer, sizeof(response_buffer), 500);
    send_at_command("AT+CGDCONT=1,\\"IP\\",\\"CMNET\\"", "OK", response_buffer, sizeof(response_buffer), 500);

    // 检查网络注册状态，快速轮询 (每次间隔500ms，总计约10秒)
    bool registered_success = false;
    for (int i = 0; i < 20; i++)
    {
        if (send_at_command("AT+CREG?", "+CREG:", response_buffer, sizeof(response_buffer), 500) == ESP_OK)
        {
            int n = 0, stat = 0;
            char *creg_start = strstr(response_buffer, "+CREG:");
            if (creg_start != NULL && sscanf(creg_start, "+CREG: %d,%d", &n, &stat) == 2)
            {
                if (stat == 1 || stat == 5)
                {
                    registered_success = true;
                    break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (!registered_success)
    {
        ESP_LOGW(TAG, "✗ Network registration timeout");
        err = ESP_FAIL;
        goto cleanup;
    }"""

if old_block_1 not in content:
    print("Block 1 not found!")
else:
    content = content.replace(old_block_1, new_block_1)
    print("Block 1 replaced successfully.")

if old_block_2 not in content:
    print("Block 2 not found!")
else:
    content = content.replace(old_block_2, new_block_2)
    print("Block 2 replaced successfully.")

with open('components/bsp/src/bsp_4g.c', 'w', encoding='utf-8') as f:
    f.write(content)

