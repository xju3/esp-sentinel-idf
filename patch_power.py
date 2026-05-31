import sys

with open('components/bsp/src/bsp_4g.c', 'r', encoding='utf-8') as f:
    content = f.read()

old_block = """    err = bsp_power_prepare_4g_energy();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to prepare 4G power path: %s", esp_err_to_name(err));
        return err;
    }
    power_enabled = true;"""

new_block = """    err = bsp_power_prepare_4g_energy();
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGW(TAG, "4G power energy check not supported, skipping and forcing enable...");
            gpio_set_level(MODEM_PWR_EN_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100)); // 给一点电容充电时间
        } else {
            ESP_LOGE(TAG, "Failed to prepare 4G power path: %s", esp_err_to_name(err));
            return err;
        }
    }
    power_enabled = true;"""

if old_block not in content:
    print("Block not found!")
else:
    content = content.replace(old_block, new_block)
    print("Block replaced successfully.")

with open('components/bsp/src/bsp_4g.c', 'w', encoding='utf-8') as f:
    f.write(content)

