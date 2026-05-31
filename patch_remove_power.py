import sys

# Patch CMakeLists.txt
with open('components/bsp/CMakeLists.txt', 'r', encoding='utf-8') as f:
    cmake_content = f.read()
cmake_content = cmake_content.replace('        "src/bsp_power.c"\\n', '')
with open('components/bsp/CMakeLists.txt', 'w', encoding='utf-8') as f:
    f.write(cmake_content)

# Patch bsp_4g.c
with open('components/bsp/src/bsp_4g.c', 'r', encoding='utf-8') as f:
    content = f.read()

content = content.replace('#include "bsp_power.h"\\n', '')

content = content.replace('        (void)bsp_power_4g_disable();', '        gpio_set_level(MODEM_PWR_EN_PIN, 0);')

old_power_block = """    err = bsp_power_prepare_4g_energy();
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
    }"""

new_power_block = """    // 直接打开 4G 主供电
    gpio_set_level(MODEM_PWR_EN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); // 等待电源稳定"""

if old_power_block in content:
    content = content.replace(old_power_block, new_power_block)
else:
    print("Warning: Could not find old_power_block in bsp_4g.c")

old_disable_block = """    // 关闭电源使能
    (void)bsp_power_4g_disable();"""

new_disable_block = """    // 关闭电源使能
    gpio_set_level(MODEM_PWR_EN_PIN, 0);"""

if old_disable_block in content:
    content = content.replace(old_disable_block, new_disable_block)
else:
    print("Warning: Could not find old_disable_block in bsp_4g.c")

with open('components/bsp/src/bsp_4g.c', 'w', encoding='utf-8') as f:
    f.write(content)

