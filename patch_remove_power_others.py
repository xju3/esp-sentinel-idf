import sys

# Patch CMakeLists.txt (again, since the previous script didn't match the exact line)
with open('components/bsp/CMakeLists.txt', 'r', encoding='utf-8') as f:
    lines = f.readlines()
with open('components/bsp/CMakeLists.txt', 'w', encoding='utf-8') as f:
    for line in lines:
        if '"src/bsp_power.c"' not in line:
            f.write(line)

# Patch bsp_4g.c (again, since include was not removed properly)
with open('components/bsp/src/bsp_4g.c', 'r', encoding='utf-8') as f:
    lines = f.readlines()
with open('components/bsp/src/bsp_4g.c', 'w', encoding='utf-8') as f:
    for line in lines:
        if '#include "bsp_power.h"' not in line:
            f.write(line)

# Patch off_sleep_manager.c
with open('components/task/src/off_sleep_manager.c', 'r', encoding='utf-8') as f:
    content = f.read()
content = content.replace('#include "bsp_power.h"\\n', '')
content = content.replace('    ret = bsp_power_sensor_disable();', '    gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);')
with open('components/task/src/off_sleep_manager.c', 'w', encoding='utf-8') as f:
    f.write(content)

# Patch task_baseline.c
with open('components/task/src/task_baseline.c', 'r', encoding='utf-8') as f:
    content = f.read()
content = content.replace('#include "bsp_power.h"\\n', '')
content = content.replace('        (void)bsp_power_sensor_disable();', '        gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);')
with open('components/task/src/task_baseline.c', 'w', encoding='utf-8') as f:
    f.write(content)

# Patch daq_iis3dwb.c
with open('components/peri/src/daq_iis3dwb.c', 'r', encoding='utf-8') as f:
    content = f.read()
content = content.replace('#include "bsp_power.h"\\n', '')
content = content.replace('    esp_err_t err = bsp_power_sensor_enable();', '    gpio_set_level(BOARD_GPIO_SENSOR_EN, 0);\\n    esp_err_t err = ESP_OK;')
content = content.replace('        (void)bsp_power_sensor_disable();', '        gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);')
content = content.replace('    err = bsp_power_sensor_disable();', '    gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);')
with open('components/peri/src/daq_iis3dwb.c', 'w', encoding='utf-8') as f:
    f.write(content)

