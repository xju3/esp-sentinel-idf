#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>

#include "init.h"
#include "config_manager.h"
#include "logger.h"
#include "task_baseline.h"
#include "web_server.h"
#include "machine_state.h"
#include "data_dispatcher.h"
#include "startup_gate.h"

void app_main(void)
{
    // 初始化 NVS (Wi-Fi 驱动必须用到)
    init_nvs();
    init_machine_state();
    ESP_ERROR_CHECK(config_manager_load(&g_user_config));
    // const bool had_valid_config_on_boot = g_user_config.is_configured;
    const bool had_valid_config_on_boot = true; // 开发阶段强制使用本地服务

    // startup_gate_reset();
    // startup_gate_set_waiting_for_config(true);
 

    // 直接启动检测程序
    ESP_ERROR_CHECK(start_local_services());
}
