#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "nvs_flash.h"
#include "network_manager.h"
#include "config_manager.h"
#include "web_server.h"
#include "logger.h"
#include "sdkconfig.h"

void app_main(void) {
    // 1. 初始化 NVS (Wi-Fi 驱动必须用到)
    LOG_INFO("Starting device...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. 加载配置
    user_config_t cfg = {0};
    ESP_ERROR_CHECK(config_manager_load(&cfg));

    // 3. 若未配置，启动 AP + Captive Portal
    if (!cfg.is_configured) {
        wifi_init_softap();
        ESP_ERROR_CHECK(web_server_start());
        return;
    }

    // 已配置：按用户 Wi-Fi 信息连接 STA
    ESP_ERROR_CHECK(wifi_init_sta(cfg.wifi.ssid, cfg.wifi.pass));

#ifdef CONFIG_DEV_MODE
    // 开发模式下仍保留 Web 调试入口
    ESP_ERROR_CHECK(web_server_start());
#endif
}
