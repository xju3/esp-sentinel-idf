#include "nvs_flash.h"
#include "network_manager.h"
#include "config_manager.h"
#include "web_server.h"

void app_main(void) {
    // 1. 初始化 NVS (Wi-Fi 驱动必须用到)
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

    // TODO: 已配置时可切换到 STA 模式
    // wifi_init_sta(cfg.wifi.ssid, cfg.wifi.pass);
}
