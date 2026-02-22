#include "network_manager.h"

#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "logger.h"

esp_err_t wifi_init_sta(const char *ssid, const char *pass)
{
    if (!ssid || ssid[0] == '\0') {
        LOG_ERROR("SSID is empty");
        return ESP_ERR_INVALID_ARG;
    }

    // 使用公共初始化函数，创建 STA 网络接口（不需要 AP）
    ESP_ERROR_CHECK(wifi_common_init(false, true));

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    if (pass) {
        strlcpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    }

    // 使用 APSTA 模式，这样设备可以同时作为 AP（用于配置界面）和 STA（连接外部 WiFi）
    // 并且可以扫描热点
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        LOG_ERRORF("esp_wifi_connect failed: %s", esp_err_to_name(err));
        return err;
    }

    LOG_INFOF("STA connecting to SSID:%s (APSTA mode)", ssid);
    return ESP_OK;
}
