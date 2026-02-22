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

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    if (pass) {
        strlcpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        LOG_ERRORF("esp_wifi_connect failed: %s", esp_err_to_name(err));
        return err;
    }

    LOG_INFOF("STA connecting to SSID:%s", ssid);
    return ESP_OK;
}
