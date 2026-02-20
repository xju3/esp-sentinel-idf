#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "network_manager.h" // 你自己的头文件

static const char *TAG = "WIFI_AP";

// AP 参数定义
#define SENTINEL_WIFI_SSID      "Sentinel_Config"
#define SENTINEL_WIFI_PASS      "12345678"
#define SENTINEL_MAX_CONN       4

void wifi_init_softap(void) {
    // 1. 初始化底层 TCP/IP 堆栈
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    // 2. 初始化 Wi-Fi 驱动
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 3. 配置 AP 参数
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = SENTINEL_WIFI_SSID,
            .ssid_len = strlen(SENTINEL_WIFI_SSID),
            .channel = 1,
            .password = SENTINEL_WIFI_PASS,
            .max_connection = SENTINEL_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    if (strlen(SENTINEL_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    // 4. 启动 Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP 启动成功! SSID:%s password:%s",
             SENTINEL_WIFI_SSID, SENTINEL_WIFI_PASS);
}