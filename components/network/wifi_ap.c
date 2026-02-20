#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "network_manager.h" // 你自己的头文件
#include <string.h>

static const char *TAG = "WIFI_AP";

// AP 参数定义
#define SENTINEL_WIFI_PASS      "12345678"
#define SENTINEL_MAX_CONN       4
#define DEFAULT_SSID_PREFIX     "AP_SENTINEL_CONFIG_"

static void build_ap_ssid(char *out, size_t out_len)
{
    if (!out || out_len == 0) {
        return;
    }
#if defined(DEVICE_ID)
    if (DEVICE_ID[0] != '\0') {
        snprintf(out, out_len, "%s", DEVICE_ID);
        return;
    }
#endif
    uint32_t rnd = esp_random() % 1000;
    snprintf(out, out_len, "%s%03u", DEFAULT_SSID_PREFIX, (unsigned)rnd);
}

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
            .channel = 1,
            .password = SENTINEL_WIFI_PASS,
            .max_connection = SENTINEL_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    char ssid_buf[33] = {0};
    build_ap_ssid(ssid_buf, sizeof(ssid_buf));
    strncpy((char *)wifi_config.ap.ssid, ssid_buf, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen((const char *)wifi_config.ap.ssid);

    if (strlen(SENTINEL_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    // 4. 启动 Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP 启动成功! SSID:%s password:%s",
             (const char *)wifi_config.ap.ssid, SENTINEL_WIFI_PASS);
}
