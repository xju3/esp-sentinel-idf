#include "bsp_wifi.h"
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "logger.h"

// STA 连接成功回调（仅 STA 使用）
static cb_communication_channel_established s_wifi_connected_cb = NULL;

// 内部事件处理函数，处理STA连接成功事件和IP获取事件
static void wifi_sta_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        LOG_INFO("Wi-Fi STA connected");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        LOG_INFOF("Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        // 等待网络完全就绪
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (s_wifi_connected_cb)
        {
            s_wifi_connected_cb();
        }
    }
}

esp_err_t wifi_init_sta(const char *ssid, const char *pass, cb_communication_channel_established cb)
{
    if (!ssid || ssid[0] == '\0')
    {
        LOG_ERROR("SSID is empty");
        return ESP_ERR_INVALID_ARG;
    }

    // 保存回调函数
    s_wifi_connected_cb = cb;

    // 使用公共初始化函数，创建 STA 网络接口（不需要 AP）
    ESP_ERROR_CHECK(wifi_common_init(false, true));

    // 注册STA连接成功和IP获取事件处理器
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &wifi_sta_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_sta_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    if (pass)
    {
        strlcpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    }

    // 使用 APSTA 模式，这样设备可以同时作为 AP（用于配置界面）和 STA（连接外部 WiFi）
    // 并且可以扫描热点
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK)
    {
        LOG_ERRORF("esp_wifi_connect failed: %s", esp_err_to_name(err));
        return err;
    }

    LOG_INFOF("STA connecting to SSID:%s (APSTA mode)", ssid);
    return ESP_OK;
}

esp_err_t wifi_stop_sta(void)
{
    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_err_t err = esp_wifi_get_mode(&mode);
    if (err != ESP_OK)
    {
        return err;
    }

    if (mode == WIFI_MODE_APSTA)
    {
        (void)esp_wifi_disconnect();
        return esp_wifi_set_mode(WIFI_MODE_AP);
    }

    if (mode == WIFI_MODE_STA)
    {
        (void)esp_wifi_disconnect();
        return esp_wifi_stop();
    }

    return ESP_OK;
}
