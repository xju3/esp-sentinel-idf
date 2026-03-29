#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "logger.h"

// 声明 captive DNS 函数（在 captive_dns.c 中定义）
void captive_dns_start(void);
void captive_dns_stop(void);

// 静态变量跟踪事件循环是否已初始化
static bool s_event_loop_initialized = false;


// AP 参数定义
#define SENTINEL_WIFI_PASS "12345678"
#define SENTINEL_MAX_CONN 4
#define DEFAULT_SSID_PREFIX "AP_SENTINEL_CONFIG_"

static void build_ap_ssid(char *out, size_t out_len)
{
    if (!out || out_len == 0)
    {
        return;
    }
#if defined(DEVICE_ID)
    if (DEVICE_ID[0] != '\0')
    {
        snprintf(out, out_len, "%s", DEVICE_ID);
        return;
    }
#endif
    uint32_t rnd = esp_random() % 1000;
    snprintf(out, out_len, "%s%03u", DEFAULT_SSID_PREFIX, (unsigned)rnd);
}

volatile bool s_scan_done = false;
volatile bool s_scan_started = false;

static void wifi_event_handler_internal(void *arg, esp_event_base_t event_base,
                                        int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE)
    {
        s_scan_done = true;
    }
}

// 公共的事件处理函数包装
void wifi_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    wifi_event_handler_internal(arg, event_base, event_id, event_data);
}

// 公共的 WiFi 初始化函数
esp_err_t wifi_common_init(bool create_ap, bool create_sta)
{
    // 1. 初始化底层 TCP/IP 堆栈
    ESP_ERROR_CHECK(esp_netif_init());
    
    // 只在第一次调用时创建默认事件循环
    if (!s_event_loop_initialized) {
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        s_event_loop_initialized = true;
    }
    
    if (create_ap) {
        esp_netif_create_default_wifi_ap();
    }
    if (create_sta) {
        esp_netif_create_default_wifi_sta();
    }
    
    // 注册 WiFi 事件处理，包括扫描完成事件
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    // 2. 初始化 Wi-Fi 驱动
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    return esp_wifi_init(&cfg);
}

void wifi_init_softap(void)
{
    // 恢复默认状态.
    esp_wifi_stop();
    esp_wifi_deinit();
    
    // xTaskDelayUntil(10, 100);

    // 使用公共初始化函数
    ESP_ERROR_CHECK(wifi_common_init(true, true));

    // 3. 配置 AP 参数
    wifi_config_t wifi_config = {
        .ap = {
            .channel = 1,
            .max_connection = SENTINEL_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    // 设置 SSID
    char ssid_buf[33] = {0};
    build_ap_ssid(ssid_buf, sizeof(ssid_buf));
    strncpy((char *)wifi_config.ap.ssid, ssid_buf, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen((const char *)wifi_config.ap.ssid);

    // 设置密码
    strncpy((char *)wifi_config.ap.password, SENTINEL_WIFI_PASS, sizeof(wifi_config.ap.password));

    // 如果密码为空，使用开放网络
    if (strlen(SENTINEL_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    // 4. 启动 Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    LOG_INFOF("SoftAP 启动成功! SSID:%s password:%s",
              (const char *)wifi_config.ap.ssid, SENTINEL_WIFI_PASS);

    // 启动 Captive Portal DNS 服务
    captive_dns_start();
}




// 扫描热点.
esp_err_t scan_wifi(void) {
    wifi_scan_config_t scan_cfg = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active = {
            .min = 100,    // 最小扫描时间 100ms
            .max = 600     // 最大扫描时间 300ms
        }
    };
    return  esp_wifi_scan_start(&scan_cfg, false);
}
