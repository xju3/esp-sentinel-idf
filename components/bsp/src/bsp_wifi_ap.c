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
#include "startup_gate.h"

// 声明 captive DNS 函数（在 captive_dns.c 中定义）
void captive_dns_start(void);
void captive_dns_stop(void);

// 静态变量跟踪事件循环是否已初始化
static bool s_event_loop_initialized = false;
static bool s_wifi_initialized = false;
static bool s_wifi_started = false;
static bool s_ap_netif_created = false;
static bool s_sta_netif_created = false;
static bool s_wifi_common_handler_registered = false;


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
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        LOG_INFOF("AP client connected, aid=%d", event ? event->aid : -1);
        startup_gate_mark_ap_client_connected();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        LOG_INFOF("AP client disconnected, aid=%d", event ? event->aid : -1);
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
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }
    
    // 只在第一次调用时创建默认事件循环
    if (!s_event_loop_initialized)
    {
        err = esp_event_loop_create_default();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        {
            return err;
        }
        s_event_loop_initialized = true;
    }
    
    if (create_ap && !s_ap_netif_created)
    {
        esp_netif_create_default_wifi_ap();
        s_ap_netif_created = true;
    }
    if (create_sta && !s_sta_netif_created)
    {
        esp_netif_create_default_wifi_sta();
        s_sta_netif_created = true;
    }
    
    // 注册 WiFi 事件处理，包括扫描完成事件
    if (!s_wifi_common_handler_registered)
    {
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
        s_wifi_common_handler_registered = true;
    }

    // 2. 初始化 Wi-Fi 驱动
    if (!s_wifi_initialized)
    {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        err = esp_wifi_init(&cfg);
        if (err != ESP_OK)
        {
            return err;
        }
        s_wifi_initialized = true;
    }

    return ESP_OK;
}

bool wifi_common_is_started(void)
{
    return s_wifi_started;
}

void wifi_common_mark_started(bool started)
{
    s_wifi_started = started;
}

void wifi_init_softap(void)
{
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
    if (!wifi_common_is_started())
    {
        ESP_ERROR_CHECK(esp_wifi_start());
        wifi_common_mark_started(true);
    }

    LOG_INFOF("SoftAP 启动成功! SSID:%s password:%s",
              (const char *)wifi_config.ap.ssid, SENTINEL_WIFI_PASS);

    // 启动 Captive Portal DNS 服务
    captive_dns_start();
}

esp_err_t wifi_stop_softap(void)
{
    captive_dns_stop();

    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_err_t err = esp_wifi_get_mode(&mode);
    if (err != ESP_OK)
    {
        return err;
    }

    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA || mode == WIFI_MODE_STA)
    {
        (void)esp_wifi_disconnect();

        err = esp_wifi_stop();
        if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED)
        {
            return err;
        }
        wifi_common_mark_started(false);

        err = esp_wifi_set_mode(WIFI_MODE_NULL);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    return ESP_OK;
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
