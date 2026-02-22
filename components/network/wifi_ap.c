#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "network_manager.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "logger.h"

static TaskHandle_t s_dns_task_handle = NULL;

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

static void dns_server_task(void *pvParameters)
{
    char rx_buffer[512];
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(53);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        LOG_ERRORF("Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    if (bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0)
    {
        LOG_ERRORF("Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    LOG_INFO("DNS Server started for Captive Portal");

    while (1)
    {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);

        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
        if (len < 0)
        {
            LOG_ERRORF("recvfrom failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (len > 12 && (len + 16 <= sizeof(rx_buffer)))
        {
            // DNS Header: ID(2), Flags(2), QDCOUNT(2), ANCOUNT(2), NSCOUNT(2), ARCOUNT(2)
            rx_buffer[2] = 0x81; // QR=1, Opcode=0, AA=1, TC=0, RD=1
            rx_buffer[3] = 0x80; // RA=1, Z=0, RCODE=0
            rx_buffer[6] = 0x00;
            rx_buffer[7] = 0x01; // ANCOUNT = 1
            rx_buffer[8] = 0x00;
            rx_buffer[9] = 0x00; // NSCOUNT = 0
            rx_buffer[10] = 0x00;
            rx_buffer[11] = 0x00; // ARCOUNT = 0

            int ptr = len;
            // Answer: Name(ptr to header), Type(A), Class(IN), TTL, Len, IP
            // 192.168.4.1 is the default SoftAP IP
            uint8_t answer[] = {0xC0, 0x0C, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x04, 192, 168, 4, 1};
            memcpy(rx_buffer + ptr, answer, sizeof(answer));
            sendto(sock, rx_buffer, ptr + sizeof(answer), 0, (struct sockaddr *)&source_addr, socklen);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    close(sock);
    s_dns_task_handle = NULL;
    vTaskDelete(NULL);
}

void captive_dns_start(void)
{
    if (s_dns_task_handle)
        return;
    xTaskCreate(dns_server_task, "dns_server", 4096, NULL, 5, &s_dns_task_handle);
}

void captive_dns_stop(void)
{
    if (s_dns_task_handle)
    {
        vTaskDelete(s_dns_task_handle);
        s_dns_task_handle = NULL;
    }
}

static volatile bool s_scan_done = false;
static volatile bool s_scan_started = false;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE)
    {
        s_scan_done = true;
    }
}

void wifi_init_softap(void)
{
    // 1. 初始化底层 TCP/IP 堆栈
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();
    // init 里注册一次：
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

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

    scan_wifi();
    // 启动 Captive Portal DNS 服务
    captive_dns_start();
}



// 扫描热点.
esp_err_t scan_wifi(esp_err_t) {
    wifi_scan_config_t scan_cfg = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_PASSIVE,
        .scan_time.passive = 1500, //TJU: 增加扫描时间, 否则返回的热点数很少
    };
    return  esp_wifi_scan_start(&scan_cfg, false);
}
