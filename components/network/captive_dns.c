#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "logger.h"

static const char *TAG = "captive_dns";
static int s_dns_socket = -1;
static TaskHandle_t s_dns_task_handle = NULL;

typedef struct __attribute__((packed)) {
    uint16_t id;
    uint16_t flags;
    uint16_t qdcount;
    uint16_t ancount;
    uint16_t nscount;
    uint16_t arcount;
} dns_hdr_t;

static void dns_server_task(void *pvParameters)
{
    uint8_t rx_buffer[512];
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(53);

    s_dns_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_dns_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int err = bind(s_dns_socket, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(s_dns_socket);
        s_dns_socket = -1;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Captive DNS Server started on port 53");

    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    while (1) {
        int len = recvfrom(s_dns_socket, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            break;
        }

        if (len < sizeof(dns_hdr_t)) {
            continue;
        }

        dns_hdr_t *hdr = (dns_hdr_t *)rx_buffer;
        // Only respond to queries (QR=0)
        if ((hdr->flags & htons(0x8000)) != 0) {
            continue;
        }

        // Prepare response header
        hdr->flags = htons(0x8180); // Standard response, No error
        hdr->ancount = htons(1);    // 1 Answer
        hdr->nscount = 0;
        hdr->arcount = 0;

        // Find end of QNAME
        int qname_len = 0;
        uint8_t *p = rx_buffer + sizeof(dns_hdr_t);
        while ((p - rx_buffer) < len && *p != 0) {
            p += (*p) + 1;
        }
        if ((p - rx_buffer) >= len) continue;
        p++; // Skip null
        p += 4; // Skip QTYPE and QCLASS

        qname_len = p - rx_buffer;
        if (qname_len > sizeof(rx_buffer) - 16) continue;

        // Append Answer (A Record pointing to 192.168.4.1)
        // Name pointer: 0xC00C
        *p++ = 0xC0;
        *p++ = 0x0C;
        // Type A
        *p++ = 0x00;
        *p++ = 0x01;
        // Class IN
        *p++ = 0x00;
        *p++ = 0x01;
        // TTL
        *p++ = 0x00;
        *p++ = 0x00;
        *p++ = 0x00;
        *p++ = 0x3C;
        // Data Len
        *p++ = 0x00;
        *p++ = 0x04;
        // IP
        *p++ = 192;
        *p++ = 168;
        *p++ = 4;
        *p++ = 1;

        int resp_len = p - rx_buffer;
        sendto(s_dns_socket, rx_buffer, resp_len, 0, (struct sockaddr *)&source_addr, socklen);
    }

    if (s_dns_socket != -1) {
        close(s_dns_socket);
        s_dns_socket = -1;
    }
    s_dns_task_handle = NULL;
    vTaskDelete(NULL);
}

void captive_dns_start(void)
{
    if (s_dns_task_handle)  {
        LOG_WARN("DNS server already running");
        return;
    }
    LOG_DEBUG("starting dns server...");
    xTaskCreate(dns_server_task, "dns_server", 4096, NULL, 5, &s_dns_task_handle);
}

void captive_dns_stop(void)
{
    if (s_dns_task_handle) {
        if (s_dns_socket != -1) {
            shutdown(s_dns_socket, 0);
            close(s_dns_socket);
            s_dns_socket = -1;
        }
        vTaskDelete(s_dns_task_handle);
        s_dns_task_handle = NULL;
    }
}
