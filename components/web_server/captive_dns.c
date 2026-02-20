#include <string.h>
#include <stdint.h>

#include "esp_log.h"
#include "lwip/udp.h"
#include "lwip/ip4_addr.h"

static const char *TAG = "captive_dns";

static struct udp_pcb *s_dns_pcb = NULL;
static ip4_addr_t s_dns_ip;

#pragma pack(push, 1)
typedef struct {
    uint16_t id;
    uint16_t flags;
    uint16_t qdcount;
    uint16_t ancount;
    uint16_t nscount;
    uint16_t arcount;
} dns_hdr_t;
#pragma pack(pop)

static void dns_send_response(struct udp_pcb *pcb, const ip_addr_t *addr, uint16_t port,
                              const uint8_t *req, size_t req_len)
{
    if (req_len < sizeof(dns_hdr_t)) {
        return;
    }

    const dns_hdr_t *hdr = (const dns_hdr_t *)req;
    if (hdr->qdcount == 0) {
        return;
    }

    // Find end of QNAME
    size_t offset = sizeof(dns_hdr_t);
    while (offset < req_len && req[offset] != 0) {
        offset += (size_t)req[offset] + 1;
    }
    if (offset + 5 > req_len) {
        return;
    }
    offset += 1; // null terminator
    const size_t qname_len = offset - sizeof(dns_hdr_t);

    // QTYPE + QCLASS
    const size_t question_len = qname_len + 4;
    if (sizeof(dns_hdr_t) + question_len > req_len) {
        return;
    }

    uint8_t buf[512];
    size_t pos = 0;

    dns_hdr_t resp = {
        .id = hdr->id,
        .flags = htons(0x8180),
        .qdcount = htons(1),
        .ancount = htons(1),
        .nscount = 0,
        .arcount = 0,
    };

    memcpy(buf + pos, &resp, sizeof(resp));
    pos += sizeof(resp);

    memcpy(buf + pos, req + sizeof(dns_hdr_t), question_len);
    pos += question_len;

    // Answer: name pointer to 0x0C
    buf[pos++] = 0xC0;
    buf[pos++] = 0x0C;
    buf[pos++] = 0x00; // TYPE A
    buf[pos++] = 0x01;
    buf[pos++] = 0x00; // CLASS IN
    buf[pos++] = 0x01;
    buf[pos++] = 0x00; // TTL 0
    buf[pos++] = 0x00;
    buf[pos++] = 0x00;
    buf[pos++] = 0x00;
    buf[pos++] = 0x00;
    buf[pos++] = 0x00; // RDLENGTH 4
    buf[pos++] = 0x04;

    buf[pos++] = ip4_addr1(&s_dns_ip);
    buf[pos++] = ip4_addr2(&s_dns_ip);
    buf[pos++] = ip4_addr3(&s_dns_ip);
    buf[pos++] = ip4_addr4(&s_dns_ip);

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, pos, PBUF_RAM);
    if (!p) {
        return;
    }
    memcpy(p->payload, buf, pos);
    udp_sendto(pcb, p, addr, port);
    pbuf_free(p);
}

static void dns_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                     const ip_addr_t *addr, u16_t port)
{
    (void)arg;
    if (!p) {
        return;
    }

    dns_send_response(pcb, addr, port, (const uint8_t *)p->payload, (size_t)p->len);
    pbuf_free(p);
}

void captive_dns_start(void)
{
    if (s_dns_pcb) {
        return;
    }

    IP4_ADDR(&s_dns_ip, 192, 168, 4, 1);

    s_dns_pcb = udp_new();
    if (!s_dns_pcb) {
        ESP_LOGE(TAG, "udp_new failed");
        return;
    }

    if (udp_bind(s_dns_pcb, IP_ADDR_ANY, 53) != ERR_OK) {
        ESP_LOGE(TAG, "udp_bind failed");
        udp_remove(s_dns_pcb);
        s_dns_pcb = NULL;
        return;
    }

    udp_recv(s_dns_pcb, dns_recv, NULL);
    ESP_LOGI(TAG, "Captive DNS started (192.168.4.1)");
}

void captive_dns_stop(void)
{
    if (!s_dns_pcb) {
        return;
    }
    udp_remove(s_dns_pcb);
    s_dns_pcb = NULL;
    ESP_LOGI(TAG, "Captive DNS stopped");
}
