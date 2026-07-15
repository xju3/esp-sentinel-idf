#include "http_proxy.h"

#include "drv_4g.h"

esp_err_t http_proxy_get(const char *url, char **out_response)
{
    if (!url || !out_response) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_response = NULL;
    return bsp_4g_http_get(url, out_response);
}

esp_err_t http_proxy_post_json(const char *url,
                               const char *payload,
                               char **out_response)
{
    if (!url || !payload) {
        return ESP_ERR_INVALID_ARG;
    }
    if (out_response) {
        *out_response = NULL;
    }
    return bsp_4g_http_post_json(url, payload, out_response);
}
