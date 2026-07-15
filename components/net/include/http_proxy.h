#ifndef HTTP_PROXY_H_
#define HTTP_PROXY_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 通过4G发起HTTP GET请求并返回响应
 * @param url 请求的完整 URL
 * @param out_response 输出指针，成功后需手动 free(*out_response)
 */
esp_err_t http_proxy_get(const char *url, char **out_response);

/**
 * @brief 通过4G发起HTTP POST请求上传JSON
 * @param url 请求的完整 URL
 * @param payload JSON 文本
 * @param out_response 输出指针，成功后需手动 free(*out_response)
 */
esp_err_t http_proxy_post_json(const char *url, const char *payload, char **out_response);

#ifdef __cplusplus
}
#endif
#endif /* HTTP_PROXY_H_ */
