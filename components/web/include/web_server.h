#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t web_server_start(void);
void web_server_stop(void);
bool web_server_is_running(void);
void web_server_mark_scan_start(void);
void web_server_mark_scan_done(void);

#ifdef __cplusplus
}
#endif

#endif // WEB_SERVER_H
