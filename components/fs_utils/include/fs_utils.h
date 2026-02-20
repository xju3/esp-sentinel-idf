#ifndef FS_UTILS_H
#define FS_UTILS_H

#include <stdbool.h>
#include <stddef.h>

#include "cJSON.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t fsu_mount_storage(bool format_if_mount_failed);
esp_err_t fsu_mount_user(bool format_if_mount_failed);
bool fsu_is_storage_mounted(void);
bool fsu_is_user_mounted(void);

bool fsu_file_exists(const char *path);
char *fsu_read_file_alloc(const char *path, size_t *out_len);
esp_err_t fsu_write_file(const char *path, const char *data, size_t len);

typedef void (*fsu_json_parser_t)(cJSON *root, void *ctx);
esp_err_t fsu_parse_json(const char *path, fsu_json_parser_t parser, void *ctx);

#ifdef __cplusplus
}
#endif

#endif // FS_UTILS_H
