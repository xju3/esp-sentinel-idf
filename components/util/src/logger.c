#include "logger.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"

#ifndef LOG_BUFFER_SIZE
#define LOG_BUFFER_SIZE 256
#endif

static const char *kTag = CONFIG_LOGGER_TAG;
static logger_level_t s_level = CONFIG_LOGGER_LEVEL;
static bool s_enabled = CONFIG_LOGGER_ENABLE;
static bool s_inited = false;

static const char *base_name(const char *path)
{
    if (!path) return "?";
    const char *slash = strrchr(path, '/');
    return slash ? slash + 1 : path;
}

static esp_log_level_t to_esp(logger_level_t lvl)
{
    switch (lvl) {
    case LOGGER_LEVEL_DEBUG: return ESP_LOG_DEBUG;
    case LOGGER_LEVEL_INFO:  return ESP_LOG_INFO;
    case LOGGER_LEVEL_WARN:  return ESP_LOG_WARN;
    case LOGGER_LEVEL_ERROR: return ESP_LOG_ERROR;
    default: return ESP_LOG_INFO;
    }
}

void logger_init(void)
{
    if (s_inited) return;
    esp_log_level_set(kTag, ESP_LOG_VERBOSE);
    s_inited = true;
}

void logger_set_level(logger_level_t level)
{
    s_level = level;
}

logger_level_t logger_get_level(void)
{
    return s_level;
}

bool logger_is_enabled(void)
{
    return s_enabled;
}

size_t logger_log(logger_level_t level, const char *file, int line, const char *msg)
{
#if CONFIG_LOGGER_ENABLE
    if (!s_enabled || level < s_level) return 0;
    logger_init();
    esp_log_level_t esp_lvl = to_esp(level);
    const char *base = base_name(file);
    char buf[LOG_BUFFER_SIZE];
    int len = snprintf(buf, sizeof(buf), "[%s:%d] %s", base, line, msg ? msg : "");
    if (len < 0) return 0;
    ESP_LOG_LEVEL(esp_lvl, kTag, "%s", buf);
    return (size_t)len;
#else
    (void)level; (void)file; (void)line; (void)msg; return 0;
#endif
}

size_t logger_logf(logger_level_t level, const char *file, int line, const char *fmt, ...)
{
#if CONFIG_LOGGER_ENABLE
    if (!s_enabled || level < s_level) return 0;
    logger_init();
    va_list args;
    va_start(args, fmt);
    char msgbuf[LOG_BUFFER_SIZE];
    int len = vsnprintf(msgbuf, sizeof(msgbuf), fmt, args);
    va_end(args);
    if (len < 0) return 0;

    esp_log_level_t esp_lvl = to_esp(level);
    const char *base = base_name(file);
    char outbuf[LOG_BUFFER_SIZE];
    int out_len = snprintf(outbuf, sizeof(outbuf), "[%s:%d] %s", base, line, msgbuf);
    if (out_len < 0) return 0;
    ESP_LOG_LEVEL(esp_lvl, kTag, "%s", outbuf);
    return (size_t)out_len;
#else
    (void)level; (void)file; (void)line; (void)fmt; return 0;
#endif
}
