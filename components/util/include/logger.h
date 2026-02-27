#ifndef LOGGER_H
#define LOGGER_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LOGGER_LEVEL_DEBUG = 0,
    LOGGER_LEVEL_INFO  = 1,
    LOGGER_LEVEL_WARN  = 2,
    LOGGER_LEVEL_ERROR = 3,
} logger_level_t;

void logger_init(void);
void logger_set_level(logger_level_t level);
logger_level_t logger_get_level(void);
bool logger_is_enabled(void);
size_t logger_log(logger_level_t level, const char *file, int line, const char *msg);
size_t logger_logf(logger_level_t level, const char *file, int line, const char *fmt, ...);

#if CONFIG_LOGGER_ENABLE
#define LOG_DEBUG(msg)    logger_log(LOGGER_LEVEL_DEBUG, __FILE__, __LINE__, msg)
#define LOG_INFO(msg)     logger_log(LOGGER_LEVEL_INFO,  __FILE__, __LINE__, msg)
#define LOG_WARN(msg)     logger_log(LOGGER_LEVEL_WARN,  __FILE__, __LINE__, msg)
#define LOG_ERROR(msg)    logger_log(LOGGER_LEVEL_ERROR, __FILE__, __LINE__, msg)

#define LOG_DEBUGF(...)   logger_logf(LOGGER_LEVEL_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_INFOF(...)    logger_logf(LOGGER_LEVEL_INFO,  __FILE__, __LINE__, __VA_ARGS__)
#define LOG_WARNF(...)    logger_logf(LOGGER_LEVEL_WARN,  __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERRORF(...)   logger_logf(LOGGER_LEVEL_ERROR, __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOG_DEBUG(msg)    do { (void)(msg); } while (0)
#define LOG_INFO(msg)     do { (void)(msg); } while (0)
#define LOG_WARN(msg)     do { (void)(msg); } while (0)
#define LOG_ERROR(msg)    do { (void)(msg); } while (0)
#define LOG_DEBUGF(...)   do { } while (0)
#define LOG_INFOF(...)    do { } while (0)
#define LOG_WARNF(...)    do { } while (0)
#define LOG_ERRORF(...)   do { } while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif // LOGGER_H
