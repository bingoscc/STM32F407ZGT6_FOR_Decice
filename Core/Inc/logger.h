#ifndef LOGGER_H
#define LOGGER_H

#include "stdint.h"
#include "cmsis_os2.h"

// 日志级别枚举
typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR
} log_level_t;

// 日志消息结构
#define LOG_MAX_MSG_LEN 128
typedef struct {
    log_level_t level;
    char message[LOG_MAX_MSG_LEN];
    uint32_t timestamp;
} log_message_t;

// 初始化日志系统
void logger_init(void);

// 日志输出函数
void logger_log(log_level_t level, const char *fmt, ...);

// 便捷宏定义
#define LOG_DEBUG(fmt, ...) logger_log(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  logger_log(LOG_LEVEL_INFO, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  logger_log(LOG_LEVEL_WARN, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) logger_log(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)

#endif