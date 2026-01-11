#include "logger.h"
#include "usart.h"
#include "stdarg.h"
#include "string.h"
#include <stdio.h>
#include <inttypes.h>
#include "FreeRTOS.h"  
#include "task.h" 

// 匹配FreeRTOS官方声明，消除类型冲突
size_t xPortGetMinimumEverFreeHeapSize( void ) PRIVILEGED_FUNCTION;

// 外部声明UART句柄
extern UART_HandleTypeDef huart1;

// 日志队列和任务句柄（静态隐藏）
static osMessageQueueId_t log_queue_id = NULL;
static osThreadId_t logger_task_id = NULL;

// 堆监控定时器（单独控制堆信息打印频率，不阻塞日志）
static TickType_t last_heap_print_tick = 0;
#define HEAP_PRINT_INTERVAL_MS 10000 // 堆信息打印间隔：10秒

// 日志任务属性配置
const osThreadAttr_t LoggerTask_attributes = {
    .name = "LGTask",
    .stack_size = 2048,
    .priority = (osPriority_t) osPriorityLow1,
};

// 日志队列属性配置
const osMessageQueueAttr_t LogQueue_attributes = {
    .name = "LogQueue"
};
// static char task_list[512]; // 改为静态，不占栈！
void print_heap_info(void) {
    if(log_queue_id == NULL) return;
    size_t free_heap = xPortGetFreeHeapSize();
    // size_t min_free_heap = xPortGetMinimumEverFreeHeapSize();
    size_t heap_usage = (configTOTAL_HEAP_SIZE - free_heap) * 100 / configTOTAL_HEAP_SIZE;
    
    // LOG_INFO("Free Heap: %u B, Min Free Heap: %u B, Usage: %u%%", 
    //          (unsigned int)free_heap, (unsigned int)min_free_heap, (unsigned int)heap_usage);
    
    // vTaskList(task_list);
    // HAL_UART_Transmit(&huart1, (uint8_t*)task_list, strlen(task_list), 50);
    if(heap_usage > 80) {
        LOG_ERROR("Heap Usage High: %u%%! Risk of Out-of-Memory!", (unsigned int)heap_usage);
    }
}

// 日志任务核心函数（彻底解耦：堆监控+日志处理，日志零延迟）
void LoggerTask(void *argument) {
    log_message_t log_msg;
    osStatus_t status;
    last_heap_print_tick = xTaskGetTickCount();
    static char output_buffer[LOG_MAX_MSG_LEN + 64];
    for(;;) {
        // 1. 检查是否到堆信息打印时间（非阻塞，不影响日志）
        if(xTaskGetTickCount() - last_heap_print_tick >= pdMS_TO_TICKS(HEAP_PRINT_INTERVAL_MS)) {
            print_heap_info();
            last_heap_print_tick = xTaskGetTickCount(); // 更新时间戳
        }
        
        // 2. 非阻塞读取日志队列（超时10ms，兼顾实时性和CPU占用）
        status = osMessageQueueGet(log_queue_id, &log_msg, NULL, pdMS_TO_TICKS(10));
        if(status == osOK) {
            char *level_str;
            switch(log_msg.level) {
                case LOG_LEVEL_DEBUG: level_str = "DEBUG"; break;
                case LOG_LEVEL_INFO:  level_str = "INFO "; break;
                case LOG_LEVEL_WARN:  level_str = "WARN "; break;
                case LOG_LEVEL_ERROR: level_str = "ERROR"; break;
                default:              level_str = "UNKWN"; break;
            }
            
            uint32_t sec = log_msg.timestamp / 1000;    
            uint32_t ms_remain = log_msg.timestamp % 1000;

           snprintf(output_buffer, sizeof(output_buffer), 
                    "[%" PRIu32 ".%03" PRIu32 " s] [%s] %s\r\n", 
                    sec, ms_remain, level_str, log_msg.message);
            
            // 分段发送串口
            uint16_t send_len = strlen(output_buffer);
            if(send_len > 0) {
                for(uint16_t i=0; i<send_len; i+=64) {
                    uint16_t chunk_len = (send_len - i) > 64 ? 64 : (send_len - i);
                    HAL_UART_Transmit(&huart1, (uint8_t*)(output_buffer+i), chunk_len, 50);
                }
            }
        }
        // 无日志时，任务休眠10ms，降低CPU占用（而非10秒）
    }
}

// 日志系统初始化函数
void logger_init(void) {
    log_queue_id = osMessageQueueNew(32, sizeof(log_message_t), &LogQueue_attributes);
    if(log_queue_id == NULL) {
        char err_msg[] = "[0.000 s] [ERROR] Log Queue Create Failed!\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)err_msg, strlen(err_msg), 100);
        return;
    }
    
    logger_task_id = osThreadNew(LoggerTask, NULL, &LoggerTask_attributes);
    if(logger_task_id == NULL) {
        char err_msg[] = "[0.000 s] [ERROR] Log Task Create Failed!\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)err_msg, strlen(err_msg), 100);
    }
}

// 日志输出核心接口
void logger_log(log_level_t level, const char *fmt, ...) {
    if(log_queue_id == NULL) return;
    
    log_message_t log_msg;
    log_msg.level = level;
    log_msg.timestamp = osKernelGetTickCount(); 
    
    va_list args;
    va_start(args, fmt);
    vsnprintf(log_msg.message, LOG_MAX_MSG_LEN, fmt, args);
    va_end(args);
    
    // 修复：入队超时缩短为1ms，避免阻塞
    osStatus_t status = osMessageQueuePut(log_queue_id, &log_msg, 0, pdMS_TO_TICKS(1));
    if(status != osOK) {
        // 修复：串口发送超时缩短为10ms，避免卡死
        char err_msg[] = "[ERROR] Log Queue Full!\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)err_msg, strlen(err_msg), 10);
    }
}