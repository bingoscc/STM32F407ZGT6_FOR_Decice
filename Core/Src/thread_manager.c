#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "thread_manager.h"
#include "logger.h"
#include <string.h>
#include "ext_dev_ctrl.h"
#include "tim.h"

// 外部函数声明
extern void StartMotorTask(void *argument);
extern void StartGPIOMonitorTask(void *argument);
extern void UDP_LED_Flash_Task(void *arg);
extern void StartETHTask(void *argument);
extern void StartFeedingTask(void *argument);

// 线程句柄
static osThreadId_t motor_task_handle = NULL;
static osThreadId_t gpio_monitor_task_handle = NULL;
static osThreadId_t udp_led_task_handle = NULL;
static osThreadId_t eth_task_handle = NULL;
static osThreadId_t Feeding_task_handle = NULL;

// 线程管理任务句柄和消息队列
static osThreadId_t thread_manager_task_handle = NULL;
static osMessageQueueId_t control_queue_id = NULL;

// 全局变量定义
static uint16_t g_target_angle = 0;
static motor_direction_t g_motor_dir = MOTOR_DIR_STOP;

// 外部声明
extern volatile uint8_t g_motor_force_stop_flag;
extern volatile uint8_t g_feeding_force_stop_flag;
extern QueueHandle_t g_motor_action_queue;
extern QueueHandle_t g_feeding_action_queue;

// 任务属性配置
// 电机任务
const osThreadAttr_t MotorTask_attributes = {
    .name = "MTTask",
    .stack_size = 8192,
    .priority = (osPriority_t) osPriorityNormal,
};

// GPIO任务
const osThreadAttr_t GPIOMonitorTask_attributes = {
    .name = "GPTask",
    .stack_size = 2048,
    .priority = (osPriority_t) osPriorityLow,
};

// ETH任务
const osThreadAttr_t ETHTask_attributes = {
    .name = "ETTask",
    .stack_size = 12288,
    .priority = (osPriority_t) osPriorityLow,
};

// LED闪烁指示任务
const osThreadAttr_t UDP_LED_Task_Attr = {
    .name = "LETask",
    .stack_size = 512,
    .priority = (osPriority_t) osPriorityLow3,
};

// 线程管理任务
const osThreadAttr_t ThreadManagerTask_attributes = {
    .name = "MgTask",
    .stack_size = 2048,
    .priority = (osPriority_t) osPriorityLow4,
};

// 送料任务
const osThreadAttr_t FeedingTask_attributes = {
    .name = "FdTask",
    .stack_size = 8192,
    .priority = (osPriority_t) osPriorityNormal,
};

// 接口函数
uint16_t motor_get_target_angle(void) {
    return g_target_angle;
}

void motor_set_target_angle(uint16_t angle) {
    #if THREAD_MANAGER_CHECK_EN
    if(angle > 3600) {
        angle = 3600;
        LOG_WARN("Target angle exceed max(3600), limit to 3600");
    }
    #endif
    g_target_angle = angle;
}

motor_direction_t motor_get_direction(void) {
    return g_motor_dir;
}

void motor_set_direction(motor_direction_t dir) {
    g_motor_dir = dir;
}

// ==================== 线程管理核心 ====================
void ThreadManagerTask(void *argument) {
    thread_control_msg_t msg;   // 任务控制消息
    osStatus_t status;          // 消息队列状态
    const uint32_t QUEUE_GET_TIMEOUT = pdMS_TO_TICKS(10); // 消息队列超时时间（10ms）
    
    LOG_INFO("ThreadManagerTask: Started successfully");
    
    for(;;) {
        // 1. 等待任务控制消息
        status = osMessageQueueGet(control_queue_id, &msg, NULL, QUEUE_GET_TIMEOUT);
        if(status == osOK) {
            osThreadId_t *target_handle = NULL;     // 初始化目标句柄
            osThreadFunc_t task_func = NULL;        // 初始化任务函数
            const osThreadAttr_t *task_attr = NULL; // 初始化线程属性
            
            // ========== 电机指令处理 ==========
            if(msg.operation == THREAD_OP_MOTOR_CTRL || msg.operation == THREAD_OP_MOTOR_FORCE_STOP) {
                // 1. 确保电机任务已创建
                osThreadId_t motor_handle = thread_manager_get_thread_handle(THREAD_TYPE_MOTOR);
                if(motor_handle == NULL) {
                    LOG_WARN("Motor task not exists, create first");
                    thread_manager_send_cmd(THREAD_OP_CREATE, THREAD_TYPE_MOTOR, 0, 0, NULL);
                    osDelay(10);
                    motor_handle = thread_manager_get_thread_handle(THREAD_TYPE_MOTOR);
                    if(motor_handle == NULL) {
                        LOG_ERROR("Create motor task failed, skip motor cmd");
                        continue;
                    }
                }
                
                // 2. 处理电机控制指令
                if(msg.operation == THREAD_OP_MOTOR_CTRL) {

                    uint16_t target_angle = (uint16_t)msg.params.raw[0];       // raw[0]传递角度
                    motor_direction_t motor_dir = (motor_direction_t)(msg.params.raw[1] & 0xFF);  // raw[1]低8位传递方向
                    
                    // 更新全局变量
                    motor_set_target_angle(target_angle);
                    motor_set_direction(motor_dir);
                    
                    LOG_INFO("ThreadMgr: Motor ctrl cmd - angle = %d, dir = %d", 
                             motor_get_target_angle(), motor_get_direction());
                    
                    // 转发到电机动作直接控制队列
                    if(g_motor_action_queue != NULL) {
                        ActionMsg_t action_msg = {0};
                        action_msg.action_type = ACTION_MOTOR_CTRL;
                        action_msg.params.motor_params.target_angle = motor_get_target_angle();
                        action_msg.params.motor_params.motor_dir = motor_get_direction();
                        
                        BaseType_t xStatus = xQueueSend(g_motor_action_queue, &action_msg, 0);
                        if(xStatus == pdPASS) {
                            LOG_DEBUG("ThreadMgr: Motor cmd forwarded to FreeRTOS queue");
                        } else {
                            LOG_ERROR("ThreadMgr: Motor queue send failed (full)");
                        }
                    } else {
                        LOG_ERROR("g_motor_action_queue is NULL, skip queue send");
                    }
                }
                
                // 3. 处理电机强制停止指令
                else if(msg.operation == THREAD_OP_MOTOR_FORCE_STOP) {
                    g_motor_force_stop_flag = 1;
                    LOG_INFO("ThreadMgr: Motor force stop cmd received");
                    
                    // 转发到FreeRTOS原生队列（双重保障）
                    if(g_motor_action_queue != NULL) {
                        ActionMsg_t action_msg = {0};
                        action_msg.action_type = ACTION_MOTOR_FORCE_STOP;
                        
                        BaseType_t xStatus = xQueueSend(g_motor_action_queue, &action_msg, 0);
                        if(xStatus == pdPASS) {
                            LOG_DEBUG("ThreadMgr: Force stop cmd forwarded to FreeRTOS queue");
                        } else {
                            LOG_ERROR("ThreadMgr: Force stop queue send failed (full)");
                        }
                    }
                }
                continue; // 处理完电机指令，跳过后续任务创建逻辑
            }
            
            // ========== 送料指令处理 ==========
            if(msg.operation == THREAD_OP_FEEDING_CTRL || msg.operation == THREAD_OP_FEEDING_FORCE_STOP) {
                // 确保送料任务已创建
                osThreadId_t feeding_handle = thread_manager_get_thread_handle(THREAD_TYPE_Feeding);
                if(feeding_handle == NULL) {
                    LOG_WARN("Feeding task not exists, create first");
                    thread_manager_send_cmd(THREAD_OP_CREATE, THREAD_TYPE_Feeding, 0, 0, NULL);
                    osDelay(10);
                    feeding_handle = thread_manager_get_thread_handle(THREAD_TYPE_Feeding);
                    if(feeding_handle == NULL) {
                        LOG_ERROR("Create feeding task failed, skip feeding cmd");
                        continue;
                    }
                }
                
                // 处理送料控制指令
                if(msg.operation == THREAD_OP_FEEDING_CTRL) {

                    uint32_t target_length = msg.params.raw[0];        // raw[0]传递长度
                    feed_direction_t feeding_dir = (feed_direction_t)(msg.params.raw[1] & 0xFF);  // raw[1]低8位传递方向
                    
                    LOG_INFO("ThreadMgr: Feeding ctrl cmd - length = %d, dir = %d", target_length, feeding_dir);
                    
                    // 转发到送料任务专用队列
                    if(g_feeding_action_queue != NULL) {
                        ActionMsg_t action_msg = {0};
                        action_msg.action_type = ACTION_FEEDING_CTRL;
                        action_msg.params.feeding_params.target_length = target_length;
                        action_msg.params.feeding_params.direction = feeding_dir;
                        
                        BaseType_t xStatus = xQueueSend(g_feeding_action_queue, &action_msg, 0);
                        if(xStatus == pdPASS) {
                            LOG_DEBUG("ThreadMgr: Feeding cmd forwarded to feeding queue");
                        } else {
                            LOG_ERROR("ThreadMgr: Feeding queue send failed (full)");
                        }
                    } else {
                        LOG_ERROR("g_feeding_action_queue is NULL, skip queue send");
                    }
                }
                
                // 处理送料强制停止指令
                else if(msg.operation == THREAD_OP_FEEDING_FORCE_STOP) {
                    g_feeding_force_stop_flag = 1;
                    LOG_INFO("ThreadMgr: Feeding force stop cmd received");
                    
                    // 转发到喂料任务专用队列
                    if(g_feeding_action_queue != NULL) {
                        ActionMsg_t action_msg = {0};
                        action_msg.action_type = ACTION_FEEDING_FORCE_STOP;
                        
                        BaseType_t xStatus = xQueueSend(g_feeding_action_queue, &action_msg, 0);
                        if(xStatus == pdPASS) {
                            LOG_DEBUG("ThreadMgr: Feeding force stop cmd forwarded to feeding queue");
                        } else {
                            LOG_ERROR("ThreadMgr: Feeding stop queue send failed (full)");
                        }
                    }
                }
                continue; // 处理完喂料指令，跳过后续任务创建逻辑
            }
            
            // ========== 外部设备指令处理 ==========
            // 处理TSJ控制指令
            if(msg.operation == THREAD_OP_TSJ_CTRL) {
                LOG_INFO("ThreadMgr: Received TSJ control command");
                ext_dev_tsj_control();
                continue; 
            }
            // 处理CUT控制指令
            if(msg.operation == THREAD_OP_CUT_CTRL) {
                LOG_INFO("ThreadMgr: Received CUT control command");
                ext_dev_cut_control();
                continue; 
            }
            // 处理SAW控制指令
            if(msg.operation == THREAD_OP_SAW_CTRL) {
                LOG_INFO("ThreadMgr: Received SAW control command");
                ext_dev_saw_control();
                continue; 
            }
            // 处理FREE控制指令
            if(msg.operation == THREAD_OP_FREE_CTRL) {
                LOG_INFO("ThreadMgr: Received FREE control command");
                ext_dev_free_control();
                continue; 
            }
            // 处理CLMP控制指令
            if(msg.operation == THREAD_OP_CLMP_CTRL) {
                LOG_INFO("ThreadMgr: Received CLMP control command");
                ext_dev_clmp_control();
                continue; 
            }
            // 处理SAW FREE控制指令
            if(msg.operation == THREAD_OP_SAW_FREE) {
                LOG_INFO("ThreadMgr: Received SAW FREE control command");
                ext_dev_saw_free();
                continue; 
            }
            
            // ========== 基础线程操作（创建/删除/挂起/恢复） ==========
            switch(msg.thread_type) {
                case THREAD_TYPE_MOTOR:
                    target_handle = &motor_task_handle;
                    task_func = StartMotorTask;
                    task_attr = &MotorTask_attributes;
                    break;
                    
                case THREAD_TYPE_GPIO_MONITOR:
                    target_handle = &gpio_monitor_task_handle;
                    task_func = StartGPIOMonitorTask;
                    task_attr = &GPIOMonitorTask_attributes;
                    break;
                    
                case THREAD_TYPE_UDP_LED:
                    target_handle = &udp_led_task_handle;
                    task_func = UDP_LED_Flash_Task;
                    task_attr = &UDP_LED_Task_Attr;
                    break;
                    
                case THREAD_TYPE_ETH:
                    target_handle = &eth_task_handle;
                    task_func = StartETHTask;
                    task_attr = &ETHTask_attributes;
                    break;
                    
                case THREAD_TYPE_Feeding:
                    target_handle = &Feeding_task_handle;
                    task_func = StartFeedingTask;
                    task_attr = &FeedingTask_attributes;
                    break;    
                    
                default:
                    LOG_WARN("Unsupported thread type: %d", msg.thread_type);
                    continue;
            }
            
            // 空指针校验
            if(target_handle == NULL || task_func == NULL || task_attr == NULL) {
                LOG_ERROR("Invalid thread config: type = %d", msg.thread_type);
                continue;
            }
            
            // 执行相应操作
            switch(msg.operation) {
                case THREAD_OP_CREATE:
                    if(*target_handle != NULL) {
                        LOG_WARN("Thread %d exists, deleting old", msg.thread_type);
                        osThreadTerminate(*target_handle);
                        *target_handle = NULL;
                        osDelay(pdMS_TO_TICKS(10));
                    }
                    
                    *target_handle = osThreadNew(task_func, msg.argument, task_attr);
                    if(*target_handle != NULL) {
                        LOG_INFO("Thread created: type = %d, handle = 0x%08X", 
                                msg.thread_type, (uint32_t)*target_handle);
                        
                        // 检查任务栈剩余（仅DEBUG模式）
                        #ifdef DEBUG
                        TaskStatus_t task_status;
                        vTaskGetInfo(*target_handle, &task_status, pdTRUE, eInvalid);
                        LOG_INFO("Thread %d stack remain: %d B", 
                                msg.thread_type, task_status.usStackHighWaterMark);
                        #endif
                    } else {
                        LOG_ERROR("Failed to create thread: type = %d", msg.thread_type);
                    }
                    osDelay(pdMS_TO_TICKS(10));
                    break;
                    
                case THREAD_OP_DELETE:
                    if(*target_handle != NULL) {
                        osThreadTerminate(*target_handle);
                        *target_handle = NULL;
                        LOG_INFO("Thread deleted: type = %d", msg.thread_type);
                        osDelay(pdMS_TO_TICKS(10));
                    } else {
                        LOG_WARN("Thread %d not exists, skip delete", msg.thread_type);
                    }
                    break;
                    
                case THREAD_OP_SUSPEND:
                    if(*target_handle != NULL) {
                        osThreadSuspend(*target_handle);
                        LOG_INFO("Thread suspended: type = %d", msg.thread_type);
                    } else {
                        LOG_WARN("Thread %d not exists, skip suspend", msg.thread_type);
                    }
                    break;
                    
                case THREAD_OP_RESUME:
                    if(*target_handle != NULL) {
                        osThreadResume(*target_handle);
                        LOG_INFO("Thread resumed: type = %d", msg.thread_type);
                    } else {
                        LOG_WARN("Thread %d not exists, skip resume", msg.thread_type);
                    }
                    break;
                    
                default:
                    LOG_ERROR("Unknown thread operation: %d", msg.operation);
                    break;
            }
        } else if(status == osErrorTimeout) {
            // 超时正常，无需处理
            continue;
        } else {
            LOG_ERROR("Queue get failed: status = %d", status);
        }
        
        // 强制释放CPU，避免占用过高
        osDelay(pdMS_TO_TICKS(1));
    }
}

// ==================== 线程管理初始 ====================
void thread_manager_init(void) {
    if(thread_manager_task_handle != NULL) {
        LOG_WARN("Thread manager already initialized");
        return;
    }
    
    // 1. 创建控制队列
    const osMessageQueueAttr_t QueueAttributes = {
        .name = "ThreadCtrlQueue"
    };
    control_queue_id = osMessageQueueNew(32, sizeof(thread_control_msg_t), &QueueAttributes);
    if(control_queue_id == NULL) {
        LOG_ERROR("Failed to create thread control queue");
        return;
    }
    
    // 2. 创建线程管理器任务
    thread_manager_task_handle = osThreadNew(ThreadManagerTask, NULL, &ThreadManagerTask_attributes);
    if(thread_manager_task_handle == NULL) {
        LOG_ERROR("Failed to create thread manager task");
        osMessageQueueDelete(control_queue_id);
        control_queue_id = NULL;
    } else {
        LOG_INFO("Thread manager initialized successfully");
        
        // 3. 检查队列是否已在freertos.c中创建，未创建则创建（避免重复）
        if(g_motor_action_queue == NULL) {
            g_motor_action_queue = xQueueCreate(MOTOR_QUEUE_LENGTH, QUEUE_ITEM_SIZE);
            if(g_motor_action_queue == NULL) {
                LOG_ERROR("Create motor action queue failed");
            } else {
                LOG_INFO("Motor action queue created successfully");
            }
        }
        
        if(g_feeding_action_queue == NULL) {
            g_feeding_action_queue = xQueueCreate(FEEDING_QUEUE_LENGTH, QUEUE_ITEM_SIZE);
            if(g_feeding_action_queue == NULL) {
                LOG_ERROR("Create feeding action queue failed");
            } else {
                LOG_INFO("Feeding action queue created successfully");
            }
        }
    }
}

// ==================== 发送控制命令 ====================
void thread_manager_send_cmd(thread_operation_t operation, thread_type_t thread_type, 
                            uint32_t param1, uint32_t param2, void *argument) {
    if(control_queue_id == NULL) {
        LOG_ERROR("Thread manager not initialized");
        return;
    }
    
    // 参数合法性校验（仅DEBUG模式）
    #if THREAD_MANAGER_CHECK_EN
    if(operation >= THREAD_OP_MAX) {
        LOG_ERROR("Invalid operation: %d (max=%d)", operation, THREAD_OP_MAX-1);
        return;
    }
    if(thread_type >= THREAD_TYPE_MAX) {
        LOG_ERROR("Invalid thread type: %d (max=%d)", thread_type, THREAD_TYPE_MAX-1);
        return;
    }
    #endif
    
    thread_control_msg_t msg = {0};
    msg.operation = operation;
    msg.thread_type = thread_type;
    msg.params.raw[0] = param1;  // 使用raw[0]存储param1
    msg.params.raw[1] = param2;  // 使用raw[1]存储param2
    msg.argument = argument;
    
    const uint32_t QUEUE_PUT_TIMEOUT = pdMS_TO_TICKS(10);
    osStatus_t result = osMessageQueuePut(control_queue_id, &msg, 0, QUEUE_PUT_TIMEOUT);
    
    if(result != osOK) {
        LOG_ERROR("Send cmd failed: op = %d, type = %d, status = %d", 
                 operation, thread_type, result);
    } else {
        LOG_DEBUG("Sent thread cmd: op = %d, type = %d", operation, thread_type);
    }
}

// ==================== 获取线程句柄 ====================
osThreadId_t thread_manager_get_thread_handle(thread_type_t type) {
    osThreadId_t handle = NULL;
    switch(type) {
        case THREAD_TYPE_MOTOR:
            handle = motor_task_handle;
            break;
        case THREAD_TYPE_GPIO_MONITOR:
            handle = gpio_monitor_task_handle;
            break;
        case THREAD_TYPE_UDP_LED:
            handle = udp_led_task_handle;
            break;
        case THREAD_TYPE_ETH:
            handle = eth_task_handle;
            break;
        case THREAD_TYPE_Feeding:
            handle = Feeding_task_handle;
            break;
        default:
            LOG_WARN("Unsupported thread type: %d", type);
            break;
    }
    LOG_DEBUG("Get thread handle: type = %d, handle = 0x%08X", type, (uint32_t)handle);
    return handle;
}

// ==================== 同步创建函数 ====================
osThreadId_t thread_manager_create_thread_sync(thread_type_t thread_type, void *argument) {
    thread_manager_send_cmd(THREAD_OP_CREATE, thread_type, 0, 0, argument);
    uint32_t timeout = 0;
    osThreadId_t handle = NULL;
    while(timeout < 200) {
        handle = thread_manager_get_thread_handle(thread_type);
        if(handle != NULL) {
            break;
        }
        osDelay(1);
        timeout++;
    }
    
    if(timeout >= 200) {
        LOG_ERROR("Create thread sync timeout: type = %d", thread_type);
    }
    
    return handle;
}

// ==================== 打印任务列表 ====================
void thread_manager_print_tasklist(void) {
    // 获取任务数量
    UBaseType_t total_tasks = uxTaskGetNumberOfTasks();
    
    // 创建缓冲区用于存储任务列表信息
    char task_list_buffer[512];
    memset(task_list_buffer, 0, sizeof(task_list_buffer));
    
    // 获取任务列表详细信息
    vTaskList((char*)&task_list_buffer);
    
    LOG_INFO("=== Task List Info === Total: %d", total_tasks);
    LOG_INFO("Task Name          State   Priority  Stack  Num");
    LOG_INFO("----------------------------------------------");
    
    // 逐行解析并输出任务信息
    char *line = strtok(task_list_buffer, "\n");
    while(line != NULL) {
        // 跳过空行
        if(strlen(line) > 0) {
            LOG_INFO("%s", line);
        }
        line = strtok(NULL, "\n");
    }
    
    // 获取运行时统计信息（如果启用了相关功能）
    #if(configGENERATE_RUN_TIME_STATS == 1)
        char runtime_stats_buffer[512];
        memset(runtime_stats_buffer, 0, sizeof(runtime_stats_buffer));
        vTaskGetRunTimeStats((char*)&runtime_stats_buffer);
        
        LOG_INFO("=== Task Runtime Stats ===");
        LOG_INFO("Task Name          Count      Percent");
        LOG_INFO("--------------------------------------");
        
        line = strtok(runtime_stats_buffer, "\n");
        while(line != NULL) {
            if(strlen(line) > 0) {
                LOG_INFO("%s", line);
            }
            line = strtok(NULL, "\n");
        }
    #endif
}

// ==================== 资源清理函数 ====================
void thread_manager_deinit(void) {
    // 1. 终止所有子线程
    thread_manager_send_cmd(THREAD_OP_DELETE, THREAD_TYPE_MOTOR, 0, 0, NULL);
    thread_manager_send_cmd(THREAD_OP_DELETE, THREAD_TYPE_GPIO_MONITOR, 0, 0, NULL);
    thread_manager_send_cmd(THREAD_OP_DELETE, THREAD_TYPE_UDP_LED, 0, 0, NULL);
    thread_manager_send_cmd(THREAD_OP_DELETE, THREAD_TYPE_ETH, 0, 0, NULL);
    thread_manager_send_cmd(THREAD_OP_DELETE, THREAD_TYPE_Feeding, 0, 0, NULL);
    osDelay(pdMS_TO_TICKS(50));
    
    // 2. 终止线程管理器自身
    if(thread_manager_task_handle != NULL) {
        osThreadTerminate(thread_manager_task_handle);
        thread_manager_task_handle = NULL;
    }
    
    // 3. 删除消息队列
    if(control_queue_id != NULL) {
        osMessageQueueDelete(control_queue_id);
        control_queue_id = NULL;
    }
    
    // 4. 复位全局变量
    g_target_angle = 0;
    g_motor_dir = MOTOR_DIR_STOP;
    
    LOG_INFO("Thread manager deinitialized completely");
}
