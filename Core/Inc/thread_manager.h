#ifndef THREAD_MANAGER_H
#define THREAD_MANAGER_H

#include "cmsis_os2.h"
#include "stdint.h"
#include "FreeRTOS.h"
#include "queue.h"

// ==================== 版本宏定义 ====================
#define THREAD_MANAGER_VERSION_MAJOR 1
#define THREAD_MANAGER_VERSION_MINOR 0
#define THREAD_MANAGER_VERSION_PATCH 0
#define THREAD_MANAGER_VERSION (THREAD_MANAGER_VERSION_MAJOR << 16 | THREAD_MANAGER_VERSION_MINOR << 8 | THREAD_MANAGER_VERSION_PATCH)

// ==================== 队列配置宏 ====================
#define MOTOR_QUEUE_LENGTH      10      // 电机队列长度
#define FEEDING_QUEUE_LENGTH    10      // 送料队列长度
#define QUEUE_ITEM_SIZE         sizeof(ActionMsg_t)

// ==================== 调试开关 ====================
#ifdef DEBUG
#define THREAD_MANAGER_CHECK_EN 1        // 开启参数校验
#else
#define THREAD_MANAGER_CHECK_EN 0        // 关闭参数校验（提升性能）
#endif

// ==================== 核心枚举定义（优化顺序+规范命名） ====================
/**
 * @brief 线程操作类型枚举
 */
typedef enum {
    THREAD_OP_CREATE = 0,               // 创建线程
    THREAD_OP_DELETE,                   // 删除线程
    THREAD_OP_SUSPEND,                  // 挂起线程
    THREAD_OP_RESUME,                   // 恢复线程
    THREAD_OP_MOTOR_CTRL,               // 电机控制指令
    THREAD_OP_MOTOR_FORCE_STOP,         // 电机强制停止指令
    THREAD_OP_FEEDING_CTRL,             // 喂料控制指令
    THREAD_OP_FEEDING_FORCE_STOP,       // 喂料强制停止指令
    THREAD_OP_TSJ_CTRL,                 // 套丝机动作
    THREAD_OP_CLMP_CTRL,                // 加紧动作
    THREAD_OP_FREE_CTRL,                // 放松动作
    THREAD_OP_CUT_CTRL,                 // 裁断动作
    THREAD_OP_SAW_CTRL,                 // 圆锯动作
    THREAD_OP_SAW_FREE,                 // 圆锯台钳释放
    THREAD_OP_MAX                       // 枚举边界（用于合法性校验）
} thread_operation_t;

/**
 * @brief 线程类型枚举
 */
typedef enum {
    THREAD_TYPE_MOTOR = 0,              // 电机任务
    THREAD_TYPE_GPIO_MONITOR,           // GPIO监控任务
    THREAD_TYPE_UDP_LED,                // UDP LED闪烁任务
    THREAD_TYPE_ETH,                    // ETH任务
    THREAD_TYPE_Feeding,                // 送料进程
    THREAD_TYPE_Encoder,                // 增量编码器进程
    THREAD_TYPE_NONE,                   // 无效线程类型（用于合法性校验）
    THREAD_TYPE_MAX                     // 枚举边界（用于合法性校验）
} thread_type_t;

/**
 * @brief 电机方向枚举（新增：类型约束）
 */
typedef enum {
    MOTOR_DIR_CW = 0x01,                // 顺时针
    MOTOR_DIR_CCW = 0x02,               // 逆时针
    MOTOR_DIR_STOP = 0x00               // 停止
} motor_direction_t;

/**
 * @brief 喂料方向枚举（优化：明确十六进制）
 */
typedef enum {
    FEED_DIR_FORWARD = 0x01,            // 正向送料
    FEED_DIR_BACKWARD = 0x02,           // 反向送料
    FEED_DIR_STOP = 0x00                // 停止
} feed_direction_t;

/**
 * @brief 动作类型枚举：定义所有可通过队列发送的指令类型
 */
typedef enum {
    ACTION_MOTOR_CTRL = 0,              // 电机位置控制（带角度/方向参数）
    ACTION_MOTOR_FORCE_STOP = 1,        // 电机中途强制停止
    ACTION_SYSTEM_STOP = 2,             // 系统整体停止
    ACTION_FEEDING_CTRL = 3,            // 喂料控制（带长度/方向参数）
    ACTION_FEEDING_FORCE_STOP = 4,      // 喂料强制停止
    ACTION_NONE       = 0xFF            // 无效动作（容错用）
} ActionType_t;

// ==================== 核心结构体定义（修复C语言兼容性） ====================
/**
 * @brief 喂料控制参数（独立定义，兼容C语言）
 */
typedef struct {
    uint32_t target_length;             // 目标长度
    feed_direction_t direction;         // 送料方向（强类型约束）
} feeding_ctrl_param_t;

/**
 * @brief 消息结构体：队列传输的最小单元
 */
typedef struct {
    ActionType_t action_type;           // 动作类型
    union {
        // 电机控制参数
        struct {
            uint16_t target_angle;      // 目标角度（如1800=180°）
            motor_direction_t motor_dir;// 电机方向（强类型约束）
        } motor_params;
        
        // 喂料控制参数（复用独立定义的结构体）
        feeding_ctrl_param_t feeding_params;
        
        // 停止类参数
        struct {
            uint8_t reserve;            // 预留字段
        } stop_params;
    } params;
} ActionMsg_t;

// 电机控制参数（兼容原有定义，保留）
typedef struct {
    uint16_t target_angle;              // 目标角度
    motor_direction_t motor_dir;        // 电机方向（替换uint8_t为强类型）
} motor_ctrl_param_t;

/**
 * @brief 线程控制消息结构（优化：参数改为联合体，增加语义）
 */
typedef struct {
    thread_operation_t operation;       // 操作类型
    thread_type_t thread_type;          // 线程类型
    union {
        struct {
            uint16_t target_angle;      // 电机目标角度
            motor_direction_t dir;      // 电机方向（强类型）
        } motor;
        struct {
            uint32_t target_length;     // 送料目标长度
            feed_direction_t dir;       // 送料方向（强类型）
        } feeding;
        uint32_t raw[2];                // 原始参数（兼容旧代码）
    } params;
    void *argument;                     // 任务参数
} thread_control_msg_t;

// ==================== 全局变量声明（优化：最小暴露） ====================
// 电机任务专用消息队列句柄
extern QueueHandle_t g_motor_action_queue;
// 喂料任务专用消息队列句柄
extern QueueHandle_t g_feeding_action_queue;
// 电机强制停止标志位
extern volatile uint8_t g_motor_force_stop_flag;
// 送料强制停止标志位
extern volatile uint8_t g_feeding_force_stop_flag;

// ==================== 接口函数声明（新增封装接口+完善注释） ====================
/**
 * @brief 线程管理初始化
 * @note 初始化线程管理器核心队列，必须在FreeRTOS启动前调用
 */
void thread_manager_init(void);

/**
 * @brief 向线程管理器发送控制消息
 * @param operation 操作类型（创建/删除/控制等）
 * @param thread_type 目标线程类型（电机/送料/ETH等）
 * @param param1 通用参数1（电机角度/送料长度）
 * @param param2 通用参数2（电机方向/送料方向）
 * @param argument 任务创建时的入参（其他操作传NULL）
 * @note 非阻塞发送，消息队列满时会丢弃（需确保队列长度足够）
 */
void thread_manager_send_cmd(thread_operation_t operation, thread_type_t thread_type, 
                            uint32_t param1, uint32_t param2, void *argument);

/**
 * @brief 通过消息队列获取当前线程句柄
 * @param type 线程类型
 * @return 成功返回线程句柄，失败返回NULL
 */
osThreadId_t thread_manager_get_thread_handle(thread_type_t type);

/**
 * @brief 同步创建线程（阻塞直到创建完成）
 * @param thread_type 线程类型
 * @param argument 任务入参
 * @return 成功返回线程句柄，失败返回NULL
 * @note 仅用于初始化阶段，避免运行时阻塞
 */
osThreadId_t thread_manager_create_thread_sync(thread_type_t thread_type, void *argument);

/**
 * @brief 资源清理函数
 * @note 释放线程管理器占用的队列和内存资源
 */
void thread_manager_deinit(void);

/**
 * @brief 获取电机目标角度（封装全局变量）
 * @return 当前电机目标角度
 */
uint16_t motor_get_target_angle(void);

/**
 * @brief 设置电机目标角度（封装全局变量）
 * @param angle 目标角度（0~3600）
 */
void motor_set_target_angle(uint16_t angle);

/**
 * @brief 获取电机当前方向（封装全局变量）
 * @return 当前电机方向
 */
motor_direction_t motor_get_direction(void);

/**
 * @brief 设置电机当前方向（封装全局变量）
 * @param dir 电机方向
 */
void motor_set_direction(motor_direction_t dir);

/**
 * @brief 打印系统中所有任务的信息列表
 * @note 输出包括任务名、状态、优先级、堆栈使用情况等信息
 */
void thread_manager_print_tasklist(void);

#endif // THREAD_MANAGER_H