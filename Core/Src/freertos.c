/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications (适配UDP传入×10角度值)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "lwip/netif.h"
#include "lwip/udp.h"
#include "lwip/inet.h"
#include "string.h"
#include "tim.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "usart.h"
#include "lwip.h"
#include "err.h"
#include "logger.h"
#include "thread_manager.h"
#include "queue.h"
#include "ext_dev_ctrl.h"  // 添加外部设备控制头文件

// -------------------------- 全局变量 --------------------------
uint16_t Motor_Angle     = 0;       // 当前角度
uint16_t Target_Angle    = 0;       // 目标角度
uint16_t Motor_Length    = 0;       // 当前送料长度
uint16_t Target_Length   = 0;       // 目标送料长度
uint8_t  Motor_DIR       = 0;       // 电机方向变量（1=顺时针，2=逆时针）
uint8_t  Feeding_DIR     = 0;       // 送料方向变量（1=顺时针，2=逆时针）
static uint8_t Motor_Sys_St = 0;    // 电机运行状态
static uint8_t Feeding_Sys_St = 0;  // 送料运行状态

// -------------------------- 消息队列+中途停止核心定义 --------------------------

volatile uint8_t g_motor_force_stop_flag = 0;
volatile uint8_t g_feeding_force_stop_flag = 0;
QueueHandle_t g_motor_action_queue = NULL;
QueueHandle_t g_feeding_action_queue = NULL;

#define QUEUE_LENGTH    10                    // 队列长度：最多缓存10条指令
#define QUEUE_ITEM_SIZE sizeof(ActionMsg_t)   // 单个消息的字节数

// -------------------------- UDP相关宏定义 --------------------------
#define UDP_MAX_SEND_LEN       1472           // UDP最大发送长度（MTU-28）
#define UDP_RECV_BUF_LEN       10             // 指令帧长度（10字节有效）
#define UDP_FRAME_HEADER       0x55           // 帧头（Byte0）
#define UDP_FRAME_Read         0x02           // Byte1 = 0x03:读取数据指令
#define UDP_FRAME_Motor        0x03           // Byte1 = 0x02:电机控制指令
#define UDP_FRAME_Feeding      0x04           // Byte1 = 0x04:送料控制指令
#define UDP_FRAME_Cut          0x05           // Byte1 = 0x05:裁断控制指令
#define UDP_FRAME_Saw          0x06           // Byte1 = 0x06:圆锯控制指令
#define UDP_FRAME_TSJ          0x07           // Byte1 = 0x07:套丝控制指令
#define UDP_FRAME_Clamp        0x08           // Byte1 = 0x08:加紧控制指令
#define UDP_FRAME_Free         0x09           // Byte1 = 0x09:放松控制指令
#define UDP_CHECK_BYTE_INDEX   9              // 和校验字节位置（Byte9）
#define UDP_FRAME_VALID_LEN    10             // 有效帧长度（Byte0~Byte9）
#define UDP_LOCAL_PORT_T       8080           // 本地发送端口
#define UDP_LOCAL_PORT_R       8041           // 本地UDP接收端口
#define ENCODER_MAX_ANGLE      3600           // 电机编码器最大角度（×10值）
#define UDP_RESP_OK            0x01           // 响应成功
#define UDP_RESP_ERR           0x02           // 响应失败
#define UDP_RESP_EXIST         0x03           // 响应已存在

// -------------------------- UDP全局变量 --------------------------
static struct udp_pcb *udp_pcb_T = NULL;      // 发送任务UDP控制块
static struct udp_pcb *udp_pcb_R = NULL;      // 接收任务UDP控制块
extern ip4_addr_t dest_ipaddr;                // 目标IP（main.c定义）
extern uint16_t dest_port;                    // 目标端口（main.c定义）
extern struct netif gnetif;                   // 网络接口（lwip.c定义）
static uint8_t udp_init_flag = 0;             // UDP初始化标记

// -------------------------- 函数声明 --------------------------
// 任务函数声明
void StartMotorTask(void *argument);
void StartETHTask(void *argument);
void StartGPIOMonitorTask(void *argument);
void StartFeedingTask(void *argument);
void UDP_LED_Flash_Task(void *arg);

// UDP核心函数声明
static err_t UDP_Send_Data(uint8_t *data, uint16_t len);
static void UDP_Recv_Callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
static int UDP_Init(void);

// 工具函数声明
static uint16_t Read_GPIO_State(void);  // 读取GPIO状态
uint32_t encoder_gray_read(void);       // 读取绝对值编码器
uint16_t NormalizeAngle(int32_t angle); // 标准化角度值

// 消息队列发送函数声明
BaseType_t SendMotorActionMsg(ActionType_t action_type, void *param);
BaseType_t SendFeedingActionMsg(ActionType_t action_type, void *param);
void SendMotorCtrlMsg(uint16_t target_angle, uint8_t motor_dir);
void SendFeedingCtrlMsg(uint32_t target_length,uint8_t direction);
void SendForceStopMsg(void);

// 外部声明
extern void MX_LWIP_Init(void);
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

// 送料机控制指令
// 故障复位命令
static const uint8_t Reset[]     = {0x03,0x06,0x31,0x00,0x00,0x86,0x07,0x76};
// 使能命令
static const uint8_t Enable[]    = {0x03,0x06,0x31,0x00,0x00,0x0F,0xC6,0xD0};
// 非使能命令
static const uint8_t Disable[]   = {0x03,0x06,0x31,0x00,0x00,0x06,0x06,0xD6};
// 设置工作模式命令
static const uint8_t Mode_Set[]  = {0x03,0x06,0x35,0x00,0x00,0x03,0xC7,0xE5};
// 设置速度命令
static const uint8_t RPM_Set[]   = {0x03,0x10,0x6F,0x00,0x00,0x02,0x04,0x55,0x55,0x00,0x08,0x11,0xFF};
// 设置正向命令
static const uint8_t DIR_Set_CW[] = {0x03,0x06,0x47,0x00,0x00,0x01,0x5D,0x5C};
// 设置逆向命令
static const uint8_t DIR_Set_CCW[] = {0x03,0x06,0x47,0x00,0x00,0x00,0x9C,0x9C};

// -------------------------- FreeRTOS初始化 --------------------------
void MX_FREERTOS_Init(void) {
    // 1. 初始化日志系统
    logger_init();
    LOG_INFO("MX_FREERTOS_Init: System start successfully");
    
    // 2. 创建电机控制消息队列
    g_motor_action_queue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    if(g_motor_action_queue == NULL) {
        LOG_ERROR("MX_FREERTOS_Init: Create Motor action queue failed");
        return;
    }
    LOG_INFO("MX_FREERTOS_Init: Motor action queue created successfully");
    // 3. 创建送料控制消息队列
    g_feeding_action_queue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    if(g_feeding_action_queue == NULL) {
        LOG_ERROR("MX_FREERTOS_Init: Create Feeding action queue failed");
        return;
    }
    LOG_INFO("MX_FREERTOS_Init: Feeding action queue created successfully");
    
    // 3. 初始化线程管理器
    thread_manager_init();
    LOG_INFO("MX_FREERTOS_Init: Thread manager initialized");
    
    // 4. 发送任务创建指令
    thread_manager_send_cmd(THREAD_OP_CREATE, THREAD_TYPE_ETH, 0, 0, NULL);
    thread_manager_send_cmd(THREAD_OP_CREATE, THREAD_TYPE_GPIO_MONITOR, 0, 0, NULL);
    thread_manager_send_cmd(THREAD_OP_CREATE, THREAD_TYPE_MOTOR, 0, 0, NULL);
    thread_manager_send_cmd(THREAD_OP_CREATE, THREAD_TYPE_Feeding, 0, 0, NULL);
    
    LOG_INFO("MX_FREERTOS_Init: Task create commands sent to thread manager");
    // thread_manager_print_tasklist();
}

// -------------------------- 消息队列发送函数 --------------------------
BaseType_t SendMotorActionMsg(ActionType_t action_type, void *param) {
    if(g_motor_action_queue == NULL || param == NULL) {
        LOG_ERROR("SendMotorActionMsg: Queue is NULL or param is NULL");
        return pdFAIL;
    }

    ActionMsg_t send_msg = {0};
    send_msg.action_type = action_type;

    // 根据动作类型拷贝参数
    switch(action_type) {
        case ACTION_MOTOR_CTRL: {
            memcpy(&send_msg.params.motor_params, param, sizeof(send_msg.params.motor_params));
            break;
        }
        case ACTION_MOTOR_FORCE_STOP:
        case ACTION_SYSTEM_STOP: {
            memcpy(&send_msg.params.stop_params, param, sizeof(send_msg.params.stop_params));
            break;
        }
        default: {
            LOG_WARN("SendMotorActionMsg: Invalid action type: %d", action_type);
            return pdFAIL;
        }
    }

    // 非阻塞发送
    BaseType_t xStatus = xQueueSend(g_motor_action_queue, &send_msg, 0);
    
    if(xStatus == pdPASS) {
        LOG_DEBUG("SendMotorActionMsg: Send %d action success", action_type);
    } else {
        LOG_ERROR("SendMotorActionMsg: Send %d action failed (queue full?)", action_type);
    }

    return xStatus;
}
BaseType_t SendFeedingActionMsg(ActionType_t action_type, void *param) {
    if(g_feeding_action_queue == NULL || param == NULL) {
        LOG_ERROR("SendFeedingActionMsg: Queue is NULL or param is NULL");
        return pdFAIL;
    }

    ActionMsg_t send_msg = {0};
    send_msg.action_type = action_type;

    // 根据动作类型拷贝参数
    switch(action_type) {
        case ACTION_FEEDING_CTRL: {
            memcpy(&send_msg.params.feeding_params, param, sizeof(send_msg.params.feeding_params));
            break;
        }
        case ACTION_FEEDING_FORCE_STOP:
        case ACTION_SYSTEM_STOP: {
            memcpy(&send_msg.params.stop_params, param, sizeof(send_msg.params.stop_params));
            break;
        }
        default: {
            LOG_WARN("SendFeedingActionMsg: Invalid action type: %d", action_type);
            return pdFAIL;
        }
    }

    // 非阻塞发送
    BaseType_t xStatus = xQueueSend(g_feeding_action_queue, &send_msg, 0);
    
    if(xStatus == pdPASS) {
        LOG_DEBUG("SendFeedingActionMsg: Send %d action success", action_type);
    } else {
        LOG_ERROR("SendFeedingActionMsg: Send %d action failed (queue full?)", action_type);
    }

    return xStatus;
}
void SendMotorCtrlMsg(uint16_t target_angle, uint8_t motor_dir) {
    // 转发到线程管理器
    thread_manager_send_cmd(THREAD_OP_MOTOR_CTRL, THREAD_TYPE_MOTOR, target_angle, motor_dir, NULL);
}
void SendFeedingCtrlMsg(uint32_t target_length,uint8_t direction) {
    feed_direction_t validated_dir;
    
    // 验证方向值是否在有效范围内
    if(direction == FEED_DIR_FORWARD) {
        validated_dir = FEED_DIR_FORWARD;
    } else if(direction == FEED_DIR_BACKWARD) {
        validated_dir = FEED_DIR_BACKWARD;
    } else if(direction == FEED_DIR_STOP || direction == 0) {
        validated_dir = FEED_DIR_STOP;
    } else {
        LOG_ERROR("Invalid feed direction: %d", direction);
        return;  // 或其他错误处理
    }    
    // 转发到线程管理器
    thread_manager_send_cmd(THREAD_OP_FEEDING_CTRL, THREAD_TYPE_Feeding, target_length, validated_dir, NULL);
}
void SendForceStopMsg(void) {
    // 转发到线程管理器
    thread_manager_send_cmd(THREAD_OP_MOTOR_FORCE_STOP, THREAD_TYPE_MOTOR, 0, 0, NULL);
    thread_manager_send_cmd(THREAD_OP_FEEDING_FORCE_STOP, THREAD_TYPE_Feeding, 0, 0, NULL);
}

// -------------------------- GPIO状态读取 --------------------------
typedef struct {
    GPIO_TypeDef *gpio_port;  // GPIO端口
    uint16_t      gpio_pin;   // GPIO引脚
    uint16_t      bit_pos;    // 映射到gpio_state的位位置
} GPIO_State_Map_t;

static const GPIO_State_Map_t gpio_state_map[] = {
    {GPIOE, GPIO_PIN_4,    0},  // bit0: PE4
    {GPIOC, GPIO_PIN_13,   1},  // bit1: PC13
    {GPIOF, GPIO_PIN_0,    2},  // bit2: PF0
    {GPIOF, GPIO_PIN_1,    3},  // bit3: PF1
    {GPIOF, GPIO_PIN_2,    4},  // bit4: PF2
    {GPIOF, GPIO_PIN_9,    5},  // bit5: PF9
};

static uint16_t Read_GPIO_State(void) {
    uint16_t gpio_state = 0;
    uint8_t map_len = sizeof(gpio_state_map) / sizeof(GPIO_State_Map_t);

    // 遍历映射表，原子性读取所有GPIO状态
    for (uint8_t i = 0; i < map_len; i++) {
        if (HAL_GPIO_ReadPin(gpio_state_map[i].gpio_port, gpio_state_map[i].gpio_pin) != GPIO_PIN_SET) {
            gpio_state |= (1 << gpio_state_map[i].bit_pos);
        }
    }
    return gpio_state;
}

// -------------------------- 格雷码绝对值编码器读取函数 --------------------------
#define GRAY_10BIT_MAX   1023U   // 10bit格雷码最大值
#define ANGLE_MAX        3600    // 最大角度360,10倍转为整形（0.1精度）
uint32_t encoder_gray_read(void)
{
    uint16_t gray = 0;
    // G0~G9 对应GPIO，!取反适配NPN集电极开路输出
    gray |= !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10)   << 0;   // G0
    gray |= !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)    << 1;   // G1
    gray |= !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)    << 2;   // G2
    gray |= !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)    << 3;   // G3
    gray |= !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)    << 4;   // G4
    gray |= !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)    << 5;   // G5
    gray |= !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)    << 6;   // G6
    gray |= !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11)   << 7;   // G7
    gray |= !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12)   << 8;   // G8
    gray |= !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13)   << 9;   // G9

    gray = gray & 0x3FF;  // 仅保留低10位
    uint16_t binary = gray;
    uint16_t mask = gray;
    
    // 格雷码转二进制（标准算法）
    while (mask) {
        mask >>= 1;
        binary ^= mask;
    }
    binary = binary & 0x3FF;
    
    // 10位格雷码（0~1023）→ 角度（0~3600）的线性映射
    uint32_t Angle = (uint32_t)binary * ANGLE_MAX / GRAY_10BIT_MAX;
    
    // 调试日志：打印格雷码→二进制→角度的转换过程
    // LOG_DEBUG("Gray:0x%03X, Bin:0x%03X, Angle: %d", gray, binary, Angle);
    return Angle;
}

// -------------------------- 角度归一化 --------------------------
uint16_t NormalizeAngle(int32_t angle) {
    // 确保角度在0~3599范围内循环（3600=360）
    angle = angle % 3600;
    if (angle < 0) {
        angle += 3600;
    }
    return (uint16_t)angle;
}

// -------------------------- UDP LED闪烁任务--------------------------
void UDP_LED_Flash_Task(void *arg) {
    uint8_t UDP_LED_FLASH_CNT = 5;          // UDP初始化成功后LED闪烁次数
    #define UDP_LED_FLASH_DELAY 200         // LED闪烁间隔（ms）
    
    LOG_INFO("UDP_LED_Flash_Task: Start flashing LED");
    for (int i = 0; i < UDP_LED_FLASH_CNT; i++) {
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_3);
        osDelay(UDP_LED_FLASH_DELAY);
    }
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
    
    // 通过线程管理器删除自身任务
    thread_manager_send_cmd(THREAD_OP_DELETE, THREAD_TYPE_UDP_LED, 0, 0, NULL);
    LOG_INFO("UDP_LED_Flash_Task: Flash completed, task deleted");
    
    osThreadExit();
}

// -------------------------- UDP发送函数 --------------------------
static err_t UDP_Send_Data(uint8_t *data, uint16_t len) {
    err_t err = ERR_OK;

    // 1. 严格参数校验
    if (udp_pcb_T == NULL || data == NULL || len == 0 || 
        len > UDP_MAX_SEND_LEN || len > UDP_FRAME_VALID_LEN) {
        LOG_ERROR("UDP_Send_Data: Invalid param (pcb=%p, len=%d)", udp_pcb_T, len);
        return ERR_ARG;
    }

    // 2. 校验网络状态
    if (!netif_is_up(&gnetif) || !netif_is_link_up(&gnetif)) {
        LOG_ERROR("UDP_Send_Data: Netif DOWN (up=%d, link=%d)", netif_is_up(&gnetif), netif_is_link_up(&gnetif));
        return ERR_RTE;
    }

    // 3. 校验UDP PCB状态（LWIP 2.x兼容）
    #if LWIP_VERSION_MAJOR >= 2
    if (udp_pcb_T->state == UDP_STATE_CLOSED) {
        LOG_ERROR("UDP_Send_Data: UDP PCB Closed");
        return ERR_CONN;
    }
    #endif
    if (udp_pcb_T->local_port == 0) {
        LOG_ERROR("UDP_Send_Data: UDP PCB Unbound");
        return ERR_CONN;
    }

    // 4. 校验目标IP/端口合法性
    if (dest_ipaddr.addr == PP_HTONL(0x00000000)) {
        LOG_ERROR("UDP_Send_Data: Dest IP is 0.0.0.0");
        return ERR_ARG;
    }
    if (dest_port == 0 || dest_port > 65535) {
        LOG_ERROR("UDP_Send_Data: Dest Port invalid (%d)", dest_port);
        return ERR_ARG;
    }

    // 5. 分配pbuf缓冲区
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == NULL) {
        LOG_ERROR("UDP_Send_Data: PBUF Alloc Fail");
        return ERR_MEM;
    }

    // 6. 安全拷贝数据
    if (pbuf_take(p, data, len) != ERR_OK) {
        LOG_ERROR("UDP_Send_Data: PBUF Copy Fail");
        pbuf_free(p);
        return ERR_BUF;
    }

    // 7. 发送UDP数据
    err = udp_sendto(udp_pcb_T, p, &dest_ipaddr, dest_port);
    
    // 8. 错误码解析
    switch (err) {
        case ERR_OK:
            LOG_DEBUG("UDP_Send_Data: Send OK (len=%d, dest=%s:%d)", len, ip4addr_ntoa(&dest_ipaddr), dest_port);
            break;
        case ERR_RTE:
            LOG_ERROR("UDP_Send_Data: No Route to dest");
            break;
        case ERR_CONN:
            LOG_ERROR("UDP_Send_Data: Port Closed");
            break;
        case ERR_MEM:
            LOG_ERROR("UDP_Send_Data: Memory Full");
            break;
        case ERR_ARG:
            LOG_ERROR("UDP_Send_Data: Invalid Argument");
            break;
        default:
            LOG_ERROR("UDP_Send_Data: Unknown Error (%d)", err);
            break;
    }

    // 9. 释放pbuf
    pbuf_free(p);
    p = NULL;

    return err;
}

// -------------------------- UDP接收回调 --------------------------
static void UDP_Recv_Callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    // 空包检查
    if (p == NULL) {
        return;
    }
    
    // 1. 基础合法性校验
    if (p->tot_len < UDP_FRAME_VALID_LEN) {
        LOG_ERROR("UDP_Recv_Callback: Frame len too short (%d < %d)", p->tot_len, UDP_FRAME_VALID_LEN);
        pbuf_free(p);
        return;
    }
    if (addr == NULL || port == 0) {
        LOG_ERROR("UDP_Recv_Callback: Invalid addr/port");
        pbuf_free(p);
        return;
    }

    // 2. 安全拷贝数据到接收缓冲区
    uint8_t recv_buf[UDP_RECV_BUF_LEN] = {0};
    if (pbuf_copy_partial(p, recv_buf, UDP_FRAME_VALID_LEN, 0) != UDP_FRAME_VALID_LEN) {
        LOG_ERROR("UDP_Recv_Callback: Data Copy Fail");
        pbuf_free(p);
        return;
    }

    // 3. 帧头校验
    if (recv_buf[0] != UDP_FRAME_HEADER) {
        LOG_ERROR("UDP_Recv_Callback: Invalid header (0x%02X != 0x%02X)", recv_buf[0], UDP_FRAME_HEADER);
        pbuf_free(p);
        return;
    }

    // 4. 和校验
    uint8_t calc_check = 0;
    for (int i = 0; i < UDP_CHECK_BYTE_INDEX; i++) {
        calc_check += recv_buf[i];
    }
    if (calc_check != recv_buf[UDP_CHECK_BYTE_INDEX]) {
        LOG_ERROR("UDP_Recv_Callback: Checksum error (calc=0x%02X, recv=0x%02X)", calc_check, recv_buf[UDP_CHECK_BYTE_INDEX]);
        pbuf_free(p);
        return;
    }

    // 5. 指令处理
    if(recv_buf[1] == UDP_FRAME_Motor) { // 电机控制指令
        uint8_t dir = recv_buf[2];
        uint16_t Target_Angle_Parse = (uint16_t)((recv_buf[3] << 8) | recv_buf[4]);

        LOG_INFO("UDP_Recv_Callback: Motor cmd - dir = %d, target_angle = %d", 
                 dir, Target_Angle_Parse);

        uint8_t resp[2] = {0};
        resp[0] = UDP_FRAME_Motor; // 响应帧头

        // 如果指令是电机停止（dir=0）
        if(dir == 0) {
            SendForceStopMsg(); // 发送强制停止指令
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
            LOG_WARN("UDP_Recv_Callback: Motor Stop cmd received, send force stop msg");
            resp[1] = UDP_RESP_OK;
            UDP_Send_Data(resp, sizeof(resp));
            Motor_DIR = 0;
            Target_Angle = 0;
            Motor_Sys_St = 0;
            osDelay(10);
            pbuf_free(p);
            return;
        }

        // 指令是电机启动（dir=1/2）
        if(dir != 1 && dir != 2) {
            LOG_ERROR("UDP_Recv_Callback: Invalid dir (%d), only 1/2 allowed", dir);
            resp[1] = UDP_RESP_ERR;
            UDP_Send_Data(resp, sizeof(resp));
            pbuf_free(p);
            return;
        }
        if(Motor_Sys_St == 0){
            uint16_t target_angle = Target_Angle_Parse;
            if(target_angle > 3600) { // 最大3600（360）
                LOG_WARN("UDP_Recv_Callback: Target angle (%d) exceed max (3600), limit to 3600", 
                        target_angle);
                target_angle = 3600;
            }

            // 发送电机控制指令
            SendMotorCtrlMsg(target_angle, dir);
            
            // 确保电机任务存在
            osThreadId_t motor_handle = thread_manager_get_thread_handle(THREAD_TYPE_MOTOR);
            if (motor_handle == NULL) {
                thread_manager_send_cmd(THREAD_OP_CREATE, THREAD_TYPE_MOTOR, 0, 0, NULL);
                osDelay(10);
                motor_handle = thread_manager_get_thread_handle(THREAD_TYPE_MOTOR);
            }
            
            if(motor_handle != NULL) {
                resp[1] = UDP_RESP_OK;
                LOG_INFO("UDP_Recv_Callback: Motor cmd send to thread manager success");
            } else {
                resp[1] = UDP_RESP_ERR;
                LOG_ERROR("UDP_Recv_Callback: Motor task create failed");
            }
            UDP_Send_Data(resp, sizeof(resp));
        } else {
            resp[1] = UDP_RESP_EXIST; // 设备忙
            UDP_Send_Data(resp, sizeof(resp));
            LOG_WARN("UDP_Recv_Callback: Motor busy (Motor_Sys_St=1), reject cmd");
            pbuf_free(p);
            return;
        }
        
    }

    else if(recv_buf[1] == UDP_FRAME_Feeding) { // 送料控制指令
        uint8_t dir = recv_buf[2];
        uint16_t Target_Length_Parse = (uint16_t)((recv_buf[3] << 8) | recv_buf[4]);

        LOG_INFO("UDP_Recv_Callback: Feeding cmd - dir = %d, target_Length = %d", 
                 dir, Target_Length_Parse);

        uint8_t resp[2] = {0};
        resp[0] = UDP_FRAME_Feeding; // 响应帧头

        // 如果指令是停止（dir=0）
        if(dir == 0) {
            SendForceStopMsg(); // 发送强制停止指令
            LOG_WARN("UDP_Recv_Callback: Feeding Stop cmd received, send force stop msg");
            resp[1] = UDP_RESP_OK;
            UDP_Send_Data(resp, sizeof(resp));
            Feeding_DIR = 0;
            Target_Length = 0;
            Feeding_Sys_St = 0;
            osDelay(10);
            pbuf_free(p);
            return;
        }

        // 指令是电机启动（dir=1/2）
        if(dir != 1 && dir != 2) {
            LOG_ERROR("UDP_Recv_Callback: Invalid dir (%d), only 1/2 allowed", dir);
            resp[1] = UDP_RESP_ERR;
            UDP_Send_Data(resp, sizeof(resp));
            pbuf_free(p);
            return;
        }
        if(Feeding_Sys_St == 0) {
            // uint16_t Target_Length = Target_Length_Parse * 36000 / 13393;
            uint16_t Target_Length = Target_Length_Parse;
            // 发送送料控制指令
            SendFeedingCtrlMsg(Target_Length, dir);
            
            // 确保电机任务存在
            osThreadId_t Feeding_handle = thread_manager_get_thread_handle(THREAD_TYPE_Feeding);
            if (Feeding_handle == NULL) {
                thread_manager_send_cmd(THREAD_OP_CREATE, THREAD_TYPE_Feeding, 0, 0, NULL);
                osDelay(10);
                Feeding_handle = thread_manager_get_thread_handle(THREAD_TYPE_Feeding);
            }
            
            if(Feeding_handle != NULL) {
                resp[1] = UDP_RESP_OK;
                LOG_INFO("UDP_Recv_Callback: Feeding cmd send to thread manager success");
            } else {
                resp[1] = UDP_RESP_ERR;
                LOG_ERROR("UDP_Recv_Callback: Feeding task create failed");
            }
            UDP_Send_Data(resp, sizeof(resp));
        } else {
            resp[1] = UDP_RESP_EXIST; // 设备忙
            UDP_Send_Data(resp, sizeof(resp));
            LOG_WARN("UDP_Recv_Callback: Feeding busy (Feeding_Sys_St=1), reject cmd");
            pbuf_free(p);
            return;
        }
    }
    else if(recv_buf[1] == UDP_FRAME_TSJ) { // 套丝机指令
    // 仅转发指令到线程管理器，不直接调用函数
        thread_manager_send_cmd(THREAD_OP_TSJ_CTRL, THREAD_TYPE_NONE, 0, 0, NULL);
        // 发送响应
        uint8_t resp[] = {UDP_FRAME_TSJ, UDP_RESP_OK};
        UDP_Send_Data(resp, sizeof(resp));
    }
    else if(recv_buf[1] == UDP_FRAME_Saw) { // 圆锯指令
        thread_manager_send_cmd(THREAD_OP_SAW_CTRL, THREAD_TYPE_NONE, 0, 0, NULL);
        uint8_t resp[] = {UDP_FRAME_Saw, UDP_RESP_OK};
        UDP_Send_Data(resp, sizeof(resp));
    }
    else if(recv_buf[1] == UDP_FRAME_Cut) { // 裁断指令
        thread_manager_send_cmd(THREAD_OP_CUT_CTRL, THREAD_TYPE_NONE, 0, 0, NULL);
        uint8_t resp[] = {UDP_FRAME_Cut, UDP_RESP_OK};
        UDP_Send_Data(resp, sizeof(resp));
    }
    else if(recv_buf[1] == UDP_FRAME_Clamp) { // 夹紧指令
        thread_manager_send_cmd(THREAD_OP_CLMP_CTRL, THREAD_TYPE_NONE, 0, 0, NULL);
        uint8_t resp[] = {UDP_FRAME_Clamp, UDP_RESP_OK};
        UDP_Send_Data(resp, sizeof(resp));
    }
    else if(recv_buf[1] == UDP_FRAME_Free) { // 放松指令
        thread_manager_send_cmd(THREAD_OP_FREE_CTRL, THREAD_TYPE_NONE, 0, 0, NULL);
        uint8_t resp[] = {UDP_FRAME_Free, UDP_RESP_OK};
        UDP_Send_Data(resp, sizeof(resp));
    }
    // 读取状态指令
    else if(recv_buf[1] == UDP_FRAME_Read) { 
        // 组装响应包
        uint8_t send_buf[UDP_FRAME_VALID_LEN] = {0};
        uint16_t gpio_state = Read_GPIO_State();

        send_buf[0] = 0x33;                                 // 响应帧头
        send_buf[1] = UDP_FRAME_Read;                       // 响应指令类型
        send_buf[2] = (Motor_Sys_St || Feeding_Sys_St) ? 1 : 0; // 系统状态：0=待机，1=运行
        send_buf[3] = (uint8_t)(gpio_state & 0xFF);         // GPIO状态低8位
        send_buf[4] = (uint8_t)((gpio_state >> 8) & 0xFF);  // GPIO状态高8位

        // 计算校验和
        uint8_t send_check = 0;
        for (int i = 0; i < UDP_CHECK_BYTE_INDEX; i++) {
            send_check += send_buf[i];
        }
        send_buf[UDP_CHECK_BYTE_INDEX] = send_check;
        
        // 发送响应
        err_t send_err = UDP_Send_Data(send_buf, UDP_FRAME_VALID_LEN);
        if (send_err != ERR_OK) {
            LOG_ERROR("UDP_Recv_Callback: Send state fail (err=%d)", send_err);
        } else {
            LOG_DEBUG("UDP_Recv_Callback: Send state OK ");
        }
    }
    else { // 未知指令
        LOG_WARN("UDP_Recv_Callback: Unknown Command (0x%02X)", recv_buf[1]);
        uint8_t err_resp[] = "Error,Unknown Command";
        UDP_Send_Data(err_resp, sizeof(err_resp)-1);
    }

    // 释放pbuf
    pbuf_free(p);
    p = NULL;
}

// -------------------------- UDP初始化 --------------------------
static int UDP_Init(void) {
    // 防重复初始化
    if (udp_init_flag == 1) {
        LOG_INFO("UDP_Init: Already initialized");
        return 0;
    }

    // 校验LWIP核心是否初始化
    #if LWIP_VERSION_MAJOR >= 2
    if (!tcpip_initialized()) {
        LOG_ERROR("UDP_Init: LWIP Not Initialized");
        return -2;
    }
    #endif

    err_t err;
    ip_addr_t local_ip = IPADDR4_INIT_BYTES(0, 0, 0, 0);

    // 1. 初始化发送控制块
    udp_pcb_T = udp_new();
    if (udp_pcb_T == NULL) {
        LOG_ERROR("UDP_Init: Create send PCB fail");
        return -1;
    }

    // 绑定发送端口
    err = udp_bind(udp_pcb_T, &local_ip, UDP_LOCAL_PORT_T);
    if (err != ERR_OK) {
        LOG_ERROR("UDP_Init: Bind send port fail (err=%d)", err);
        udp_remove(udp_pcb_T);
        udp_pcb_T = NULL;
        return -3;
    }

    // 2. 初始化接收控制块
    udp_pcb_R = udp_new();
    if (udp_pcb_R == NULL) {
        LOG_ERROR("UDP_Init: Create recv PCB fail");
        udp_remove(udp_pcb_T);
        udp_pcb_T = NULL;
        return -4;
    }

    // 绑定接收端口
    err = udp_bind(udp_pcb_R, &local_ip, UDP_LOCAL_PORT_R);
    if (err != ERR_OK) {
        LOG_ERROR("UDP_Init: Bind recv port fail (err=%d)", err);
        udp_remove(udp_pcb_T);
        udp_remove(udp_pcb_R);
        udp_pcb_T = NULL;
        udp_pcb_R = NULL;
        return -5;
    }

    // 3. 注册接收回调
    udp_recv(udp_pcb_R, UDP_Recv_Callback, NULL);

    // 4. 标记初始化完成
    udp_init_flag = 1;
    LOG_INFO("UDP_Init: Success (send port=%d, recv port=%d)", UDP_LOCAL_PORT_T, UDP_LOCAL_PORT_R);
    
    // 5. 创建LED闪烁任务
    thread_manager_send_cmd(THREAD_OP_CREATE, THREAD_TYPE_UDP_LED, 0, 0, NULL);

    return 0;
}

// -------------------------- 电机控制任务（最终版） --------------------------
#define ANGLE_ERROR_THRESHOLD 20      // 角度误差阈值（±2）
void StartMotorTask(void *argument) {
    ActionMsg_t recv_msg;                 // FreeRTOS队列消息
    BaseType_t xStatus;                   // 队列操作状态
    uint16_t Current_Angle      = 0;      // 当前绝对角度
    uint16_t Start_Angle        = 0;      // 任务启动时的初始角度
    int16_t  Target_Angle_      = 0;      // 目标绝对角度
    uint32_t Rotate_Increment   = 0;      // 旋转增量
    uint8_t  stop_flag          = 0;      // 停止标志
    uint32_t task_start_tick    = 0;      // 任务启动时间戳
    uint8_t  irq_saved          = 0;      // 中断状态保存

    LOG_INFO("StartMotorTask: Motor control task started (FreeRTOS queue)");
    
    for(;;) { // 常驻循环，监听FreeRTOS原生队列
        // 阻塞等待队列消息
        xStatus = xQueueReceive(g_motor_action_queue, &recv_msg, portMAX_DELAY);
        
        if(xStatus == pdPASS) {
            // 解析动作类型
            switch(recv_msg.action_type) {
                case ACTION_MOTOR_CTRL: {
                    // 重置停止标志和时间戳
                    g_motor_force_stop_flag = 0;
                    stop_flag = 0;
                    task_start_tick = osKernelGetTickCount();
                    
                    // 赋值电机参数
                    Target_Angle = recv_msg.params.motor_params.target_angle;
                    Motor_DIR = recv_msg.params.motor_params.motor_dir;
                    
                    LOG_INFO("StartMotorTask: Receive Motor CMD (target_angle = %d , dir = %d)",
                             Target_Angle, Motor_DIR);

                    // ========== 临界区保护 ==========
                    irq_saved = __get_PRIMASK();
                    __disable_irq();
                    Start_Angle = encoder_gray_read(); // 读取当前角度
                    __enable_irq();
                    if(Start_Angle == 0xFFFF) {
                        LOG_ERROR("StartMotorTask: Encoder read failed!");
                        stop_flag = 1;
                        goto motor_cleanup;
                    }
                    Rotate_Increment = Target_Angle; 
                    if(Rotate_Increment > 3600) {
                        LOG_WARN("StartMotorTask: Increment too large, limit to 3600");
                        Rotate_Increment = 3600;
                    }

                    Motor_Sys_St = 1; // 标记系统运行中

                    // 设置电机方向 + 计算目标角度（核心逻辑）
                    if(Motor_DIR == 1) { // 顺时针（正传）：当前角度 + 增量
                        Target_Angle_ = NormalizeAngle((int32_t)Start_Angle + Rotate_Increment);
                        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
                        LOG_INFO("Rotate clockwise : %d / %d / %d ", 
                                 Start_Angle, Target_Angle_, Rotate_Increment);
                    } else if(Motor_DIR == 2) { // 逆时针（反转）：当前角度 - 增量
                        Target_Angle_ = NormalizeAngle((int32_t)Start_Angle - Rotate_Increment);
                        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
                        LOG_INFO("Rotate counterclockwise : %d / %d / %d ", 
                                 Start_Angle, Target_Angle_, Rotate_Increment);
                    } else {
                        LOG_WARN("StartMotorTask: Invalid dir, stop motor");
                        stop_flag = 1;
                        goto motor_cleanup;
                    }

                    // ========== 闭环控制主循环 ==========
                    while(!stop_flag) {
                        __enable_irq();
                        // osDelay(5);
                        // 检测中途停止标志
                        if(g_motor_force_stop_flag == 1) {
                            LOG_INFO("StartMotorTask: Force stop detected! Stop motor immediately");
                            g_motor_force_stop_flag = 0;
                            stop_flag = 1;
                            break;
                        }

                        // 读取当前角度
                        Current_Angle = encoder_gray_read();
                        if(Current_Angle == 0xFFFF) {
                            LOG_ERROR("StartMotorTask: Encoder read failed in loop!");
                            stop_flag = 1;
                            break;
                        }

                        // 计算角度差（绝对值）
                        uint16_t Angle_D = abs((int32_t)Target_Angle_ - (int32_t)Current_Angle);
                        // 处理360循环的最小角度差（如350和10的差是20，不是340）
                        if (Angle_D > 1800) {
                            Angle_D = 3600 - Angle_D;
                        }

                        // 调试日志（每100ms打印一次）
                        if((osKernelGetTickCount() - task_start_tick) % 100 == 0) {
                            LOG_DEBUG("StartMotorTask: Current = %d , Target = %d , Error = %d",
                                     Current_Angle, Target_Angle_, Angle_D);
                        }

                        // 达到目标角度停止
                        if(Angle_D <= ANGLE_ERROR_THRESHOLD) {
                            LOG_INFO("StartMotorTask: Reach target (error = %d ≤ %d)",
                                     Angle_D, ANGLE_ERROR_THRESHOLD);
                            stop_flag = 1;
                            break;
                        }

                        osDelay(20); // 20ms延时

                        // 超时防护（10秒）
                        if(osKernelGetTickCount() - task_start_tick > pdMS_TO_TICKS(10000)) {
                            LOG_ERROR("StartMotorTask: Timeout (10s), force stop!");
                            stop_flag = 1;
                            break;
                        }
                    }

                    // ========== 电机停机+资源清理 ==========
                motor_cleanup:
                    if(irq_saved != 0) { // 原状态是关中断，才需要恢复
                        __enable_irq();
                    }
                    
                    // 停止电机
                    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
                    
                    // 重置状态
                    Motor_Sys_St = 0;
                    Motor_DIR = 0;
                    Target_Angle = 0;
                    LOG_INFO("StartMotorTask: Motor stopped, cleanup done");
                    break;
                }
                case ACTION_MOTOR_FORCE_STOP: {
                    // 收到强制停止指令，置位标志
                    LOG_INFO("StartMotorTask: Receive force stop cmd from queue");
                    g_motor_force_stop_flag = 1;
                    break;
                }
                default: {
                    LOG_WARN("StartMotorTask: Receive invalid action type: %d", recv_msg.action_type);
                    break;
                }
            }
        } else {
            LOG_WARN("StartMotorTask: Read queue failed (status = %d)", xStatus);
        }
    }

    // 容错退出
    LOG_ERROR("StartMotorTask: Unexpected exit");
    thread_manager_send_cmd(THREAD_OP_DELETE, THREAD_TYPE_MOTOR, 0, 0, NULL);
    osThreadExit();
}

// -------------------------- GPIO监控任务 --------------------------
void StartGPIOMonitorTask(void *argument) {
    uint16_t gpio_state = 0;
    uint16_t last_gpio_state = 0;
    uint16_t gpio_debounce_buf[3] = {0}; // 防抖缓存

    LOG_INFO("StartGPIOMonitorTask: Started");
    for(;;) {
        // 1. 防抖处理
        gpio_debounce_buf[0] = gpio_debounce_buf[1];
        gpio_debounce_buf[1] = gpio_debounce_buf[2];
        gpio_debounce_buf[2] = Read_GPIO_State();
        // gpio_state = Read_GPIO_State();
        // LOG_DEBUG("GPIO_State: %d",gpio_state);
        
        if((gpio_debounce_buf[0] == gpio_debounce_buf[1]) && 
           (gpio_debounce_buf[1] == gpio_debounce_buf[2])) {
            gpio_state = gpio_debounce_buf[2];
        } else {
            gpio_state = last_gpio_state;
        }
        
        // 2. 状态变化则发送UDP反馈
        if(gpio_state != last_gpio_state) {
            uint8_t feedback_buf[UDP_FRAME_VALID_LEN] = {0};
            uint32_t current_angle = encoder_gray_read(); // 读取当前角度（×10值）

            feedback_buf[0] = 0x33;                                 // 响应帧头
            feedback_buf[1] = UDP_FRAME_Read;                       // 响应指令类型
            feedback_buf[2] = (Motor_Sys_St || Feeding_Sys_St) ? 1 : 0; // 系统状态：0=待机，1=运行
            feedback_buf[3] = (uint8_t)(gpio_state & 0xFF);         // GPIO状态低8位
            feedback_buf[4] = (uint8_t)((gpio_state >> 8) & 0xFF);  // GPIO状态高8位

            // 计算校验和
            uint8_t check_sum = 0;
            for(int i = 0; i < UDP_CHECK_BYTE_INDEX; i++) {
                check_sum += feedback_buf[i];
            }
            feedback_buf[UDP_CHECK_BYTE_INDEX] = check_sum;
            
            // 发送反馈
            UDP_Send_Data(feedback_buf, UDP_FRAME_VALID_LEN);
            last_gpio_state = gpio_state;
            
            LOG_DEBUG("StartGPIOMonitorTask: GPIO state changed (0x%04X), current_angle=%d (raw=%d)", 
                     gpio_state, current_angle/10, current_angle);
        }
        
        osDelay(5); // 5ms检查一次
    }
}

// -------------------------- 以太网通信任务 --------------------------
#define NETIF_READY_TIMEOUT_MS 10000 // 网络就绪超时（10s）
#define NETIF_CHECK_INTERVAL_MS 200  // 网络检查间隔（200ms）
void StartETHTask(void *argument) {
    int udp_init_ret;
    uint32_t netif_wait_ticks = 0;
    uint8_t netif_ready_flag = 0;

    LOG_INFO("StartETHTask: Started, initializing LWIP");
    // 确保LWIP只初始化一次
    static uint8_t lwip_init_flag = 0;
    if(lwip_init_flag == 0) {
        MX_LWIP_Init();
        lwip_init_flag = 1;
        LOG_INFO("StartETHTask: LWIP initialized successfully");
    }

    // 2. 等待网络就绪
    LOG_INFO("StartETHTask: Waiting for network ready...");
    while (1) {
        if (netif_is_up(&gnetif) &&
            netif_is_link_up(&gnetif) &&
            !ip4_addr_isany(&gnetif.ip_addr)) {
            netif_ready_flag = 1;
            break;
        }

        netif_wait_ticks += NETIF_CHECK_INTERVAL_MS;
        if (netif_wait_ticks >= NETIF_READY_TIMEOUT_MS) {
            LOG_ERROR("StartETHTask: Network ready timeout (10s)");
            LOG_ERROR("StartETHTask: Netif state - up = %d, link = %d, IP = %s",
                        netif_is_up(&gnetif) ? 1 : 0,
                        netif_is_link_up(&gnetif) ? 1 : 0,
                        ip4addr_ntoa(&gnetif.ip_addr));
            break;
        }
        osDelay(NETIF_CHECK_INTERVAL_MS);
    }

    // 3. 网络就绪则初始化UDP
    if (netif_ready_flag) {
        LOG_INFO("StartETHTask: Network ready - IP = %s, MAC = %02X:%02X:%02X:%02X:%02X:%02X",
                    ip4addr_ntoa(&gnetif.ip_addr),
                    gnetif.hwaddr[0], gnetif.hwaddr[1], gnetif.hwaddr[2],
                    gnetif.hwaddr[3], gnetif.hwaddr[4], gnetif.hwaddr[5]);

        udp_init_ret = UDP_Init();
        if (udp_init_ret == 0) {
            LOG_INFO("StartETHTask: UDP init success");
        } else {
            LOG_ERROR("StartETHTask: UDP init fail (ret = %d)", udp_init_ret);
        }
    } else {
        LOG_ERROR("StartETHTask: Skip UDP init (network not ready)");
    }

    // 4. 网络维护主循环
    for (;;) {
        // 处理以太网接收数据
        ethernetif_input(&gnetif);
        // LWIP核心定时任务
        sys_check_timeouts();

        // 检测网络是否断开
        if (!netif_is_up(&gnetif) ||
            !netif_is_link_up(&gnetif) ||
            ip4_addr_isany(&gnetif.ip_addr)) {
            LOG_ERROR("StartETHTask: Network disconnected, trying to reconnect");
            // 尝试重启LWIP
            if(lwip_init_flag == 1) {
                netif_set_down(&gnetif);
                netif_set_up(&gnetif);
                LOG_INFO("StartETHTask: Netif restarted");
            }
        }
        
        osDelay(50); // 50ms维护一次
    }
}

static uint16_t T_Delay = 30;               // 发送间隔
#define FEED_CALC_FACTOR_NUMERATOR   12954  // 分子：π × 41.2 × 100，保留2位小数精度
#define FEED_CALC_FACTOR_DENOMINATOR 4000   // 分母：每圈脉冲数


uint32_t feed_get_encoder_count(void) {
    return __HAL_TIM_GET_COUNTER(&htim3);
}
// void StartFeedingTask(void *argument) {
//     ActionMsg_t recv_msg;               // 队列消息
//     BaseType_t xStatus;                 // 队列操作状态
//     uint32_t start_count        = 0;    // 开始时编码器计数值
//     uint32_t current_count      = 0;    // 当前编码器计数值
//     uint32_t target_length      = 0;    // 目标长度
//     // uint32_t target_angle       = 0;    // 目标角度（已不再使用）
//     int32_t angle_difference    = 0;    // 角度差值
//     uint8_t stop_flag           = 0;    // 停止标志
//     uint32_t task_start_tick    = 0;    // 任务启动时间戳
//     feed_direction_t direction  = 0;    // 方向
    

//     if((htim3.Instance->CR1 & TIM_CR1_CEN) == 0) {
//         // 启动TIM3编码器模式计数（硬件配置已由CubeMX完成，仅需启动）
//         if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
//             LOG_ERROR("StartFeedingTask: TIM3 encoder start failed!");
//             // 初始化失败则退出任务
//             osThreadExit();
//         }
//         __HAL_TIM_SET_COUNTER(&htim3, 0);
//         LOG_INFO("StartFeedingTask: TIM3 encoder initialized and started");

//     } else {
//         LOG_DEBUG("StartFeedingTask: TIM3 encoder already running");
//     }

//     LOG_INFO("StartFeedingTask: Feeding control task started (FreeRTOS queue)");
    
//     for(;;) {
//         // 阻塞等待队列消息
//         xStatus = xQueueReceive(g_feeding_action_queue, &recv_msg, portMAX_DELAY);
        
//         if(xStatus == pdPASS) {
//             // 解析动作类型
//             switch(recv_msg.action_type) {
//                 case ACTION_FEEDING_CTRL: {
//                     // 重置停止标志和时间戳
//                     g_feeding_force_stop_flag = 0;
//                     stop_flag = 0;
//                     task_start_tick = osKernelGetTickCount();
//                     __HAL_TIM_SET_COUNTER(&htim3, 0);

//                     // 赋值送料参数
//                     target_length = recv_msg.params.feeding_params.target_length;
//                     direction = recv_msg.params.feeding_params.direction;
                    
//                     LOG_INFO("StartFeedingTask: Receive Feeding CMD (target_length = %d , dir = %d)",
//                              target_length, direction);

//                     // 读取当前编码器值作为起始值
//                     start_count = feed_get_encoder_count();
//                     if(start_count == 0xFFFF) {
//                         LOG_ERROR("StartFeedingTask: Encoder read failed!");
//                         stop_flag = 1;
//                         goto feeding_cleanup;
//                     }

//                     // 不再计算目标角度，直接使用目标长度
//                     LOG_DEBUG("StartFeedingTask: Start feeding (target_length = %d)", target_length);
                    
//                     Feeding_Sys_St = 1; // 标记系统运行中

//                     // 初始化电机：复位、禁用、设置模式、设置速度
//                     HAL_UART_Transmit(&huart4, Reset, sizeof(Reset), 100);
//                     vTaskDelay(T_Delay);
//                     HAL_UART_Transmit(&huart4, Disable, sizeof(Disable), 100);
//                     vTaskDelay(T_Delay);
//                     HAL_UART_Transmit(&huart4, Mode_Set, sizeof(Mode_Set), 100);
//                     vTaskDelay(T_Delay);
//                     HAL_UART_Transmit(&huart4, RPM_Set, sizeof(RPM_Set), 100);
//                     vTaskDelay(T_Delay);

//                     // 根据方向设置电机
//                     switch(direction) {
//                         case FEED_DIR_FORWARD: // 正向送料
//                             HAL_UART_Transmit(&huart4, DIR_Set_CW, sizeof(DIR_Set_CW), 100);
//                             vTaskDelay(T_Delay);
//                             HAL_UART_Transmit(&huart4, Enable, sizeof(Enable), 100);
//                             LOG_INFO("StartFeedingTask: Forward feeding, start_count = %d, target_length = %d", 
//                                      start_count, target_length);
//                             break;
//                         case FEED_DIR_BACKWARD: // 反向送料
//                             HAL_UART_Transmit(&huart4, DIR_Set_CCW, sizeof(DIR_Set_CCW), 100);
//                             vTaskDelay(T_Delay);
//                             HAL_UART_Transmit(&huart4, Enable, sizeof(Enable), 100);
//                             LOG_INFO("StartFeedingTask: Backward feeding, start_count = %d, target_length = %d", 
//                                      start_count, target_length);
//                             break;
//                         default:
//                             LOG_ERROR("StartFeedingTask: Invalid feed direction: %d", direction);
//                             stop_flag = 1;
//                             goto feeding_cleanup;
//                     }

//                     // ========== 闭环控制主循环 ==========
//                     while(!stop_flag) {
//                         // 检测中途停止标志
//                         if(g_feeding_force_stop_flag == 1) {
//                             LOG_INFO("StartFeedingTask: Force stop detected! Stop Feeding immediately");
//                             g_feeding_force_stop_flag = 0;
//                             stop_flag = 1;
//                             break;
//                         }
                                                
//                         // 检查是否达到目标
//                         current_count = feed_get_encoder_count();
//                         // if(current_count == 0xFFFF) {
//                         //     LOG_ERROR("StartFeedingTask: Encoder read failed in loop!");
//                         //     stop_flag = 1;
//                         //     break;
//                         // }

//                         // 计算当前角度差 - 使用强制类型转换为有符号整型，自动处理溢出问题
//                         int32_t angle_difference = (int32_t)(current_count - start_count);
//                         // LOG_DEBUG("StartFeedingTask: Current angle difference = %d", angle_difference);
//                         // // uint32_t length_difference = (uint64_t)abs(angle_difference) * FEED_CALC_FACTOR_NUMERATOR / FEED_CALC_FACTOR_DENOMINATOR;
//                         // uint32_t length_difference = (uint32_t)(angle_difference / 32);
//                         // if(length_difference >= target_length) {
//                         //     LOG_INFO("StartFeedingTask: Feeding complete! Stop Feeding");
//                         //     stop_flag = 1;
//                         //     break;
//                         // }
//                         uint32_t length_mm_x1000 = abs(angle_difference) * 314159 * 40 / 4000 / 1000;
//                         if(length_mm_x1000 >= target_length * 1000) { // target_length也放大1000倍
//                             stop_flag = 1;
//                             break;
//                         }
//                         // // 计数值→长度→角度（和送料指令的转换逻辑完全一致）
//                         // uint32_t current_length = (uint64_t)abs(angle_difference) * FEED_CALC_FACTOR_DENOMINATOR / FEED_CALC_FACTOR_NUMERATOR;
//                         // uint32_t current_angle = feed_calc_target_angle(current_length);
//                         // // 计算当前角度差
//                         // if (direction == FEED_DIR_FORWARD) {
//                         //     // 正向：当前计数应该比开始计数大
//                         //     if (current_count >= start_count) {
//                         //         angle_difference = current_count - start_count;
//                         //     } else {
//                         //         // 处理计数器翻转情况（从65535到0）
//                         //         angle_difference = (0xFFFF - start_count) + current_count + 1;
//                         //     }
//                         // } else {
//                         //     // 反向：当前计数应该比开始计数小
//                         //     if (current_count <= start_count) {
//                         //         angle_difference = start_count - current_count;
//                         //     } else {
//                         //         // 处理计数器翻转情况（从0到65535）
//                         //         angle_difference = (0xFFFF - current_count) + start_count + 1;
//                         //     }
//                         // }

//                         // // 计数值→长度→角度（和送料指令的转换逻辑完全一致）
//                         // uint32_t current_length = (uint64_t)abs(angle_difference) * FEED_CALC_FACTOR_DENOMINATOR / FEED_CALC_FACTOR_NUMERATOR;
//                         // uint32_t current_angle = feed_calc_target_angle(current_length);

//                         // // 调试日志（每100ms打印一次）
//                         // if((osKernelGetTickCount() - task_start_tick) % 100 == 0) {
//                         //     LOG_DEBUG("StartFeedingTask: Current angle = %d , Target = %d", 
//                         //              current_angle, target_angle);
//                         // }

//                         // // 达到目标长度停止
//                         // if (current_angle >= target_angle) {
//                         //     LOG_INFO("StartFeedingTask: Reach target (current_angle = %d ≥ %d)",
//                         //              current_angle, target_angle);
//                         //     stop_flag = 1;
//                         //     break;
//                         // }

//                         vTaskDelay(pdMS_TO_TICKS(20)); // 20ms延时

//                         // 超时防护（10秒）
//                         if(osKernelGetTickCount() - task_start_tick > pdMS_TO_TICKS(10000)) {
//                             LOG_ERROR("StartFeedingTask: Timeout (10s), force stop!");
//                             stop_flag = 1;
//                             break;
//                         }
//                     }

//                     // ========== 电机停机+资源清理 ==========
//                 feeding_cleanup:
//                     // 停止电机
//                     HAL_UART_Transmit(&huart4, Disable, sizeof(Disable), 100);
                    
//                     // 重置状态
//                     Feeding_Sys_St = 0;
//                     Feeding_DIR = 0;
//                     target_length = 0;
//                     LOG_INFO("StartFeedingTask: Feeding stopped, cleanup done");
//                     break;
//                 }
//                 case ACTION_FEEDING_FORCE_STOP: {
//                     // 收到强制停止指令，停止电机
//                     LOG_INFO("StartFeedingTask: Receive force stop cmd from queue");
//                     HAL_UART_Transmit(&huart4, Disable, sizeof(Disable), 100);
//                     g_feeding_force_stop_flag = 1;
//                     break;
//                 }
//                 case ACTION_SYSTEM_STOP: {
//                     // 收到系统停止指令，退出任务
//                     LOG_INFO("StartFeedingTask: Receive system stop cmd from queue");
//                     HAL_UART_Transmit(&huart4, Disable, sizeof(Disable), 100);
//                     vQueueDelete(g_feeding_action_queue);
//                     g_feeding_action_queue = NULL;
//                     osThreadExit();
//                     break;
//                 }
//                 default: {
//                     LOG_WARN("StartFeedingTask: Receive invalid action type: %d", recv_msg.action_type);
//                     break;
//                 }
//             }
//         } else {
//             LOG_WARN("StartFeedingTask: Read queue failed (status = %d)", xStatus);
//         }
//     }

//     // 容错退出
//     LOG_ERROR("StartFeedingTask: Unexpected exit");
//     if(g_feeding_action_queue != NULL) {
//         vQueueDelete(g_feeding_action_queue);
//         g_feeding_action_queue = NULL;
//     }
//     osThreadExit();
// }
void StartFeedingTask(void *argument) {
    ActionMsg_t recv_msg;               // 队列消息
    BaseType_t xStatus;                 // 队列操作状态
    uint32_t current_count      = 0;    // 当前编码器计数值
    uint32_t last_count         = 0;    // 上一次编码器计数值
    uint32_t target_length      = 0;    // 目标长度（单位：mm）
    uint32_t total_pulse_diff   = 0;    // 累计总脉冲差（绝对值，32位）
    uint8_t stop_flag           = 0;    // 停止标志
    uint32_t task_start_tick    = 0;    // 任务启动时间戳
    feed_direction_t direction  = 0;    // 方向（仅用于电机控制）

    if((htim3.Instance->CR1 & TIM_CR1_CEN) == 0) {
        // 启动TIM3编码器模式计数
        if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
            LOG_ERROR("StartFeedingTask: TIM3 encoder start failed!");
            osThreadExit();
        }
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        LOG_INFO("StartFeedingTask: TIM3 encoder initialized and started");
    } else {
        LOG_DEBUG("StartFeedingTask: TIM3 encoder already running");
    }

    LOG_INFO("StartFeedingTask: Feeding control task started (FreeRTOS queue)");
    
    for(;;) {
        xStatus = xQueueReceive(g_feeding_action_queue, &recv_msg, portMAX_DELAY);
        
        if(xStatus == pdPASS) {
            switch(recv_msg.action_type) {
                case ACTION_FEEDING_CTRL: {
                    // 重置所有状态
                    g_feeding_force_stop_flag = 0;
                    stop_flag = 0;
                    task_start_tick = osKernelGetTickCount();
                    __HAL_TIM_SET_COUNTER(&htim3, 0);
                    total_pulse_diff = 0;  
                    last_count = 0;        

                    // 赋值送料参数
                    target_length = recv_msg.params.feeding_params.target_length;
                    direction = recv_msg.params.feeding_params.direction;
                    
                    LOG_INFO("StartFeedingTask: Receive Feeding CMD (target_length = %d mm , dir = %d)",
                             target_length, direction);

                    // 读取初始计数值
                    current_count = feed_get_encoder_count();
                    if(current_count == 0xFFFF) {
                        LOG_ERROR("StartFeedingTask: Encoder read failed!");
                        stop_flag = 1;
                        goto feeding_cleanup;
                    }
                    last_count = current_count; // 初始化last_count

                    LOG_DEBUG("StartFeedingTask: Start feeding (target_length = %d mm)", target_length);
                    Feeding_Sys_St = 1;

                    // 电机初始化
                    HAL_UART_Transmit(&huart4, Reset, sizeof(Reset), 100);
                    vTaskDelay(T_Delay);
                    HAL_UART_Transmit(&huart4, Disable, sizeof(Disable), 100);
                    vTaskDelay(T_Delay);
                    HAL_UART_Transmit(&huart4, Mode_Set, sizeof(Mode_Set), 100);
                    vTaskDelay(T_Delay);
                    HAL_UART_Transmit(&huart4, RPM_Set, sizeof(RPM_Set), 100);
                    vTaskDelay(T_Delay);

                    // 电机方向控制（仅控制转向，不参与计数）
                    switch(direction) {
                        case FEED_DIR_FORWARD: 
                            HAL_UART_Transmit(&huart4, DIR_Set_CW, sizeof(DIR_Set_CW), 100);
                            vTaskDelay(T_Delay);
                            HAL_UART_Transmit(&huart4, Enable, sizeof(Enable), 100);
                            LOG_INFO("StartFeedingTask: Forward feeding, target_length = %d mm", target_length);
                            break;
                        case FEED_DIR_BACKWARD: 
                            HAL_UART_Transmit(&huart4, DIR_Set_CCW, sizeof(DIR_Set_CCW), 100);
                            vTaskDelay(T_Delay);
                            HAL_UART_Transmit(&huart4, Enable, sizeof(Enable), 100);
                            LOG_INFO("StartFeedingTask: Backward feeding, target_length = %d mm", target_length);
                            break;
                        default:
                            LOG_ERROR("StartFeedingTask: Invalid feed direction: %d", direction);
                            stop_flag = 1;
                            goto feeding_cleanup;
                    }

                    // ========== 闭环控制主循环（核心：处理两种溢出） ==========
                    while(!stop_flag) {
                        // 强制停止检测
                        if(g_feeding_force_stop_flag == 1) {
                            LOG_INFO("StartFeedingTask: Force stop detected!");
                            g_feeding_force_stop_flag = 0;
                            stop_flag = 1;
                            break;
                        }
                                                
                        // 读取当前计数值
                        current_count = feed_get_encoder_count();
                        // if(current_count == 0xFFFF) {
                        //     LOG_ERROR("StartFeedingTask: Encoder read failed in loop!");
                        //     stop_flag = 1;
                        //     break;
                        // }

                        // ========== 核心：计算脉冲差（处理正向/反向两种溢出） ==========
                        uint32_t pulse_diff = 0;
                        uint32_t pulse_max = 0xFFFF; // 16位计数器最大值
                        uint32_t normal_diff = abs((int32_t)current_count - (int32_t)last_count); // 正常差值（无溢出）
                        
                        // 判断是否是溢出场景：正常差值 > 脉冲最大值的一半 → 说明是反向溢出
                        if(normal_diff > (pulse_max / 2)) {
                            // 溢出场景（正向/反向都适用）：实际差值 = 脉冲最大值 - 正常差值
                            pulse_diff = pulse_max - normal_diff + 1;
                        } else {
                            // 无溢出：直接用正常差值
                            pulse_diff = normal_diff;
                        }

                        // ========== 异常脉冲差过滤 ==========
                        if(pulse_diff > 1000) { // 阈值根据电机速度调整（比如20ms最多走200脉冲就设200）
                            LOG_WARN("StartFeedingTask: Abnormal pulse diff = %d, skip", pulse_diff);
                            last_count = current_count;
                            vTaskDelay(pdMS_TO_TICKS(20));
                            continue;
                        }

                        // 累加总脉冲差（绝对值）
                        total_pulse_diff += pulse_diff;
                        last_count = current_count; // 更新上一次计数值

                        // ========== 脉冲→长度换算 ==========
                        // uint32_t current_length_mm = (uint32_t)((uint64_t)total_pulse_diff * 314159 * 40 / 4000 / 100000);
                        uint32_t current_length_mm = (uint32_t)((uint64_t)total_pulse_diff * 1301002 / 10000 / 4000);
                        
                        LOG_DEBUG("StartFeedingTask: Total pulse = %d, Current length = %d mm, Target = %d mm",
                                 total_pulse_diff, current_length_mm, target_length);

                        // 到达目标长度停止
                        if(current_length_mm >= target_length) {
                            LOG_INFO("StartFeedingTask: Feeding complete! Current length = %d mm ≥ Target = %d mm",
                                     current_length_mm, target_length);
                            stop_flag = 1;
                            break;
                        }

                        // 延时+超时防护
                        vTaskDelay(pdMS_TO_TICKS(20));
                        if(osKernelGetTickCount() - task_start_tick > pdMS_TO_TICKS(10000)) {
                            LOG_ERROR("StartFeedingTask: Timeout (10s), force stop!");
                            stop_flag = 1;
                            break;
                        }
                    }

                    // ========== 电机停机+清理 ==========
                feeding_cleanup:
                    HAL_UART_Transmit(&huart4, Disable, sizeof(Disable), 100);
                    Feeding_Sys_St = 0;
                    Feeding_DIR = 0;
                    target_length = 0;
                    total_pulse_diff = 0;
                    last_count = 0;
                    LOG_INFO("StartFeedingTask: Feeding stopped, cleanup done");
                    break;
                }
                case ACTION_FEEDING_FORCE_STOP: {
                    LOG_INFO("StartFeedingTask: Receive force stop cmd from queue");
                    HAL_UART_Transmit(&huart4, Disable, sizeof(Disable), 100);
                    g_feeding_force_stop_flag = 1;
                    break;
                }
                case ACTION_SYSTEM_STOP: {
                    LOG_INFO("StartFeedingTask: Receive system stop cmd from queue");
                    HAL_UART_Transmit(&huart4, Disable, sizeof(Disable), 100);
                    vQueueDelete(g_feeding_action_queue);
                    g_feeding_action_queue = NULL;
                    osThreadExit();
                    break;
                }
                default: {
                    LOG_WARN("StartFeedingTask: Receive invalid action type: %d", recv_msg.action_type);
                    break;
                }
            }
        } else {
            LOG_WARN("StartFeedingTask: Read queue failed (status = %d)", xStatus);
        }
    }

    // 容错退出
    LOG_ERROR("StartFeedingTask: Unexpected exit");
    if(g_feeding_action_queue != NULL) {
        vQueueDelete(g_feeding_action_queue);
        g_feeding_action_queue = NULL;
    }
    osThreadExit();
}