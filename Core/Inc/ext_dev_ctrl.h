#ifndef __EXT_DEV_CTRL_H
#define __EXT_DEV_CTRL_H

#include "main.h"
#include "usart.h"
// static uint8_t T_Delay = 30;
/**
 * @brief 外部设备控制功能
 * @note 用于控制通过UART3（485通道）连接的外部设备
 */

// 套丝机设备控制命令
void ext_dev_tsj_control(void);

// 裁断设备控制任务
void ext_dev_cut_control(void);

// 设备加紧控制任务
void ext_dev_clmp_control(void);

// 设备放松控制任务
void ext_dev_free_control(void);

// 圆锯设备控制任务
void ext_dev_saw_control(void);
// 其他外部设备控制函数声明
// void ext_dev_example_control(void);

#endif