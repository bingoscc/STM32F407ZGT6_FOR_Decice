/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ext_dev_ctrl.h
  * @brief   This file contains all the function prototypes for
  *          the ext_dev_ctrl.c file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXT_DEV_CTRL_H__
#define __EXT_DEV_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"  // 包含CAN相关定义

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/**
 * @brief  TSJ设备控制功能
 * @note  通过UART3（485通道）发送预定义的数据序列
 */
void ext_dev_tsj_control(void);

/**
 * @brief  锯切设备控制功能
 * @note  控制GPIO引脚以实现锯切动作
 */
void ext_dev_saw_control(void);

/**
 * @brief  松开设备控制功能
 * @note  控制气缸松开马达
 */
void ext_dev_free_control(void);

/**
 * @brief  夹紧设备控制功能
 * @note  控制气缸夹紧马达
 */
void ext_dev_clmp_control(void);

/**
 * @brief  裁断设备控制功能
 * @note  通过CAN总线发送预定义的数据序列控制裁断设备
 */
void ext_dev_cut_control(void);

/**
 * @brief  使用CAN1发送数据
 * @param  id: 消息ID
 * @param  ide: 标识符类型 (CAN_ID_STD 或 CAN_ID_EXT)
 * @param  data: 要发送的数据数组
 * @param  len: 数据长度 (最大8字节)
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN1_Transmit(uint32_t id, uint32_t ide, uint8_t *data, uint8_t len);

/**
 * @brief  使用CAN2发送数据
 * @param  id: 消息ID
 * @param  ide: 标识符类型 (CAN_ID_STD 或 CAN_ID_EXT)
 * @param  data: 要发送的数据数组
 * @param  len: 数据长度 (最大8字节)
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN2_Transmit(uint32_t id, uint32_t ide, uint8_t *data, uint8_t len);

/**
 * @brief  使用CAN1发送标准数据帧
 * @param  std_id: 标准ID (11位, 范围: 0-0x7FF)
 * @param  data: 要发送的数据数组
 * @param  len: 数据长度 (最大8字节)
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN1_SendStdData(uint32_t std_id, uint8_t *data, uint8_t len);

/**
 * @brief  使用CAN1发送扩展数据帧
 * @param  ext_id: 扩展ID (29位, 范围: 0-0x1FFFFFFF)
 * @param  data: 要发送的数据数组
 * @param  len: 数据长度 (最大8字节)
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN1_SendExtData(uint32_t ext_id, uint8_t *data, uint8_t len);

/**
 * @brief  使用CAN2发送标准数据帧
 * @param  std_id: 标准ID (11位, 范围: 0-0x7FF)
 * @param  data: 要发送的数据数组
 * @param  len: 数据长度 (最大8字节)
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN2_SendStdData(uint32_t std_id, uint8_t *data, uint8_t len);

/**
 * @brief  使用CAN2发送扩展数据帧
 * @param  ext_id: 扩展ID (29位, 范围: 0-0x1FFFFFFF)
 * @param  data: 要发送的数据数组
 * @param  len: 数据长度 (最大8字节)
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN2_SendExtData(uint32_t ext_id, uint8_t *data, uint8_t len);

/**
 * @brief  启动CAN1接收
 * @note  需要在初始化完成后调用此函数启动CAN接收
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN1_Start(void);

/**
 * @brief  启动CAN2接收
 * @note  需要在初始化完成后调用此函数启动CAN接收
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN2_Start(void);

/**
 * @brief  从CAN1接收数据
 * @param  rxData: 接收到的数据缓冲区
 * @param  rxLen: 接收到的数据长度
 * @param  rxID: 接收到的报文ID
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN1_Receive(uint8_t *rxData, uint8_t *rxLen, uint32_t *rxID);

/**
 * @brief  从CAN2接收数据
 * @param  rxData: 接收到的数据缓冲区
 * @param  rxLen: 接收到的数据长度
 * @param  rxID: 接收到的报文ID
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN2_Receive(uint8_t *rxData, uint8_t *rxLen, uint32_t *rxID);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __EXT_DEV_CTRL_H__ */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */