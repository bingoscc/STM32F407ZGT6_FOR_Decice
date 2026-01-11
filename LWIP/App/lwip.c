/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : LWIP.c
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
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
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */
#include "ethernetif.h"
#include <string.h>
#include "usart.h"
#include "cmsis_os.h"
#include "main.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "stm32f4xx_hal_flash.h"
#include <stdlib.h>
#include <stdio.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/* Private function prototypes -----------------------------------------------*/
static void ethernet_link_status_updated(struct netif *netif);
/* ETH Variables initialization ----------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Variables Initialization */
struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];
ip4_addr_t dest_ipaddr;
uint16_t dest_port;

/* USER CODE BEGIN OS_THREAD_ATTR_CMSIS_RTOS_V2 */
#define INTERFACE_THREAD_STACK_SIZE ( 1024 )
osThreadAttr_t attributes;
/* USER CODE END OS_THREAD_ATTR_CMSIS_RTOS_V2 */

// 串口配置网络参数相关变量
extern UART_HandleTypeDef huart1;
static uint8_t uart_rx_buffer[100];
static uint8_t config_mode = 0;     // 0-正常模式，1-配置模式

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

// 定义Flash存储地址（选择Sector 11，地址范围0x080E0000 - 0x080FFFFF）
#define CONFIG_FLASH_ADDRESS  0x080E0000
#define CONFIG_MAGIC_NUMBER   0x12345678

// 配置结构体
typedef struct {
    uint32_t  magic_number;  // 魔数，用于验证数据有效性
    uint8_t   ip_address[4];
    uint8_t   netmask[4];
    uint8_t   gateway[4];
    uint8_t   dest_ip[4];
    uint16_t  dest_port;
    uint32_t  crc;           // 校验和，用于验证数据完整性
} NetworkConfigTypeDef;

// Flash操作互斥锁
static osMutexId_t flashMutexHandle;

/**
  * @brief  计算简单的CRC校验和
  * @param  data: 数据指针
  * @param  length: 数据长度
  * @retval CRC校验和
  */
static uint32_t calculate_crc(uint8_t *data, uint32_t length) {
    uint32_t crc = 0;
    for (uint32_t i = 0; i < length; i++) {
        crc += data[i];
    }
    return crc;
}

/**
  * @brief  将网络配置保存到Flash
  * @param  config: 网络配置结构体指针
  * @retval 保存结果 (HAL_OK/HAL_ERROR)
  */
static HAL_StatusTypeDef save_config_to_flash(NetworkConfigTypeDef *config) {
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t address = CONFIG_FLASH_ADDRESS;
    uint32_t data;
    uint32_t *ptr = (uint32_t *)config;
    uint32_t data_length = sizeof(NetworkConfigTypeDef);
    
    // 获取Flash操作互斥锁，超时1秒
    if (osMutexAcquire(flashMutexHandle, pdMS_TO_TICKS(1000)) != osOK) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"Flash Mutex Acquire Failed!\r\n", 30, 100);
        return HAL_ERROR;
    }
    
    // 解锁Flash
    HAL_FLASH_Unlock();
    
    // 清除Sector 11的错误标志
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                          FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_PGSERR);  

    // 擦除Sector 11
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_11;
    EraseInitStruct.NbSectors = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        osMutexRelease(flashMutexHandle);
        return HAL_ERROR;
    }
    
    // 写入数据
    for (uint32_t i = 0; i < (data_length + 3) / 4; i++) {
        data = ptr[i];
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data) != HAL_OK) {
            HAL_FLASH_Lock();
            osMutexRelease(flashMutexHandle);
            return HAL_ERROR;
        }
        address += 4;
    }
    
    // 锁定Flash
    HAL_FLASH_Lock();
    
    // 释放互斥锁
    osMutexRelease(flashMutexHandle);
    
    return status;
}

/**
  * @brief  从Flash加载网络配置
  * @param  config: 网络配置结构体指针
  * @retval 加载结果 (HAL_OK/HAL_ERROR)
  */
static HAL_StatusTypeDef load_config_from_flash(NetworkConfigTypeDef *config) {
    // 获取Flash操作互斥锁，超时1秒
    if (osMutexAcquire(flashMutexHandle, pdMS_TO_TICKS(1000)) != osOK) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"Flash Mutex Acquire Failed!\r\n", 30, 100);
        return HAL_ERROR;
    }
    
    uint32_t *flash_ptr = (uint32_t *)CONFIG_FLASH_ADDRESS;
    uint32_t *config_ptr = (uint32_t *)config;
    uint32_t data_length = sizeof(NetworkConfigTypeDef);
    
    // 读取数据
    for (uint32_t i = 0; i < (data_length + 3) / 4; i++) {
        config_ptr[i] = flash_ptr[i];
    }
    
    // 验证魔数
    if (config->magic_number != CONFIG_MAGIC_NUMBER) {
        osMutexRelease(flashMutexHandle);
        return HAL_ERROR;
    }
    
    // 验证校验和
    uint32_t crc = calculate_crc((uint8_t *)config, data_length - 4); // 不包括CRC字段本身
    if (crc != config->crc) {
        osMutexRelease(flashMutexHandle);
        return HAL_ERROR;
    }
    
    // 释放互斥锁
    osMutexRelease(flashMutexHandle);
    
    return HAL_OK;
}

/**
  * @brief  Parses a string to extract IP address components
  * @param  str: Input string containing IP address (e.g., "192.168.1.100")
  * @param  ip_array: Output array to store the 4 IP components
  * @retval 1 if parsing successful, 0 otherwise
  */
static uint8_t parse_ip_string(char* str, uint8_t* ip_array) {
  uint32_t ip[4];
  if (sscanf(str, "%lu.%lu.%lu.%lu", &ip[0], &ip[1], &ip[2], &ip[3]) == 4) {
    if (ip[0] <= 255 && ip[1] <= 255 && ip[2] <= 255 && ip[3] <= 255) {
      ip_array[0] = (uint8_t)ip[0];
      ip_array[1] = (uint8_t)ip[1];
      ip_array[2] = (uint8_t)ip[2];
      ip_array[3] = (uint8_t)ip[3];
      return 1;
    }
  }
  return 0;
}

/**
 * @brief  串口按\r\n接收字符串（适配任意长度IP，最方便的通用函数）
 * @param  huart: 串口句柄（如&huart1）
 * @param  buf: 接收缓冲区（复用原有uart_rx_buffer）
 * @param  buf_max_len: 缓冲区最大长度（防止溢出）
 * @param  timeout_ms: 总超时时间（ms）
 * @retval >0: 实际接收的有效字节数；0: 超时/失败
 */
uint16_t UART_Recv_Until_Enter(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t buf_max_len, uint32_t timeout_ms) {
    uint16_t rx_len = 0;                  // 实际接收长度
    uint32_t start_tick = HAL_GetTick();  // 超时计时起点
    uint8_t temp_byte = 0;                // 临时接收1字节
    
    memset(buf, 0, buf_max_len);          // 清空缓冲区（避免残留）
    while (1) {
        // 1. 超时判断（超过设定时间直接返回0）
        if (HAL_GetTick() - start_tick > timeout_ms) {
            return 0;
        }
        // 2. 非阻塞接收1字节（每次等10ms，不卡死）
        if (HAL_UART_Receive(huart, &temp_byte, 1, 10) == HAL_OK) {
            // 3. 遇到终止符（\r或\n），停止接收
            if (temp_byte == '\r' || temp_byte == '\n') {
                buf[rx_len] = '\0';     // 字符串结尾符
                return rx_len;          // 返回有效长度
            }
            // 4. 正常字符，拼接至缓冲区（防止溢出）
            if (rx_len < buf_max_len - 1) {
                buf[rx_len++] = temp_byte;
            }
        }
    }
}

/**
  * @brief  配置网络参数（一次性接收所有参数：IP 子网掩码 网关 目标IP 端口）
  * @retval None
  */
static void configure_network_parameters(void) {
  uint8_t temp_ip[4], temp_netmask[4], temp_gateway[4], temp_dest_ip[4];
  uint16_t temp_dest_port;
  uint8_t valid_ip = 0, valid_netmask = 0, valid_gateway = 0, valid_dest_ip = 0, valid_dest_port = 0;
  uint32_t rx_len = 0;
  char response[100];
  NetworkConfigTypeDef config;
  
  // 提示用户一次性输入所有参数（空格分隔）
  HAL_UART_Transmit(&huart1, (uint8_t*)"Network Configuration Mode\r\n", 28, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*)"Enter All Params (Format: IP Netmask Gateway DestIP Port)\r\n", 59, 200);
  HAL_UART_Transmit(&huart1, (uint8_t*)"Example: 192.168.1.100 255.255.255.0 192.168.1.1 192.168.1.3 8080\r\n", 67, 200);
  
  // 一次性接收完整配置字符串（30秒超时）
  memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
  rx_len = UART_Recv_Until_Enter(&huart1, uart_rx_buffer, sizeof(uart_rx_buffer), 30000);

  if (rx_len > 0) {
    // 拆分字符串（空格分隔）
    char *token = strtok((char*)uart_rx_buffer, " ");
    if (token == NULL) {
      HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid Format! No IP found\r\n", 31, 100);
      return;
    }
    
    // 1. 解析本机IP
    if (parse_ip_string(token, temp_ip)) {
      valid_ip = 1;
      snprintf(response, sizeof(response), "Local IP Address: %d.%d.%d.%d\r\n", temp_ip[0], temp_ip[1], temp_ip[2], temp_ip[3]);
      HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), 100);
    } else {
      HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid Local IP Format!\r\n", 28, 100);
      return;
    }
    
    // 2. 解析子网掩码
    token = strtok(NULL, " ");
    if (token == NULL || !parse_ip_string(token, temp_netmask)) {
      HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid Netmask Format!\r\n", 25, 100);
      return;
    }
    valid_netmask = 1;
    snprintf(response, sizeof(response), "Netmask: %d.%d.%d.%d\r\n", temp_netmask[0], temp_netmask[1], temp_netmask[2], temp_netmask[3]);
    HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), 100);
    
    // 3. 解析网关
    token = strtok(NULL, " ");
    if (token == NULL || !parse_ip_string(token, temp_gateway)) {
      HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid Gateway Format!\r\n", 25, 100);
      return;
    }
    valid_gateway = 1;
    snprintf(response, sizeof(response), "Gateway: %d.%d.%d.%d\r\n", temp_gateway[0], temp_gateway[1], temp_gateway[2], temp_gateway[3]);
    HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), 100);
    
    // 4. 解析目标IP
    token = strtok(NULL, " ");
    if (token == NULL || !parse_ip_string(token, temp_dest_ip)) {
      HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid Destination IP Format!\r\n", 31, 100);
      return;
    }
    valid_dest_ip = 1;
    snprintf(response, sizeof(response), "Destination IP: %d.%d.%d.%d\r\n", temp_dest_ip[0], temp_dest_ip[1], temp_dest_ip[2], temp_dest_ip[3]);
    HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), 100);
    
    // 5. 解析目标端口
    token = strtok(NULL, " ");
    if (token == NULL) {
      HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid Destination Port!\r\n", 27, 100);
      return;
    }
    temp_dest_port = atoi(token);
    if (temp_dest_port > 0 && temp_dest_port <= 65535) {
      valid_dest_port = 1;
      snprintf(response, sizeof(response), "Destination Port: %d\r\n", temp_dest_port);
      HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), 100);
    } else {
      HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid Destination Port!\r\n", 27, 100);
      return;
    }
    
  } else {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Timeout! Using default IP settings.\r\n", 37, 100);
    return;
  }
  
  // 如果所有参数都有效，则更新网络配置并保存到Flash
  if (valid_ip && valid_netmask && valid_gateway && valid_dest_ip && valid_dest_port) {
    IP_ADDRESS[0] = temp_ip[0];
    IP_ADDRESS[1] = temp_ip[1];
    IP_ADDRESS[2] = temp_ip[2];
    IP_ADDRESS[3] = temp_ip[3];
    
    NETMASK_ADDRESS[0] = temp_netmask[0];
    NETMASK_ADDRESS[1] = temp_netmask[1];
    NETMASK_ADDRESS[2] = temp_netmask[2];
    NETMASK_ADDRESS[3] = temp_netmask[3];
    
    GATEWAY_ADDRESS[0] = temp_gateway[0];
    GATEWAY_ADDRESS[1] = temp_gateway[1];
    GATEWAY_ADDRESS[2] = temp_gateway[2];
    GATEWAY_ADDRESS[3] = temp_gateway[3];
    
    // 修正：用LWIP标准宏赋值（无需反向）
    IP4_ADDR(&dest_ipaddr, temp_dest_ip[0], temp_dest_ip[1], temp_dest_ip[2], temp_dest_ip[3]);
    dest_port = temp_dest_port;
    
    // 准备要保存的配置
    config.magic_number = CONFIG_MAGIC_NUMBER;
    memcpy(config.ip_address, temp_ip, 4);
    memcpy(config.netmask, temp_netmask, 4);
    memcpy(config.gateway, temp_gateway, 4);
    memcpy(config.dest_ip, temp_dest_ip, 4);
    config.dest_port = temp_dest_port;
    config.crc = calculate_crc((uint8_t*)&config, sizeof(config) - 4); // 不包括CRC字段
    
    // 保存到Flash
    if (save_config_to_flash(&config) == HAL_OK) {
      HAL_UART_Transmit(&huart1, (uint8_t*)"Network configuration saved successfully!\r\n", 43, 100);
    } else {
      HAL_UART_Transmit(&huart1, (uint8_t*)"Failed to save network configuration!\r\n", 39, 100);
    }
  } else {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Failed to update network configuration!\r\n", 41, 100);
  }
}
/**
  * LwIP initialization function
  */
void MX_LWIP_Init(void)
{
  // 初始化Flash操作互斥锁
  osMutexAttr_t flashMutexAttr = {
    .name = "FlashMutex",
    .attr_bits = osMutexPrioInherit,
  };
  flashMutexHandle = osMutexNew(&flashMutexAttr);
  if (flashMutexHandle == NULL) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Flash Mutex Create Failed!\r\n", 28, 100);
  }
  
  // 检查是否进入配置模式 (如果串口接收缓冲区第一个字符是'C')
  if (HAL_UART_Receive(&huart1, uart_rx_buffer, 1, 1000) == HAL_OK) {
    if (uart_rx_buffer[0] == 'C' || uart_rx_buffer[0] == 'c') {
      config_mode = 1;
    }
  }
  
  // 默认配置
  IP_ADDRESS[0] = 192;
  IP_ADDRESS[1] = 168;
  IP_ADDRESS[2] = 1;
  IP_ADDRESS[3] = 100;
  NETMASK_ADDRESS[0] = 255;
  NETMASK_ADDRESS[1] = 255;
  NETMASK_ADDRESS[2] = 255;
  NETMASK_ADDRESS[3] = 0;
  GATEWAY_ADDRESS[0] = 192;
  GATEWAY_ADDRESS[1] = 168;
  GATEWAY_ADDRESS[2] = 1;
  GATEWAY_ADDRESS[3] = 1;
  
  // 默认目标IP和端口
  IP4_ADDR(&dest_ipaddr, 192, 168, 1, 3); 
  dest_port = 8080;

  // 如果不是配置模式，尝试从Flash加载配置
  if (!config_mode) {
    NetworkConfigTypeDef config;
    if (load_config_from_flash(&config) == HAL_OK) {
      // 使用从Flash加载的配置
      IP_ADDRESS[0] = config.ip_address[0];
      IP_ADDRESS[1] = config.ip_address[1];
      IP_ADDRESS[2] = config.ip_address[2];
      IP_ADDRESS[3] = config.ip_address[3];
      
      NETMASK_ADDRESS[0] = config.netmask[0];
      NETMASK_ADDRESS[1] = config.netmask[1];
      NETMASK_ADDRESS[2] = config.netmask[2];
      NETMASK_ADDRESS[3] = config.netmask[3];
      
      GATEWAY_ADDRESS[0] = config.gateway[0];
      GATEWAY_ADDRESS[1] = config.gateway[1];
      GATEWAY_ADDRESS[2] = config.gateway[2];
      GATEWAY_ADDRESS[3] = config.gateway[3];

      IP4_ADDR(&dest_ipaddr, config.dest_ip[0], config.dest_ip[1], config.dest_ip[2], config.dest_ip[3]);
      dest_port = config.dest_port;
      
      HAL_UART_Transmit(&huart1, (uint8_t*)"Configuration loaded from Flash.\r\n", 34, 100);
    } else {
      HAL_UART_Transmit(&huart1, (uint8_t*)"Using default configuration.\r\n", 30, 100);
    }
  } else {
    configure_network_parameters();
  }

/* USER CODE BEGIN IP_ADDRESSES */
/* USER CODE END IP_ADDRESSES */

  /* Initialize the LwIP stack with RTOS */
  tcpip_init( NULL, NULL );

  /* IP addresses initialization without DHCP (IPv4) */
  IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

  /* add the network interface (IPv4/IPv6) with RTOS */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

  /* Registers the default network interface */
  netif_set_default(&gnetif);

  /* We must always bring the network interface up connection or not... */
  netif_set_up(&gnetif);

  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&gnetif, ethernet_link_status_updated);

  /* Create the Ethernet link handler thread */
/* USER CODE BEGIN H7_OS_THREAD_NEW_CMSIS_RTOS_V2 */
  memset(&attributes, 0x0, sizeof(osThreadAttr_t));
  attributes.name = "EthLink";
  attributes.stack_size = INTERFACE_THREAD_STACK_SIZE;
  attributes.priority = osPriorityBelowNormal;
  osThreadNew(ethernet_link_thread, &gnetif, &attributes);
/* USER CODE END H7_OS_THREAD_NEW_CMSIS_RTOS_V2 */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */
}

#ifdef USE_OBSOLETE_USER_CODE_SECTION_4
/* Kept to help code migration. (See new 4_1, 4_2... sections) */
/* Avoid to use this user section which will become obsolete. */
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
#endif

/**
  * @brief  Notify the User about the network interface config status
  * @param  netif: the network interface
  * @retval None
  */
static void ethernet_link_status_updated(struct netif *netif)
{
  if (netif_is_up(netif) && netif_is_link_up(netif)) {
    /* 链路已连接：打印提示+重启DHCP（若启用）+更新LED状态 */
    HAL_UART_Transmit(&huart1, (uint8_t*)"Ethernet link is UP\r\n", 21, 500);
    // 若启用DHCP，可在此处触发DHCP重新获取IP
    // dhcp_start(netif);
  } else {
    /* 链路断开：打印提示+标记链路状态+停止发包 */
    HAL_UART_Transmit(&huart1, (uint8_t*)"Ethernet link is DOWN\r\n", 23, 500);
    netif_set_link_down(netif); // 显式标记链路断开
  }
}

#if defined ( __CC_ARM )  /* MDK ARM Compiler */
/**
 * Opens a serial device for communication.
 *
 * @param devnum device number
 * @return handle to serial device if successful, NULL otherwise
 */
sio_fd_t sio_open(u8_t devnum)
{
  sio_fd_t sd;

/* USER CODE BEGIN 7 */
  sd = 0; // dummy code
/* USER CODE END 7 */

  return sd;
}

/**
 * Sends a single character to the serial device.
 *
 * @param c character to send
 * @param fd serial device handle
 *
 * @note This function will block until the character can be sent.
 */
void sio_send(u8_t c, sio_fd_t fd)
{
/* USER CODE BEGIN 8 */
/* USER CODE END 8 */
}

/**
 * Reads from the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 *
 * @note This function will block until data can be received. The blocking
 * can be cancelled by calling sio_read_abort().
 */
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 9 */
  recved_bytes = 0; // dummy code
/* USER CODE END 9 */
  return recved_bytes;
}

/**
 * Tries to read from the serial device. Same as sio_read but returns
 * immediately if no data is available and never blocks.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received
 */
u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 10 */
  recved_bytes = 0; // dummy code
/* USER CODE END 10 */
  return recved_bytes;
}
#endif /* MDK ARM Compiler */