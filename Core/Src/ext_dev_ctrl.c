#include "ext_dev_ctrl.h"
#include "can.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "logger.h"
#include "stm32f407xx.h"
static uint8_t T_Delay = 30;
/**
 * @brief TSJ设备控制功能
 * @note 通过UART3（485通道）发送预定义的数据序列
 */
void ext_dev_tsj_control(void) {
    //进入正常模式	82 C5 B8 33 B2 30 30 03 B4 35
    uint8_t Normal[]  = {0x82, 0xC5, 0xB8, 0x33, 0xB2, 0x30, 0x30, 0x03, 0xB4, 0x35};
    //一键启动	82 C5 B7 30 C3 30 30 03 35 B2
    uint8_t Start_BIN[]  = {0x82, 0xC5, 0xB7, 0x30, 0xC3, 0x30, 0x30, 0x03, 0x35, 0xB2};
    //一键启动结束	82 C5 B8 30 C3 30 30 03 35 33
    uint8_t Start_END[]  = {0x82, 0xC5, 0xB8, 0x30, 0xC3, 0x30, 0x30, 0x03, 0x35, 0x33};
    
    LOG_INFO("Sending TSJ control commands via UART3");
    
    // 控制动作
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, Normal, sizeof(Normal), 100);
    if(status != HAL_OK) {
        LOG_ERROR("Failed to send Normal command");
    }
    
    osDelay(100);
    
    status = HAL_UART_Transmit(&huart3, Start_BIN, sizeof(Start_BIN), 100);
    if(status != HAL_OK) {
        LOG_ERROR("Failed to send Start_BIN command");
    }
    
    osDelay(200);
    
    status = HAL_UART_Transmit(&huart3, Start_END, sizeof(Start_END), 100);
    if(status != HAL_OK) {
        LOG_ERROR("Failed to send Start_END command");
    }
    
    // osDelay(1000);
    
    LOG_INFO("TSJ control commands completed");
}

void ext_dev_saw_control(void) {
    /**
    圆锯机动作流程
    * 1、判断CAN1是否启动，未启动则启动CAN1并激活FIFO0中断
    * 2、控制台钳夹紧，CAN报文发送流程如下：
    *   01、清除错误:           2B 40 60 00 86 00
        02、力矩模式:           2F 60 60 00 04 00
        03、运行速度:           23 FF 60 00 D0 55 08 00
        04、目标力矩:           2B 71 60 00 90 01
        05、运行方向:           2F 7E 60 00 01 00
        06、设置使能:           2B 40 60 00 0F 00
        07、设置禁能:           2B 40 60 00 06 00
        08、位置模式:           2F 60 60 00 01 00
        09、目标位置:           23 7A 60 00 XX XX XX XX
        10、相对位置:           2F 60 60 00 4F 00 → 2F 60 60 00 5F 00
    * 3、控制圆锯机切割，控制继电器吸合1秒后断开
    */
    // 定义CAN通讯报文
    uint32_t CAN_ID = 0x601;  // 报文ID
    uint8_t Clear_Err[]     = {0x2B, 0x40, 0x60, 0x00, 0x86, 0x00}; // 清除错误
    uint8_t Enable[]        = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00}; // 使能
    uint8_t Mode_Troque[]   = {0x2F, 0x60, 0x60, 0x00, 0x04, 0x00}; // 力矩模式
    uint8_t Set_RPM[]       = {0x23, 0xFF, 0x60, 0x00, 0xAB, 0x2A, 0x04, 0x00}; // 运行速度100rpm  = 273067 = 42AAB
    uint8_t Set_Troque[]    = {0x2B, 0x71, 0x60, 0x00, 0x90, 0x01}; // 目标力矩(40%)
    uint8_t Set_DIR[]       = {0x2F, 0x7E, 0x60, 0x00, 0x00, 0x00}; // 运行方向
    uint8_t DISABLE[]       = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00}; // 禁能
    uint8_t Mode_Pos[]      = {0x2F, 0x60, 0x60, 0x00, 0x01, 0x00}; // 位置模式
    uint8_t Set_Pos[]       = {0x23, 0x7A, 0x60, 0x00, 0x3C, 0xF6, 0xFF, 0xFF}; // 目标位置(FFFFF63C = -90°)
    uint8_t Enable_Pos_1[]  = {0x2B, 0x40, 0x60, 0x00, 0x4F, 0x00}; // 相对位置控制使能报文1
    uint8_t Enable_Pos_2[]  = {0x2B, 0x40, 0x60, 0x00, 0x5F, 0x00}; // 相对位置控制使能报文2

    // 检查CAN1是否已启动，如果没有则启动它
    if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_READY || 
        HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING) {
    // CAN已经启动，无需重复启动
    } else {
        // 启动CAN1
        if (HAL_CAN_Start(&hcan1) != HAL_OK) {
            Error_Handler(); // CAN启动失败
        }
        
        // 激活CAN1 FIFO0中断
        if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
            Error_Handler();
        }
    }
    // 发送台钳夹紧报文
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Clear_Err, sizeof(Clear_Err));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, DISABLE, sizeof(DISABLE));
    osDelay(10);    
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Mode_Troque, sizeof(Mode_Troque));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Set_RPM, sizeof(Set_RPM));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Set_Troque, sizeof(Set_Troque));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Set_DIR, sizeof(Set_DIR));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Enable, sizeof(Enable));
    osDelay(3000);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Clear_Err, sizeof(Clear_Err));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, DISABLE, sizeof(DISABLE));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Mode_Pos, sizeof(Mode_Pos));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Set_Pos, sizeof(Set_Pos));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Enable_Pos_1, sizeof(Enable_Pos_1));
    osDelay(100);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Enable_Pos_2, sizeof(Enable_Pos_2));
    osDelay(1000);


    //控制继电器吸合，开始切割
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
    osDelay(100);
}

void ext_dev_saw_free(void) {

    // 定义CAN通讯报文
    uint32_t CAN_ID = 0x601;  // 报文ID
    uint8_t Clear_Err[]     = {0x2B, 0x40, 0x60, 0x00, 0x86, 0x00}; // 清除错误
    uint8_t Set_RPM[]       = {0x23, 0xFF, 0x60, 0x00, 0xAB, 0x2A, 0x04, 0x00}; // 运行速度100rpm  = 273067 = 42AAB
    uint8_t Set_DIR[]       = {0x2F, 0x7E, 0x60, 0x00, 0x00, 0x00}; // 运行方向
    uint8_t DISABLE[]       = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00}; // 禁能
    uint8_t Mode_Pos[]      = {0x2F, 0x60, 0x60, 0x00, 0x01, 0x00}; // 位置模式
    uint8_t Set_Pos[]       = {0x23, 0x7A, 0x60, 0x00, 0xA0, 0x15, 0xFF, 0xFF}; // 目标位置(FFFF15A0 = -2160°)
    uint8_t Enable_Pos_1[]  = {0x2B, 0x40, 0x60, 0x00, 0x4F, 0x00}; // 相对位置控制使能报文1
    uint8_t Enable_Pos_2[]  = {0x2B, 0x40, 0x60, 0x00, 0x5F, 0x00}; // 相对位置控制使能报文2

    // 检查CAN1是否已启动，如果没有则启动它
    if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_READY || 
        HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING) {
    // CAN已经启动，无需重复启动
    } else {
        // 启动CAN1
        if (HAL_CAN_Start(&hcan1) != HAL_OK) {
            Error_Handler(); // CAN启动失败
        }
        
        // 激活CAN1 FIFO0中断
        if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
            Error_Handler();
        }
    }
    // 发送台钳放松报文
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Clear_Err, sizeof(Clear_Err));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, DISABLE, sizeof(DISABLE));
    osDelay(10);    
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Mode_Pos, sizeof(Mode_Pos));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Set_RPM, sizeof(Set_RPM));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Set_Pos, sizeof(Set_Pos));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Set_DIR, sizeof(Set_DIR));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Enable_Pos_1, sizeof(Enable_Pos_1));
    osDelay(10);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, Enable_Pos_2, sizeof(Enable_Pos_2));
    osDelay(2000);
    CAN1_Transmit(CAN_ID, CAN_ID_STD, DISABLE, sizeof(DISABLE));
    osDelay(100);
}

// Modbus CRC16计算函数
static uint16_t modbus_crc16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Modbus命令生成函数
uint8_t* generate_modbus_command(uint8_t device_address, uint8_t func_code, uint16_t reg_address, 
                                uint16_t reg_value, uint8_t *cmd_length) {
    // Modbus命令格式: 设备地址 + 功能码 + 寄存器地址(2字节) + 数据(2字节) + CRC(2字节)
    // 总共需要8个字节
    static uint8_t modbus_cmd[8];
    
    modbus_cmd[0] = device_address;             // 设备地址
    modbus_cmd[1] = func_code;                  // 功能码
    modbus_cmd[2] = (reg_address >> 8) & 0xFF;  // 寄存器地址高字节
    modbus_cmd[3] = reg_address & 0xFF;         // 寄存器地址低字节
    modbus_cmd[4] = (reg_value >> 8) & 0xFF;    // 数据高字节
    modbus_cmd[5] = reg_value & 0xFF;           // 数据低字节
    
    // 计算CRC16校验
    uint16_t crc = modbus_crc16(modbus_cmd, 6);
    modbus_cmd[6] = crc & 0xFF;                 // CRC低字节
    modbus_cmd[7] = (crc >> 8) & 0xFF;          // CRC高字节
    
    *cmd_length = 8;
    return modbus_cmd;
}


void ext_dev_free_control(void) {
    //使能OFF
    uint8_t Disable_2[]  = {0x02, 0x06, 0x31, 0x00, 0x00, 0x06, 0x07, 0x07};
    uint8_t Disable_1[]  = {0x01, 0x06, 0x31, 0x00, 0x00, 0x06, 0x07, 0x34};
    //工作模式为力矩模式
    uint8_t Mode_Set_2[]  = {0x02, 0x06, 0x35, 0x00, 0x00, 0x04, 0x87, 0xF6};
    uint8_t Mode_Set_1[]  = {0x01, 0x06, 0x35, 0x00, 0x00, 0x04, 0x87, 0xC5};
    //力矩模式下力矩值为5%
    uint8_t Tr_Set_1[]  = {0x01, 0x06, 0x3C, 0x00, 0x01, 0x00, 0x84, 0x0A};
    uint8_t Tr_Set_2[]  = {0x02, 0x06, 0x3C, 0x00, 0x01, 0x00, 0x84, 0x39};
    //最高速度200rpm
    uint8_t RPM_Set_2[]  = {0x02, 0x10, 0x6F, 0x00, 0x00, 0x02, 0x04, 0x55, 0x55, 0x00, 0x08, 0x15, 0x03};
    uint8_t RPM_Set_1[]  = {0x01, 0x10, 0x6F, 0x00, 0x00, 0x02, 0x04, 0x55, 0x55, 0x00, 0x08, 0x1A, 0x47};
    //配置运行方向
    uint8_t DIR_Set_2[]  = {0x02, 0x06, 0x47, 0x00, 0x00, 0x00, 0x9D, 0x4D};
    uint8_t DIR_Set_1[]  = {0x01, 0x06, 0x47, 0x00, 0x00, 0x00, 0x9D, 0x7E};
    //使能ON
    uint8_t Enable_1[]   = {0x01, 0x06, 0x31, 0x00, 0x00, 0x0F, 0xC7, 0x32};  
    uint8_t Enable_2[]   = {0x02, 0x06, 0x31, 0x00, 0x00, 0x0F, 0xC7, 0x01};
    //清除错误
    uint8_t Clr_Err_1[] = {0x01, 0x06, 0x31, 0x00, 0x00, 0x86, 0x06, 0x94};
    uint8_t Clr_Err_2[] = {0x02, 0x06, 0x31, 0x00, 0x00, 0x86, 0x06, 0xA7};


    // 清除错误
    HAL_UART_Transmit(&huart4, Clr_Err_1, sizeof(Clr_Err_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, Clr_Err_2, sizeof(Clr_Err_2), 100);
    osDelay(T_Delay);
    //使能OFF
    HAL_UART_Transmit(&huart4, Disable_1, sizeof(Disable_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, Disable_2, sizeof(Disable_2), 100);
    osDelay(T_Delay);
    //工作模式为力矩模式
    HAL_UART_Transmit(&huart4, Mode_Set_1, sizeof(Mode_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, Mode_Set_2, sizeof(Mode_Set_2), 100);
    osDelay(T_Delay);
    //力矩模式下力矩值为5%
    HAL_UART_Transmit(&huart4, Tr_Set_1, sizeof(Tr_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, Tr_Set_2, sizeof(Tr_Set_2), 100);
    osDelay(T_Delay);
    //最高速度200rpm
    HAL_UART_Transmit(&huart4, RPM_Set_1, sizeof(RPM_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, RPM_Set_2, sizeof(RPM_Set_2), 100);
    osDelay(T_Delay);
    //配置运行方向
    HAL_UART_Transmit(&huart4, DIR_Set_1, sizeof(DIR_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, DIR_Set_2, sizeof(DIR_Set_2), 100);
    osDelay(T_Delay);
    //使能ON
    HAL_UART_Transmit(&huart4, Enable_1, sizeof(Enable_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, Enable_2, sizeof(Enable_2), 100);

}
void ext_dev_clmp_control(void) {
    //使能OFF
    uint8_t Disable_2[]  = {0x02, 0x06, 0x31, 0x00, 0x00, 0x06, 0x07, 0x07};
    uint8_t Disable_1[]  = {0x01, 0x06, 0x31, 0x00, 0x00, 0x06, 0x07, 0x34};
    //工作模式为力矩模式
    uint8_t Mode_Set_2[]  = {0x02, 0x06, 0x35, 0x00, 0x00, 0x04, 0x87, 0xF6};
    uint8_t Mode_Set_1[]  = {0x01, 0x06, 0x35, 0x00, 0x00, 0x04, 0x87, 0xC5};
    //力矩模式下力矩值为5%
    //01 06 3C 00 01 00 9B 9A  
    //02 06 3C 00 01 00 9B A9 
    uint8_t Tr_Set_1[]  = {0x01, 0x06, 0x3C, 0x00, 0x01, 0x00, 0x84, 0x0A};
    uint8_t Tr_Set_2[]  = {0x02, 0x06, 0x3C, 0x00, 0x01, 0x00, 0x84, 0x39};
    // uint8_t Tr_Set_2[]  = {0x02, 0x06, 0x3C, 0x00, 0x05, 0x00, 0x86, 0xF9};
    // uint8_t Tr_Set_1[]  = {0x01, 0x06, 0x3C, 0x00, 0x05, 0x00, 0x86, 0xCA};
    //最高速度200rpm
    //01 10 6F 00 00 02 04 15 55 00 02 8F 80
    //02 10 6F 00 00 02 04 15 55 00 02 80 C4
    uint8_t RPM_Set_1[]  = {0x01, 0x10, 0x6F, 0x00, 0x00, 0x02, 0x04, 0x15, 0x00, 0x00, 0x02, 0x8F, 0x80};
    uint8_t RPM_Set_2[]  = {0x02, 0x10, 0x6F, 0x00, 0x00, 0x02, 0x04, 0x15, 0x55, 0x00, 0x02, 0x80, 0xc4};
    // uint8_t RPM_Set_2[]  = {0x02, 0x10, 0x6F, 0x00, 0x00, 0x02, 0x04, 0x55, 0x55, 0x00, 0x08, 0x15, 0x03};
    // uint8_t RPM_Set_1[]  = {0x01, 0x10, 0x6F, 0x00, 0x00, 0x02, 0x04, 0x55, 0x55, 0x00, 0x08, 0x1A, 0x47};
    //配置运行方向
    uint8_t DIR_Set_2[]  = {0x02, 0x06, 0x47, 0x00, 0x00, 0x01, 0x5C, 0x8D};
    uint8_t DIR_Set_1[]  = {0x01, 0x06, 0x47, 0x00, 0x00, 0x01, 0x5C, 0xBE};
    //使能ON
    uint8_t Enable_1[]   = {0x01, 0x06, 0x31, 0x00, 0x00, 0x0F, 0xC7, 0x32};  
    uint8_t Enable_2[]   = {0x02, 0x06, 0x31, 0x00, 0x00, 0x0F, 0xC7, 0x01};
    //清除错误
    uint8_t Clr_Err_1[] = {0x01, 0x06, 0x31, 0x00, 0x00, 0x86, 0x06, 0x94};
    uint8_t Clr_Err_2[] = {0x02, 0x06, 0x31, 0x00, 0x00, 0x86, 0x06, 0xA7};

    // 清除错误
    HAL_UART_Transmit(&huart4, Clr_Err_1, sizeof(Clr_Err_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, Clr_Err_2, sizeof(Clr_Err_2), 100);
    osDelay(T_Delay);
    //使能OFF
    HAL_UART_Transmit(&huart4, Disable_1, sizeof(Disable_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, Disable_2, sizeof(Disable_2), 100);
    osDelay(T_Delay);
    //工作模式为力矩模式
    HAL_UART_Transmit(&huart4, Mode_Set_1, sizeof(Mode_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, Mode_Set_2, sizeof(Mode_Set_2), 100);
    osDelay(T_Delay);
    //力矩模式下力矩值为5%
    HAL_UART_Transmit(&huart4, Tr_Set_1, sizeof(Tr_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, Tr_Set_2, sizeof(Tr_Set_2), 100);
    osDelay(T_Delay);
    //最高速度200rpm
    HAL_UART_Transmit(&huart4, RPM_Set_1, sizeof(RPM_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, RPM_Set_2, sizeof(RPM_Set_2), 100);
    osDelay(T_Delay);
    //配置运行方向
    HAL_UART_Transmit(&huart4, DIR_Set_1, sizeof(DIR_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, DIR_Set_2, sizeof(DIR_Set_2), 100);
    osDelay(T_Delay);
    //使能ON
    HAL_UART_Transmit(&huart4, Enable_1, sizeof(Enable_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart4, Enable_2, sizeof(Enable_2), 100);

}
// void ext_dev_cut_control(void) {
//     // 使能OFF
//     // 07 06 31 00 00 06 07 52 
//     uint8_t Disable[]  = {0x07, 0x06, 0x31, 0x00, 0x00, 0x06, 0x07, 0x52};
//     // 工作模式为力矩模式
//     // 07 06 35 00 00 04 87 A3 
//     uint8_t Mode_Set[]  = {0x07, 0x06, 0x35, 0x00, 0x00, 0x04, 0x87, 0xA3};
//     // 力矩模式下力矩值为20%
//     // 07 06 3C 00 00 C8 84 6A 
//     uint8_t Tr_Set[]  = {0x07, 0x06, 0x3C, 0x00, 0x00, 0xC8, 0x84, 0x6A};
//     // uint8_t Tr_Set_2[]  = {0x02, 0x06, 0x3C, 0x00, 0x05, 0x00, 0x86, 0xF9};
//     // 最高速度200rpm
//     // 07 10 6F 00 00 02 04 2A AA 00 04 2D 2E
//     uint8_t RPM_Set[]  = {0x07, 0x10, 0x6F, 0x00, 0x00, 0x02, 0x04, 0x2A, 0xAA, 0x00, 0x04, 0x2D, 0x2E};
//     // 配置运行方向
//     // 07 06 47 00 00 00 9D 18 
//     // 07 06 47 00 00 01 5C D8 
//     uint8_t DIR_Set_1[]  = {0x07, 0x06, 0x47, 0x00, 0x00, 0x00, 0x9D, 0x18};
//     uint8_t DIR_Set_2[]  = {0x07, 0x06, 0x47, 0x00, 0x00, 0x01, 0x5C, 0xD8};
//     // 使能ON
//     // 07 06 31 00 00 0F C7 54 
//     uint8_t Enable[]   = {0x07, 0x06, 0x31, 0x00, 0x00, 0x0F, 0xC7, 0x54};  
//     // 清除错误
//     // 07 06 31 00 00 86 06 F2 
//     uint8_t Clr_Err[] = {0x07, 0x06, 0x31, 0x00, 0x00, 0x86, 0x06, 0xF2};

//     // 清除错误
//     HAL_UART_Transmit(&huart4, Clr_Err, sizeof(Clr_Err), 100);
//     osDelay(T_Delay);
//     HAL_UART_Transmit(&huart4, Disable, sizeof(Disable), 100);
//     osDelay(T_Delay);
//     HAL_UART_Transmit(&huart4, Mode_Set, sizeof(Mode_Set), 100);
//     osDelay(T_Delay);
//     HAL_UART_Transmit(&huart4, Tr_Set, sizeof(Tr_Set), 100);
//     osDelay(T_Delay);
//     HAL_UART_Transmit(&huart4, RPM_Set, sizeof(RPM_Set), 100);
//     osDelay(T_Delay);
//     HAL_UART_Transmit(&huart4, DIR_Set_2, sizeof(DIR_Set_2), 100);
//     osDelay(T_Delay);
//     HAL_UART_Transmit(&huart4, Enable, sizeof(Enable), 100);
//     osDelay(1000);
//     HAL_UART_Transmit(&huart4, Clr_Err, sizeof(Clr_Err), 100);
//     osDelay(T_Delay);
//     HAL_UART_Transmit(&huart4, Disable, sizeof(Disable), 100);
//     osDelay(T_Delay);
//     HAL_UART_Transmit(&huart4, Mode_Set, sizeof(Mode_Set), 100);
//     osDelay(T_Delay);
//     HAL_UART_Transmit(&huart4, Tr_Set, sizeof(Tr_Set), 100);
//     osDelay(T_Delay);
//     HAL_UART_Transmit(&huart4, RPM_Set, sizeof(RPM_Set), 100);
//     osDelay(T_Delay);
//     HAL_UART_Transmit(&huart4, DIR_Set_1, sizeof(DIR_Set_1), 100);
//     osDelay(T_Delay);
//     HAL_UART_Transmit(&huart4, Enable, sizeof(Enable), 100);
// }
void ext_dev_cut_control(void) {
    // 检查CAN1是否已启动，如果没有则启动它
    if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_READY || 
        HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING) {
        // CAN已经启动，无需重复启动
    } else {
        // 启动CAN1
        if (HAL_CAN_Start(&hcan1) != HAL_OK) {
            Error_Handler(); // CAN启动失败
        }
        
        // 激活CAN1 FIFO0中断
        if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
            Error_Handler();
        }
    }
    
    // 控制指令数据场
    // 01. 使能，2B 40 60 00 0F 00
    uint8_t Enable[]   = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00};
    // 02. 运行模式，2F 60 60 00 04 00
    uint8_t Mode[]   = {0x2F, 0x60, 0x60, 0x00, 0x04, 0x00};
    // 03. 运行方向，2F 7E 60 00 01 00
    uint8_t DIR_CW[]   = {0x2F, 0x7E, 0x60, 0x00, 0x01, 0x00};
    uint8_t DIR_CCW[]   = {0x2F, 0x7E, 0x60, 0x00, 0x00, 0x00};
    // 04. 运行速度，23 FF 60 00 D0 55 08 00
    uint8_t RPM_Set[]  = {0x23, 0xFF, 0x60, 0x00, 0xD0, 0x55, 0x08, 0x00};
    // 05. 运行力矩，2B 71 60 00 C8 00
    uint8_t Tr_Set[]  = {0x2B, 0x71, 0x60, 0x00, 0x64, 0x00};
    // 06. 清除错误，2B 40 60 00 86 00
    uint8_t Clr_Err[] = {0x2B, 0x40, 0x60, 0x00, 0x86, 0x00};
    // 07. 禁用，2B 40 60 00 06 00
    uint8_t Disable[] = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00};
    
    // 使用标准ID 0x601 发送控制命令
    uint32_t motor_control_id = 0x601; // 标准ID
    
    // 执行裁断动作，清除错误>>禁用>>运行模式>>运行方向>>运行速度>>运行力矩>>使能>>等待1秒>>禁用>>运行模式>>运行方向反向>>运行速度>>运行力矩>>使能
    CAN1_Transmit(motor_control_id, CAN_ID_STD, Clr_Err, sizeof(Clr_Err));
    osDelay(20);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, Disable, sizeof(Disable));
    osDelay(20);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, Mode, sizeof(Mode));
    osDelay(20);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, DIR_CW, sizeof(DIR_CW));
    osDelay(20);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, RPM_Set, sizeof(RPM_Set));
    osDelay(20);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, Tr_Set, sizeof(Tr_Set));
    osDelay(20);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, Enable, sizeof(Enable));
    osDelay(1000);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, Clr_Err, sizeof(Clr_Err));
    osDelay(20);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, Disable, sizeof(Disable));
    osDelay(20);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, Mode, sizeof(Mode));
    osDelay(20);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, DIR_CCW, sizeof(DIR_CCW));
    osDelay(20);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, RPM_Set, sizeof(RPM_Set));
    osDelay(20);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, Tr_Set, sizeof(Tr_Set));
    osDelay(20);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, Enable, sizeof(Enable));
    osDelay(1000);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, Clr_Err, sizeof(Clr_Err));
    osDelay(20);
    CAN1_Transmit(motor_control_id, CAN_ID_STD, Disable, sizeof(Disable));
}
/**
 * @brief  发送CAN消息
 * @param  hcan: CAN句柄指针
 * @param  id: 消息ID (标准ID或扩展ID)
 * @param  ide: 标识符类型 (CAN_ID_STD 或 CAN_ID_EXT)
 * @param  rtr: 远程传输请求 (CAN_RTR_DATA 或 CAN_RTR_REMOTE)
 * @param  data: 要发送的数据数组 (最大8字节)
 * @param  len: 数据长度 (0-8)
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN_SendMessage(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t ide, uint32_t rtr, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    
    // 配置发送消息头
    if (ide == CAN_ID_STD) 
    {
        TxHeader.StdId = id;      // 标准ID (11位)
        TxHeader.IDE = CAN_ID_STD;
    } 
    else 
    {
        TxHeader.ExtId = id;      // 扩展ID (29位)
        TxHeader.IDE = CAN_ID_EXT;
    }
    
    TxHeader.RTR = rtr;           // 数据帧还是远程帧
    TxHeader.DLC = len;           // 数据长度
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // 发送消息
    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox) != HAL_OK)
    {
        return HAL_ERROR; // 发送失败
    }
    
    return HAL_OK; // 发送成功
}

/**
 * @brief  使用CAN1发送数据
 * @param  id: 消息ID
 * @param  ide: 标识符类型 (CAN_ID_STD 或 CAN_ID_EXT)
 * @param  data: 要发送的数据数组
 * @param  len: 数据长度 (最大8字节)
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN1_Transmit(uint32_t id, uint32_t ide, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    
    // 配置发送消息头
    if (ide == CAN_ID_STD) 
    {
        TxHeader.StdId = id;      // 标准ID (11位)
        TxHeader.IDE = CAN_ID_STD;
    } 
    else 
    {
        TxHeader.ExtId = id;      // 扩展ID (29位)
        TxHeader.IDE = CAN_ID_EXT;
    }
    
    TxHeader.RTR = CAN_RTR_DATA;  // 数据帧
    TxHeader.DLC = len;          // 数据长度
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // 确保长度不超过8字节
    if (len > 8) len = 8;
    
    // 发送消息
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK)
    {
        return HAL_ERROR; // 发送失败
    }
    
    return HAL_OK; // 发送成功
}

/**
 * @brief  使用CAN2发送数据
 * @param  id: 消息ID
 * @param  ide: 标识符类型 (CAN_ID_STD 或 CAN_ID_EXT)
 * @param  data: 要发送的数据数组
 * @param  len: 数据长度 (最大8字节)
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN2_Transmit(uint32_t id, uint32_t ide, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    
    // 配置发送消息头
    if (ide == CAN_ID_STD) 
    {
        TxHeader.StdId = id;      // 标准ID (11位)
        TxHeader.IDE = CAN_ID_STD;
    } 
    else 
    {
        TxHeader.ExtId = id;      // 扩展ID (29位)
        TxHeader.IDE = CAN_ID_EXT;
    }
    
    TxHeader.RTR = CAN_RTR_DATA;  // 数据帧
    TxHeader.DLC = len;          // 数据长度
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // 确保长度不超过8字节
    if (len > 8) len = 8;
    
    // 发送消息
    if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &TxMailbox) != HAL_OK)
    {
        return HAL_ERROR; // 发送失败
    }
    
    return HAL_OK; // 发送成功
}

/**
 * @brief  使用CAN1发送标准数据帧
 * @param  std_id: 标准ID (11位, 范围: 0-0x7FF)
 * @param  data: 要发送的数据数组
 * @param  len: 数据长度 (最大8字节)
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN1_SendStdData(uint32_t std_id, uint8_t *data, uint8_t len)
{
    // 确保长度不超过8字节
    if (len > 8) len = 8;
    
    return CAN_SendMessage(&hcan1, std_id, CAN_ID_STD, CAN_RTR_DATA, data, len);
}

/**
 * @brief  使用CAN1发送扩展数据帧
 * @param  ext_id: 扩展ID (29位, 范围: 0-0x1FFFFFFF)
 * @param  data: 要发送的数据数组
 * @param  len: 数据长度 (最大8字节)
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN1_SendExtData(uint32_t ext_id, uint8_t *data, uint8_t len)
{
    // 确保长度不超过8字节
    if (len > 8) len = 8;
    
    return CAN_SendMessage(&hcan1, ext_id, CAN_ID_EXT, CAN_RTR_DATA, data, len);
}

/**
 * @brief  使用CAN2发送标准数据帧
 * @param  std_id: 标准ID (11位, 范围: 0-0x7FF)
 * @param  data: 要发送的数据数组
 * @param  len: 数据长度 (最大8字节)
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN2_SendStdData(uint32_t std_id, uint8_t *data, uint8_t len)
{
    // 确保长度不超过8字节
    if (len > 8) len = 8;
    
    return CAN_SendMessage(&hcan2, std_id, CAN_ID_STD, CAN_RTR_DATA, data, len);
}

/**
 * @brief  使用CAN2发送扩展数据帧
 * @param  ext_id: 扩展ID (29位, 范围: 0-0x1FFFFFFF)
 * @param  data: 要发送的数据数组
 * @param  len: 数据长度 (最大8字节)
 * @retval HAL状态
 */
HAL_StatusTypeDef CAN2_SendExtData(uint32_t ext_id, uint8_t *data, uint8_t len)
{
    // 确保长度不超过8字节
    if (len > 8) len = 8;
    
    return CAN_SendMessage(&hcan2, ext_id, CAN_ID_EXT, CAN_RTR_DATA, data, len);
}

/* USER CODE END 1 */
