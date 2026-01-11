#include "ext_dev_ctrl.h"
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
    //控制继电器吸合，开始切割
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
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
    HAL_UART_Transmit(&huart3, Clr_Err_1, sizeof(Clr_Err_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Clr_Err_2, sizeof(Clr_Err_2), 100);
    osDelay(T_Delay);
    //使能OFF
    HAL_UART_Transmit(&huart3, Disable_1, sizeof(Disable_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Disable_2, sizeof(Disable_2), 100);
    osDelay(T_Delay);
    //工作模式为力矩模式
    HAL_UART_Transmit(&huart3, Mode_Set_1, sizeof(Mode_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Mode_Set_2, sizeof(Mode_Set_2), 100);
    osDelay(T_Delay);
    //力矩模式下力矩值为5%
    HAL_UART_Transmit(&huart3, Tr_Set_1, sizeof(Tr_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Tr_Set_2, sizeof(Tr_Set_2), 100);
    osDelay(T_Delay);
    //最高速度200rpm
    HAL_UART_Transmit(&huart3, RPM_Set_1, sizeof(RPM_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, RPM_Set_2, sizeof(RPM_Set_2), 100);
    osDelay(T_Delay);
    //配置运行方向
    HAL_UART_Transmit(&huart3, DIR_Set_1, sizeof(DIR_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, DIR_Set_2, sizeof(DIR_Set_2), 100);
    osDelay(T_Delay);
    //使能ON
    HAL_UART_Transmit(&huart3, Enable_1, sizeof(Enable_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Enable_2, sizeof(Enable_2), 100);

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
    HAL_UART_Transmit(&huart3, Clr_Err_1, sizeof(Clr_Err_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Clr_Err_2, sizeof(Clr_Err_2), 100);
    osDelay(T_Delay);
    //使能OFF
    HAL_UART_Transmit(&huart3, Disable_1, sizeof(Disable_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Disable_2, sizeof(Disable_2), 100);
    osDelay(T_Delay);
    //工作模式为力矩模式
    HAL_UART_Transmit(&huart3, Mode_Set_1, sizeof(Mode_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Mode_Set_2, sizeof(Mode_Set_2), 100);
    osDelay(T_Delay);
    //力矩模式下力矩值为5%
    HAL_UART_Transmit(&huart3, Tr_Set_1, sizeof(Tr_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Tr_Set_2, sizeof(Tr_Set_2), 100);
    osDelay(T_Delay);
    //最高速度200rpm
    HAL_UART_Transmit(&huart3, RPM_Set_1, sizeof(RPM_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, RPM_Set_2, sizeof(RPM_Set_2), 100);
    osDelay(T_Delay);
    //配置运行方向
    HAL_UART_Transmit(&huart3, DIR_Set_1, sizeof(DIR_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, DIR_Set_2, sizeof(DIR_Set_2), 100);
    osDelay(T_Delay);
    //使能ON
    HAL_UART_Transmit(&huart3, Enable_1, sizeof(Enable_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Enable_2, sizeof(Enable_2), 100);

}
void ext_dev_cut_control(void) {
    // 使能OFF
    // 07 06 31 00 00 06 07 52 
    uint8_t Disable[]  = {0x07, 0x06, 0x31, 0x00, 0x00, 0x06, 0x07, 0x52};
    // 工作模式为力矩模式
    // 07 06 35 00 00 04 87 A3 
    uint8_t Mode_Set[]  = {0x07, 0x06, 0x35, 0x00, 0x00, 0x04, 0x87, 0xA3};
    // 力矩模式下力矩值为20%
    // 07 06 3C 00 00 C8 84 6A 
    uint8_t Tr_Set[]  = {0x07, 0x06, 0x3C, 0x00, 0x00, 0xC8, 0x84, 0x6A};
    // uint8_t Tr_Set_2[]  = {0x02, 0x06, 0x3C, 0x00, 0x05, 0x00, 0x86, 0xF9};
    // 最高速度200rpm
    // 07 10 6F 00 00 02 04 2A AA 00 04 2D 2E
    uint8_t RPM_Set[]  = {0x07, 0x10, 0x6F, 0x00, 0x00, 0x02, 0x04, 0x2A, 0xAA, 0x00, 0x04, 0x2D, 0x2E};
    // 配置运行方向
    // 07 06 47 00 00 00 9D 18 
    // 07 06 47 00 00 01 5C D8 
    uint8_t DIR_Set_1[]  = {0x07, 0x06, 0x47, 0x00, 0x00, 0x00, 0x9D, 0x18};
    uint8_t DIR_Set_2[]  = {0x07, 0x06, 0x47, 0x00, 0x00, 0x01, 0x5C, 0xD8};
    // 使能ON
    // 07 06 31 00 00 0F C7 54 
    uint8_t Enable[]   = {0x07, 0x06, 0x31, 0x00, 0x00, 0x0F, 0xC7, 0x54};  
    // 清除错误
    // 07 06 31 00 00 86 06 F2 
    uint8_t Clr_Err[] = {0x07, 0x06, 0x31, 0x00, 0x00, 0x86, 0x06, 0xF2};

    // 清除错误
    HAL_UART_Transmit(&huart3, Clr_Err, sizeof(Clr_Err), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Disable, sizeof(Disable), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Mode_Set, sizeof(Mode_Set), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Tr_Set, sizeof(Tr_Set), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, RPM_Set, sizeof(RPM_Set), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, DIR_Set_2, sizeof(DIR_Set_2), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Enable, sizeof(Enable), 100);
    osDelay(1000);
    HAL_UART_Transmit(&huart3, Clr_Err, sizeof(Clr_Err), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Disable, sizeof(Disable), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Mode_Set, sizeof(Mode_Set), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Tr_Set, sizeof(Tr_Set), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, RPM_Set, sizeof(RPM_Set), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, DIR_Set_1, sizeof(DIR_Set_1), 100);
    osDelay(T_Delay);
    HAL_UART_Transmit(&huart3, Enable, sizeof(Enable), 100);
}