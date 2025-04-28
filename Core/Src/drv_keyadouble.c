/*
 * 履带车的驱动程序，使用can通信方式。
 * 适配LKBLS481502双路电机驱动器
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "drv_keyadouble.h"

// 新驱动器CAN ID定义
#define DRIVER_ADDRESS       0x01        // 驱动器地址(默认为1)
#define DRIVER_TX_ID         0x06000000  // 发送基础ID (主机->驱动器)
#define DRIVER_RX_ID         0x05800000  // 接收基础ID (驱动器->主机)
#define DRIVER_HEARTBEAT_ID  0x07000000  // 心跳包ID (驱动器->主机)

// 电机通道定义
#define MOTOR_CHANNEL_A      0x01        // A路电机(左轮)
#define MOTOR_CHANNEL_B      0x02        // B路电机(右轮)

// 命令类型定义
#define CMD_ENABLE           0x01        // 使能电机
#define CMD_DISABLE          0x02        // 失能电机
#define CMD_SPEED            0x03        // 设置速度

extern CAN_HandleTypeDef hcan;
extern uint8_t bk_flag_left;
extern uint8_t bk_flag_right;
extern UART_HandleTypeDef huart5;

/**
 * 发送CAN数据
 * @param id CAN扩展ID
 * @param data 8字节数据
 */
static void keya_send_data(uint32_t id, uint8_t* data)
{
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.ExtId = id;  
  TxHeader.IDE = CAN_ID_EXT;             // 扩展帧(29位ID)
  TxHeader.RTR = CAN_RTR_DATA;           // 数据帧
  TxHeader.DLC = 8;                      // 帧长度8字节
  TxHeader.TransmitGlobalTime = DISABLE;
  
  uint32_t TxMailbox;

  HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox);
  
  // 打印发送的CAN数据(调试用)
  char buffer[50];
  sprintf(buffer, "CAN:%08lX:", id);
  HAL_UART_Transmit(&huart5, (uint8_t*)buffer, strlen(buffer), 100);
  for(int i=0; i<8; i++) {
    sprintf(buffer, "%02X ", data[i]);
    HAL_UART_Transmit(&huart5, (uint8_t*)buffer, strlen(buffer), 100);
  }
  HAL_UART_Transmit(&huart5, (uint8_t*)"\r\n", 2, 100);
  
  HAL_Delay(10); 
}

/**
 * 电机控制
 * @param cmd_type 命令类型: CMD_ENABLE/CMD_DISABLE/CMD_SPEED
 * @param channel 电机通道: MOTOR_CHANNEL_A(左)/MOTOR_CHANNEL_B(右)
 * @param speed 速度(-100到100，对应-10000到10000)
 */
static void motor_control(uint8_t cmd_type, uint8_t channel, int8_t speed)
{
  uint8_t TxData[8] = {0};
  uint32_t tx_id = DRIVER_TX_ID + DRIVER_ADDRESS;
  
  if (cmd_type == CMD_ENABLE) {
    // 使能电机: 23 0D 20 01/02 00 00 00 00
    TxData[0] = 0x23;
    TxData[1] = 0x0D;
    TxData[2] = 0x20;
    TxData[3] = channel;  // 01=A路(左轮), 02=B路(右轮)
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;
  } 
  else if (cmd_type == CMD_DISABLE) {
    // 失能电机: 23 0C 20 01/02 00 00 00 00
    TxData[0] = 0x23;
    TxData[1] = 0x0C;
    TxData[2] = 0x20;
    TxData[3] = channel;  // 01=A路(左轮), 02=B路(右轮)
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;
  }
  else if (cmd_type == CMD_SPEED) {
    // 设置速度: 23 00 20 01/02 HH HH LL LL
    TxData[0] = 0x23;
    TxData[1] = 0x00;
    TxData[2] = 0x20;
    TxData[3] = channel;  // 01=A路(左轮), 02=B路(右轮)
    
    // 将-100到100的速度转换为-10000到10000
    int32_t sp_value = (int32_t)speed * 100;
    
    // 32位有符号整数表示，低字节在后
    TxData[4] = (sp_value >> 24) & 0xFF;  // 最高字节
    TxData[5] = (sp_value >> 16) & 0xFF;
    TxData[6] = (sp_value >> 8) & 0xFF;
    TxData[7] = sp_value & 0xFF;         // 最低字节
  }

  keya_send_data(tx_id, TxData);
}

/**
 * 配置左右轮速度实现运动
 * @param speed_left 左轮速度(-100到100)
 * @param speed_right 右轮速度(-100到100)
 * @return 0=成功，1=参数错误
 */
uint8_t intf_move_keyadouble(int8_t speed_left, int8_t speed_right)
{
  if ((abs(speed_left) > 100) || (abs(speed_right) > 100)) {
    printf("Wrong parameter in intf_move!!![lf=%d],[ri=%d]", speed_left, speed_right);
    return 1;
  }

  // 左轮抱闸控制 (A路)
  if (speed_left != 0) {
    bk_flag_left = 1;
    HAL_GPIO_WritePin(GPIOA, left_bk_Pin, GPIO_PIN_SET); // 高电平，松抱闸，电机转
  } else {
    bk_flag_left = 0; // 0为抱闸，1为松开
  }
 
  // 右轮抱闸控制 (B路)
  if (speed_right != 0) {
    bk_flag_right = 1;
    HAL_GPIO_WritePin(GPIOA, right_bk_Pin, GPIO_PIN_SET);
  } else {
    bk_flag_right = 0; // 0为抱闸，1为松开
  }

  // 静态变量记录上次命令时间，保证看门狗不超时
  static uint32_t last_cmd_time = 0;
  uint32_t current_time = HAL_GetTick();
  
  // 如果距离上次命令超过900ms，需要重新使能电机
  if (current_time - last_cmd_time > 900) {
    motor_control(CMD_ENABLE, MOTOR_CHANNEL_A, 0);  // 使能A路(左轮)
    motor_control(CMD_ENABLE, MOTOR_CHANNEL_B, 0);  // 使能B路(右轮)
  }
  
  // 更新发送时间
  last_cmd_time = current_time;

  // 使能左右两路电机
  motor_control(CMD_ENABLE, MOTOR_CHANNEL_A, 0);  // 使能A路(左轮)
  motor_control(CMD_ENABLE, MOTOR_CHANNEL_B, 0);  // 使能B路(右轮)
  // 发送速度命令
  motor_control(CMD_SPEED, MOTOR_CHANNEL_A, speed_left);   // A路(左轮)速度
  motor_control(CMD_SPEED, MOTOR_CHANNEL_B, speed_right);  // B路(右轮)速度

  return 0; 
}
