/*
 * 履带车的驱动程序，使用can通信方式。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "drv_keya.h"
#include "main.h"


extern CAN_HandleTypeDef hcan;
extern uint8_t bk_flag_left;
extern uint8_t bk_flag_right;
extern UART_HandleTypeDef huart5;

static void keya_send_data(uint32_t id, uint8_t* data)
{
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.ExtId = id;  
  TxHeader.IDE = CAN_ID_EXT; // 帧格式，扩展帧
  TxHeader.RTR = CAN_RTR_DATA; // 帧类型，数据帧
  TxHeader.DLC = 8; // 帧长度
  TxHeader.TransmitGlobalTime = DISABLE;
  
  uint32_t TxMailbox;

  HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox);
  
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

#define KEYA_ENABLE 0
#define KEYA_SET_SP 1
static void motor_control(uint32_t id, uint8_t type, int8_t speed)
{
  uint8_t TxData[8] = {0x23, 0x0d, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00}; // 默认使能数据

  if (type == KEYA_SET_SP) {
    TxData[1] = 0x00;
    
    uint16_t sp = (uint16_t)speed * 10; // *10，转换成-1000到1000
    TxData[4] = sp & 0xff;
    TxData[5] = (sp >> 8) & 0xff;
  } 

  keya_send_data(id, TxData);
}

/*
 * 配置左右速度实现运动
 * 取值-100到100，正值前进，负值后退
 */
uint8_t intf_move_keya(int8_t speed_left, int8_t speed_right)
{
  if ((abs(speed_left) > 100) || (abs(speed_right) > 100)) {
    printf("Wrong parameter in intf_move!!![lf=%d],[ri=%d]", speed_left, speed_right);
    return 1;
  }

  if (speed_left != 0) {
    bk_flag_left = 1;
    HAL_GPIO_WritePin(GPIOA, left_bk_Pin, GPIO_PIN_SET); // 高电平，松抱闸，电机转
  } else {
    bk_flag_left = 0; // 0为抱闸，1为松开
  }
 
  if (speed_right != 0) {
    bk_flag_right = 1;
    HAL_GPIO_WritePin(GPIOA, right_bk_Pin, GPIO_PIN_SET);
  } else {
    bk_flag_right = 0; // 0为抱闸，1为松开
  }

  motor_control(0x601, KEYA_ENABLE, 0); // 左使能
  motor_control(0x602, KEYA_ENABLE, 0); // 右使能
  motor_control(0x601, KEYA_SET_SP, speed_left); // 左速度
  motor_control(0x602, KEYA_SET_SP, speed_right); // 右速度

  return 0; 
}
