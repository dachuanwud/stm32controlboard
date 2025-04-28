/*
 * 履带车的驱动程序，使用485通信方式。
 */

#include <stdio.h>
#include <stdlib.h>
#include "drv_real.h"
#include "main.h"

extern UART_HandleTypeDef huart3;
extern uint8_t g_485_rx_buf[];
extern uint8_t g_485_pt;

extern uint8_t bk_flag_left;
extern uint8_t bk_flag_right;

unsigned short CRC16_Modbus(unsigned char *data, unsigned int length)
{
  unsigned short crc = 0xFFFF;
  for (unsigned int i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/*
  发送数据：Data1 Data2 Data3 Data4 Data5 Data6 Data7 Data8
  Data1 为设备 ID 码，左侧电机为01，右侧电机为02；
  Data2 为操作码 0x06；
  Data3 为寄存器地址高字节，Data4 为寄存器地址低字节；
  Data5 为寄存器值高字节，Data6 为寄存器值低字节；
  Data7 为CRC 校验码低字节；Data8 为CRC 校验码高字节
*/
static void motor_control(int8_t speed_left, int8_t speed_right)
{
  uint8_t mt_cmd[8] = {0};

  mt_cmd[0] = 0x00; // 使用永久ID
  mt_cmd[1] = 0x06; // 操作码
  mt_cmd[2] = 0x00; // 地址
  mt_cmd[3] = 0x01; // 地址
  mt_cmd[4] = speed_left;
  mt_cmd[5] = speed_right;

  unsigned short crc = CRC16_Modbus(&mt_cmd[0], 6);
  mt_cmd[6] = crc & 0xff; // crc
  mt_cmd[7] = (crc >> 8) & 0xff; // crc

  HAL_UART_Transmit(&huart3, (uint8_t*)&mt_cmd, 8, 0xFFFF);    
}

/*
  根据目标值（来自遥控），结合当前值（来自编码器），得到设定值
 */
//static int8_t speed_adjust(int8_t sp_tgt, uint8_t id)
//{
//  // 获取当前转速
//  uint8_t mt_cmd[8] = {0};
//  mt_cmd[0] = id; // ID
//  mt_cmd[1] = 0x03; // 操作码
//  mt_cmd[2] = 0x02;
//  mt_cmd[3] = 0x04;
//  mt_cmd[4] = 0x00;
//  mt_cmd[5] = 0x01;
//  unsigned short crc = CRC16_Modbus(&mt_cmd[0], 6);
//  mt_cmd[6] = crc & 0xff; // crc
//  mt_cmd[7] = (crc >> 8) & 0xff; // crc
//  HAL_UART_Transmit(&huart3, mt_cmd, 8, 0xFFFF);   
//  
//  HAL_Delay(20);
//  // 数据解析
//  crc = CRC16_Modbus(g_485_rx_buf, 5);
//  if (((uint16_t)g_485_rx_buf[6] * 256 + g_485_rx_buf[5]) != crc) {
//    g_485_pt = 0;
//    return sp_tgt;
//  }

//  // 转速转成百分比
//  int8_t sp_cur = (int8_t)(((uint16_t)g_485_rx_buf[3] * 256 + g_485_rx_buf[4]) / 3000);
//  g_485_pt = 0;
//  
//  int8_t sp_set = 0;
//  if ((sp_tgt > 0 && sp_cur < 0) || (sp_tgt > 0 && sp_cur < 0)) {
//    // 反向
//    sp_set = sp_tgt - sp_cur;
//  } else {
//    if (abs(sp_cur) > abs(sp_tgt)) {
//      // 同向，cur比tgt快，包括tgt为0的场景
//      sp_set = sp_tgt - sp_cur + sp_tgt;
//    } else {
//      // 同向，cur比tgt慢或相等
//      sp_set = sp_tgt;
//    }
//  }
//  return sp_set; 
//}

/*
 * 配置左右速度实现运动
 * 取值-100到100，正值前进，负值后退
 */
uint8_t intf_move_real(int8_t speed_left, int8_t speed_right)
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
    bk_flag_right = 0;
  }

//  // 速度修正。自己做闭环，确保速度正确
//  speed_left = speed_adjust(speed_left, 1);
//  speed_right = speed_adjust(speed_right, 2);

  // 这里速度值的含义是：电机转速除以最高转速换算出的百分比。
  motor_control((-1) * speed_left, (-1) * speed_right);

  return 0; 
}
