/*
 * �Ĵ�������������ʹ��485ͨ�ŷ�ʽ��
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
  �������ݣ�Data1 Data2 Data3 Data4 Data5 Data6 Data7 Data8
  Data1 Ϊ�豸 ID �룬�����Ϊ01���Ҳ���Ϊ02��
  Data2 Ϊ������ 0x06��
  Data3 Ϊ�Ĵ�����ַ���ֽڣ�Data4 Ϊ�Ĵ�����ַ���ֽڣ�
  Data5 Ϊ�Ĵ���ֵ���ֽڣ�Data6 Ϊ�Ĵ���ֵ���ֽڣ�
  Data7 ΪCRC У������ֽڣ�Data8 ΪCRC У������ֽ�
*/
static void motor_control(int8_t speed_left, int8_t speed_right)
{
  uint8_t mt_cmd[8] = {0};

  mt_cmd[0] = 0x00; // ʹ������ID
  mt_cmd[1] = 0x06; // ������
  mt_cmd[2] = 0x00; // ��ַ
  mt_cmd[3] = 0x01; // ��ַ
  mt_cmd[4] = speed_left;
  mt_cmd[5] = speed_right;

  unsigned short crc = CRC16_Modbus(&mt_cmd[0], 6);
  mt_cmd[6] = crc & 0xff; // crc
  mt_cmd[7] = (crc >> 8) & 0xff; // crc

  HAL_UART_Transmit(&huart3, (uint8_t*)&mt_cmd, 8, 0xFFFF);    
}

/*
  ����Ŀ��ֵ������ң�أ�����ϵ�ǰֵ�����Ա����������õ��趨ֵ
 */
//static int8_t speed_adjust(int8_t sp_tgt, uint8_t id)
//{
//  // ��ȡ��ǰת��
//  uint8_t mt_cmd[8] = {0};
//  mt_cmd[0] = id; // ID
//  mt_cmd[1] = 0x03; // ������
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
//  // ���ݽ���
//  crc = CRC16_Modbus(g_485_rx_buf, 5);
//  if (((uint16_t)g_485_rx_buf[6] * 256 + g_485_rx_buf[5]) != crc) {
//    g_485_pt = 0;
//    return sp_tgt;
//  }

//  // ת��ת�ɰٷֱ�
//  int8_t sp_cur = (int8_t)(((uint16_t)g_485_rx_buf[3] * 256 + g_485_rx_buf[4]) / 3000);
//  g_485_pt = 0;
//  
//  int8_t sp_set = 0;
//  if ((sp_tgt > 0 && sp_cur < 0) || (sp_tgt > 0 && sp_cur < 0)) {
//    // ����
//    sp_set = sp_tgt - sp_cur;
//  } else {
//    if (abs(sp_cur) > abs(sp_tgt)) {
//      // ͬ��cur��tgt�죬����tgtΪ0�ĳ���
//      sp_set = sp_tgt - sp_cur + sp_tgt;
//    } else {
//      // ͬ��cur��tgt�������
//      sp_set = sp_tgt;
//    }
//  }
//  return sp_set; 
//}

/*
 * ���������ٶ�ʵ���˶�
 * ȡֵ-100��100����ֵǰ������ֵ����
 */
uint8_t intf_move_real(int8_t speed_left, int8_t speed_right)
{
  if ((abs(speed_left) > 100) || (abs(speed_right) > 100)) {
    printf("Wrong parameter in intf_move!!![lf=%d],[ri=%d]", speed_left, speed_right);
    return 1;
  }

  if (speed_left != 0) {
    bk_flag_left = 1;
    HAL_GPIO_WritePin(GPIOA, left_bk_Pin, GPIO_PIN_SET); // �ߵ�ƽ���ɱ�բ�����ת
  } else {
    bk_flag_left = 0; // 0Ϊ��բ��1Ϊ�ɿ�
  }
 
  if (speed_right != 0) {
    bk_flag_right = 1;
    HAL_GPIO_WritePin(GPIOA, right_bk_Pin, GPIO_PIN_SET);
  } else {
    bk_flag_right = 0;
  }

//  // �ٶ��������Լ����ջ���ȷ���ٶ���ȷ
//  speed_left = speed_adjust(speed_left, 1);
//  speed_right = speed_adjust(speed_right, 2);

  // �����ٶ�ֵ�ĺ����ǣ����ת�ٳ������ת�ٻ�����İٷֱȡ�
  motor_control((-1) * speed_left, (-1) * speed_right);

  return 0; 
}
