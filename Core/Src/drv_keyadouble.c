/*
 * �Ĵ�������������ʹ��canͨ�ŷ�ʽ��
 * ����LKBLS481502˫·���������
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "drv_keyadouble.h"

// ��������CAN ID����
#define DRIVER_ADDRESS       0x01        // ��������ַ(Ĭ��Ϊ1)
#define DRIVER_TX_ID         0x06000000  // ���ͻ���ID (����->������)
#define DRIVER_RX_ID         0x05800000  // ���ջ���ID (������->����)
#define DRIVER_HEARTBEAT_ID  0x07000000  // ������ID (������->����)

// ���ͨ������
#define MOTOR_CHANNEL_A      0x01        // A·���(����)
#define MOTOR_CHANNEL_B      0x02        // B·���(����)

// �������Ͷ���
#define CMD_ENABLE           0x01        // ʹ�ܵ��
#define CMD_DISABLE          0x02        // ʧ�ܵ��
#define CMD_SPEED            0x03        // �����ٶ�

extern CAN_HandleTypeDef hcan;
extern uint8_t bk_flag_left;
extern uint8_t bk_flag_right;
extern UART_HandleTypeDef huart5;

/**
 * ����CAN����
 * @param id CAN��չID
 * @param data 8�ֽ�����
 */
static void keya_send_data(uint32_t id, uint8_t* data)
{
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.ExtId = id;  
  TxHeader.IDE = CAN_ID_EXT;             // ��չ֡(29λID)
  TxHeader.RTR = CAN_RTR_DATA;           // ����֡
  TxHeader.DLC = 8;                      // ֡����8�ֽ�
  TxHeader.TransmitGlobalTime = DISABLE;
  
  uint32_t TxMailbox;

  HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox);
  
  // ��ӡ���͵�CAN����(������)
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
 * �������
 * @param cmd_type ��������: CMD_ENABLE/CMD_DISABLE/CMD_SPEED
 * @param channel ���ͨ��: MOTOR_CHANNEL_A(��)/MOTOR_CHANNEL_B(��)
 * @param speed �ٶ�(-100��100����Ӧ-10000��10000)
 */
static void motor_control(uint8_t cmd_type, uint8_t channel, int8_t speed)
{
  uint8_t TxData[8] = {0};
  uint32_t tx_id = DRIVER_TX_ID + DRIVER_ADDRESS;
  
  if (cmd_type == CMD_ENABLE) {
    // ʹ�ܵ��: 23 0D 20 01/02 00 00 00 00
    TxData[0] = 0x23;
    TxData[1] = 0x0D;
    TxData[2] = 0x20;
    TxData[3] = channel;  // 01=A·(����), 02=B·(����)
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;
  } 
  else if (cmd_type == CMD_DISABLE) {
    // ʧ�ܵ��: 23 0C 20 01/02 00 00 00 00
    TxData[0] = 0x23;
    TxData[1] = 0x0C;
    TxData[2] = 0x20;
    TxData[3] = channel;  // 01=A·(����), 02=B·(����)
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;
  }
  else if (cmd_type == CMD_SPEED) {
    // �����ٶ�: 23 00 20 01/02 HH HH LL LL
    TxData[0] = 0x23;
    TxData[1] = 0x00;
    TxData[2] = 0x20;
    TxData[3] = channel;  // 01=A·(����), 02=B·(����)
    
    // ��-100��100���ٶ�ת��Ϊ-10000��10000
    int32_t sp_value = (int32_t)speed * 100;
    
    // 32λ�з���������ʾ�����ֽ��ں�
    TxData[4] = (sp_value >> 24) & 0xFF;  // ����ֽ�
    TxData[5] = (sp_value >> 16) & 0xFF;
    TxData[6] = (sp_value >> 8) & 0xFF;
    TxData[7] = sp_value & 0xFF;         // ����ֽ�
  }

  keya_send_data(tx_id, TxData);
}

/**
 * �����������ٶ�ʵ���˶�
 * @param speed_left �����ٶ�(-100��100)
 * @param speed_right �����ٶ�(-100��100)
 * @return 0=�ɹ���1=��������
 */
uint8_t intf_move_keyadouble(int8_t speed_left, int8_t speed_right)
{
  if ((abs(speed_left) > 100) || (abs(speed_right) > 100)) {
    printf("Wrong parameter in intf_move!!![lf=%d],[ri=%d]", speed_left, speed_right);
    return 1;
  }

  // ���ֱ�բ���� (A·)
  if (speed_left != 0) {
    bk_flag_left = 1;
    HAL_GPIO_WritePin(GPIOA, left_bk_Pin, GPIO_PIN_SET); // �ߵ�ƽ���ɱ�բ�����ת
  } else {
    bk_flag_left = 0; // 0Ϊ��բ��1Ϊ�ɿ�
  }
 
  // ���ֱ�բ���� (B·)
  if (speed_right != 0) {
    bk_flag_right = 1;
    HAL_GPIO_WritePin(GPIOA, right_bk_Pin, GPIO_PIN_SET);
  } else {
    bk_flag_right = 0; // 0Ϊ��բ��1Ϊ�ɿ�
  }

  // ��̬������¼�ϴ�����ʱ�䣬��֤���Ź�����ʱ
  static uint32_t last_cmd_time = 0;
  uint32_t current_time = HAL_GetTick();
  
  // ��������ϴ������900ms����Ҫ����ʹ�ܵ��
  if (current_time - last_cmd_time > 900) {
    motor_control(CMD_ENABLE, MOTOR_CHANNEL_A, 0);  // ʹ��A·(����)
    motor_control(CMD_ENABLE, MOTOR_CHANNEL_B, 0);  // ʹ��B·(����)
  }
  
  // ���·���ʱ��
  last_cmd_time = current_time;

  // ʹ��������·���
  motor_control(CMD_ENABLE, MOTOR_CHANNEL_A, 0);  // ʹ��A·(����)
  motor_control(CMD_ENABLE, MOTOR_CHANNEL_B, 0);  // ʹ��B·(����)
  // �����ٶ�����
  motor_control(CMD_SPEED, MOTOR_CHANNEL_A, speed_left);   // A·(����)�ٶ�
  motor_control(CMD_SPEED, MOTOR_CHANNEL_B, speed_right);  // B·(����)�ٶ�

  return 0; 
}
