/*
 * ˫���Ĵ������������򣬻���pwm��ʽ��
 */

#include <stdio.h>
#include <stdlib.h>
#include "drv_shb.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

static uint8_t motor_left_forward(uint8_t val)
{
  HAL_GPIO_WritePin(GPIOC, left_dir_Pin, GPIO_PIN_RESET);
  htim4.Instance->CCR4 = val; // ARR+1Ϊ100
  return 0;   
}

static uint8_t motor_left_back(uint8_t val)
{
  HAL_GPIO_WritePin(GPIOC, left_dir_Pin, GPIO_PIN_SET);
  htim4.Instance->CCR4 = val; // ARR+1Ϊ100
  return 0;   
}

static uint8_t motor_right_forward(uint8_t val)
{
  HAL_GPIO_WritePin(GPIOC, right_dir_Pin, GPIO_PIN_RESET);
  htim4.Instance->CCR1 = val; // ARR+1Ϊ100
  return 0;   
}

static uint8_t motor_right_back(uint8_t val)
{
  HAL_GPIO_WritePin(GPIOC, right_dir_Pin, GPIO_PIN_SET);
  htim4.Instance->CCR1 = val; // ARR+1Ϊ100
  return 0;   
}

static uint8_t motor_left_stop(void)
{
  htim4.Instance->CCR4 = 0;
  HAL_GPIO_WritePin(GPIOC, left_en_Pin, GPIO_PIN_SET);
  return 0;
}

static uint8_t motor_right_stop(void)
{
  htim4.Instance->CCR1 = 0;
  HAL_GPIO_WritePin(GPIOC, right_en_Pin, GPIO_PIN_SET);
  return 0;
}


/*
 * ���������ٶ�ʵ���˶�
 * ȡֵ-100��100����ֵǰ������ֵ����
 */
uint8_t intf_move_shb(int8_t speed_left, int8_t speed_right)
{
  if ((abs(speed_left) > 100) || (abs(speed_right) > 100)) {
    printf("Wrong parameter in intf_move!!![lf=%d],[ri=%d]", speed_left, speed_right);
    return 1;
  }

  // ��ֵȥ������
  uint8_t sl = abs(speed_left);
  uint8_t sr = abs(speed_right);

  if (speed_left > 0) {
    motor_left_forward(sl);
  } else if (speed_left < 0) {
    motor_left_back(sl);
  } else {
    motor_left_stop();
  }

  if (speed_right > 0) {
    motor_right_forward(sr);
  } else if (speed_right < 0) {
    motor_right_back(sr);
  } else {
    motor_right_stop();
  }

  return 0; 
}
