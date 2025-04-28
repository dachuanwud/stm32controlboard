/*
 * 小履带车的驱动程序。
 */

#include <stdio.h>
#include <stdlib.h>
#include "drv_toy.h"

extern TIM_HandleTypeDef htim4;

static uint8_t motor_left_forward(uint8_t val)
{
  htim4.Instance->CCR3 = 0; // ARR+1为100
  HAL_Delay(1);
  htim4.Instance->CCR4 = val; // ARR+1为100
  return 0;   
}

static uint8_t motor_left_back(uint8_t val)
{
  htim4.Instance->CCR4 = 0; // ARR+1为100
  HAL_Delay(1);
  htim4.Instance->CCR3 = val; // ARR+1为100
  return 0;   
}

static uint8_t motor_right_forward(uint8_t val)
{
  htim4.Instance->CCR2 = 0; // ARR+1为100
  HAL_Delay(1);
  htim4.Instance->CCR1 = val; // ARR+1为100
  return 0;   
}

static uint8_t motor_right_back(uint8_t val)
{
  htim4.Instance->CCR1 = 0; // ARR+1为100
  HAL_Delay(1);
  htim4.Instance->CCR2 = val; // ARR+1为100   
  return 0;   
}

static uint8_t motor_left_stop(void)
{
  htim4.Instance->CCR3 = 0;
  htim4.Instance->CCR4 = 0;
  return 0;
}

static uint8_t motor_right_stop(void)
{
  htim4.Instance->CCR1 = 0;
  htim4.Instance->CCR2 = 0;
  return 0;
}


/*
 * 配置左右速度实现运动
 * 取值-100到100，正值前进，负值后退
 */
uint8_t intf_move_toy(int8_t speed_left, int8_t speed_right)
{
  if ((abs(speed_left) > 100) || (abs(speed_right) > 100)) {
    printf("Wrong parameter in intf_move!!![lf=%d],[ri=%d]", speed_left, speed_right);
    return 1;
  }
 
  // 将值去掉符号
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
