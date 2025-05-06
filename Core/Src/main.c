/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"

#include "channel_parse.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LEN_SBUS 25
#define LEN_CHANEL 12
#define LEN_CMD 7
#define LEN_485 16

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// 串口临时接收缓冲区
uint8_t rxbuf1;
uint8_t rxbuf3; // 485
uint8_t rxbuf4; // sbus
uint8_t rxbuf5; // cmd_vel

// 485接收缓冲区
uint8_t g_485_rx_buf[LEN_485] = {0};
uint8_t g_485_pt = 0;

// sbus数据接收缓冲区
uint8_t g_sbus_rx_buf[LEN_SBUS] = {0};
uint8_t g_sbus_pt = 0; // 缓冲区操作下标，收完一帧数据，最高位置1

// cmd_vel接收缓冲区
uint8_t g_cmd_rx_buf[LEN_CMD] = {0};
uint8_t g_cmd_pt = 0; // 缓冲区操作下标，收完一帧数据，最高位置1

// 抱闸标记，0抱死，1松开
uint8_t bk_flag_left = 0;
uint8_t bk_flag_right = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)    
{
    HAL_UART_Transmit(&huart1, (unsigned char *)&ch, 1, 0xFFFF);   
    return ch;
}

/*
 * 解析sbus数据，填充到12个通道里
 */
static uint8_t parse_sbus_msg(uint16_t* channel)
{
  channel[0] = (g_sbus_rx_buf[1] >> 0 | g_sbus_rx_buf[2] << 8) & 0x07FF;
  channel[1] = (g_sbus_rx_buf[2] >> 3 | g_sbus_rx_buf[3] << 5) & 0x07FF;
  channel[2] = (g_sbus_rx_buf[3] >> 6 | g_sbus_rx_buf[4] << 2 | g_sbus_rx_buf[5] << 10) & 0x07FF;
  channel[3] = (g_sbus_rx_buf[5] >> 1 | g_sbus_rx_buf[6] << 7) & 0x07FF;
  channel[4] = (g_sbus_rx_buf[6] >> 4 | g_sbus_rx_buf[7] << 4) & 0x07FF;
  channel[5] = (g_sbus_rx_buf[7] >> 7 | g_sbus_rx_buf[8] << 1 | g_sbus_rx_buf[9] << 9) & 0x07FF;
  channel[6] = (g_sbus_rx_buf[9] >> 2 | g_sbus_rx_buf[10] << 6) & 0x07FF;
  channel[7] = (g_sbus_rx_buf[10] >> 5 | g_sbus_rx_buf[11] << 3) & 0x07FF;
  channel[8] = (g_sbus_rx_buf[12] >> 0 | g_sbus_rx_buf[13] << 8) & 0x07FF;
  channel[9] = (g_sbus_rx_buf[13] >> 3 | g_sbus_rx_buf[14] << 5) & 0x07FF;
  channel[10] = (g_sbus_rx_buf[14] >> 6 | g_sbus_rx_buf[15] << 2 | g_sbus_rx_buf[16] << 10) & 0x07FF;
  channel[11] = (g_sbus_rx_buf[16] >> 1 | g_sbus_rx_buf[17] << 7) & 0x07FF;

  // PPM协议的数据需要映射到SBUS范围（1050~1950映射到282~1722）
  for (int i = 0; i < LEN_CHANEL; i++) {
    channel[i] = (channel[i] - 282) * 5 / 8 + 1050;
  }
  
	return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2); // 使能定时器中断
  
  HAL_UART_Receive_IT(&huart1, &rxbuf1, 1); 	//enable uart	
	HAL_UART_Receive_IT(&huart3, &rxbuf3, 1);
  HAL_UART_Receive_IT(&huart4, &rxbuf4, 1);
  HAL_UART_Receive_IT(&huart5, &rxbuf5, 1);
  
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  htim4.Instance->CCR1 = 0; // ARR+1为100
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  htim4.Instance->CCR2 = 0; // ARR+1为100
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  htim4.Instance->CCR3 = 0; // ARR+1为100
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  htim4.Instance->CCR4 = 0; // ARR+1为100

  HAL_GPIO_WritePin(GPIOC, left_en_Pin|left_dir_Pin|right_dir_Pin|right_en_Pin, GPIO_PIN_SET);
  
  HAL_CAN_Start(&hcan); // 开启can通讯

	printf("Start while cycle...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t ch_val[LEN_CHANEL] = {0}; // 遥控器12个通道的值

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(100);

    
    if ((g_sbus_pt & 0x80) != 0) {
      // 发送g_cmd_pt的值到UART5
      char buffer[20];
      sprintf(buffer, "g_cmd_pt=%d\r\n", g_cmd_pt);
      HAL_UART_Transmit(&huart5, (uint8_t*)buffer, strlen(buffer), 100);
			// sbus收到数据，解析  
      parse_sbus_msg(ch_val);
      parse_chan_val(ch_val); 
      g_sbus_pt = 0; // 清0，继续下一帧数据的接收
		}
    
    if ((g_cmd_pt & 0x80) != 0) {
        // 打印调试信息，确认进入此分支
        char buffer[30];
        sprintf(buffer, "CMD received: %02X %02X\r\n", g_cmd_rx_buf[2], g_cmd_rx_buf[3]);
        HAL_UART_Transmit(&huart5, (uint8_t*)buffer, strlen(buffer), 100);
      HAL_GPIO_TogglePin(led_blue_GPIO_Port, led_blue_Pin);
      HAL_Delay(500); // 延时500ms 
      parse_cmd_vel(g_cmd_rx_buf[2], g_cmd_rx_buf[3]);
      g_cmd_pt = 0; // 清0，继续下一帧数据的接收	
		} 
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 63999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 31;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 100000;
  huart4.Init.WordLength = UART_WORDLENGTH_9B;
  huart4.Init.StopBits = UART_STOPBITS_2;
  huart4.Init.Parity = UART_PARITY_EVEN;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, left_en_Pin|left_dir_Pin|right_dir_Pin|right_en_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, left_bk_Pin|right_bk_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_blue_GPIO_Port, led_blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : left_en_Pin left_dir_Pin right_dir_Pin right_en_Pin */
  GPIO_InitStruct.Pin = left_en_Pin|left_dir_Pin|right_dir_Pin|right_en_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : left_bk_Pin right_bk_Pin */
  GPIO_InitStruct.Pin = left_bk_Pin|right_bk_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : led_blue_Pin */
  GPIO_InitStruct.Pin = led_blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_blue_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /*串口1*/
  if (huart->Instance == USART1) {
    HAL_UART_Transmit(&huart1, &rxbuf1, 1, 1000);
    HAL_UART_Receive_IT(&huart1, &rxbuf1, 1);
  }
  
  /*串口3*/
  if (huart->Instance == USART3) {
    // 缓冲区溢出，重新接收
    if (g_485_pt > (LEN_485 - 1)) {
      g_485_pt = 0;             
    }
    // 存入缓冲区
    g_485_rx_buf[g_485_pt] = rxbuf3;
    g_485_pt++;
    HAL_UART_Receive_IT(&huart3, &rxbuf3, 1);
  }
  
  /*串口4*/
  if (huart->Instance == UART4) {
    if ((g_sbus_pt & 0x80) == 0) { // 接收未完成
			if (g_sbus_pt > (LEN_SBUS - 1)) { // 缓冲区溢出，重新接收
				g_sbus_pt = 0;             
			}
			// 存入缓冲区
			g_sbus_rx_buf[g_sbus_pt] = rxbuf4;
			g_sbus_pt++;
			
		  // 判断数据头
			if (g_sbus_pt == 1) {
				if (rxbuf4 != 0x0f) {
					g_sbus_pt--; // 回滚，重新存
				}
			} else if (g_sbus_pt == 25) {
				// 判断数据尾
				if (rxbuf4 == 0x00) {
					g_sbus_pt |= 0x80; // 完成一帧数据的接收
				} else {
          g_sbus_pt = 0; // 数据错误，重新存
        }
			}
		}
    HAL_UART_Receive_IT(&huart4, &rxbuf4, 1);
  }
  
  /*串口5*/
  if (huart->Instance == UART5) {
    if ((g_cmd_pt & 0x80) == 0) { // 接收未完成
			if (g_cmd_pt > (LEN_CMD - 1)) { // 缓冲区溢出，重新接收
				g_cmd_pt = 0;             
			}
			// 存入缓冲区
			g_cmd_rx_buf[g_cmd_pt] = rxbuf5;
			g_cmd_pt++;
			
		  // 判断数据头
			if ((g_cmd_pt == 1) && (rxbuf5 != 0xff)) {
					g_cmd_pt--; // 回滚，重新存
			} else if ((g_cmd_pt == 2) && (rxbuf5 != 0x2)) {
					g_cmd_pt--; // 回滚，重新存
			} else if (g_cmd_pt == 5) {
				// 判断数据尾
				if (rxbuf5 == 0x00) {
					g_cmd_pt |= 0x80; // 完成一帧数据的接收
				} else {
          g_cmd_pt = 0; // 数据错误，重新存
        }
			}
		}
    HAL_UART_Receive_IT(&huart5, &rxbuf5, 1);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2) {
    // 200ms检查一次
    // 左侧
    static uint8_t count_left = 0;
    count_left++;
    if (bk_flag_left != 0) {
      count_left = 0;
    }
    if (count_left >= 25) { // 5s后抱闸
      count_left = 0;
		  HAL_GPIO_WritePin(GPIOA, left_bk_Pin, GPIO_PIN_RESET);
    }
    
    // 右侧
    static uint8_t count_right = 0;
    count_right++;
    if (bk_flag_right != 0) {
      count_right = 0;
    }
    if (count_right >= 25) { // 5s后抱闸
      count_right = 0;
		  HAL_GPIO_WritePin(GPIOA, right_bk_Pin, GPIO_PIN_RESET);
    }
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
