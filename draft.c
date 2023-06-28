/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/// define type for reading keypad & board address
typedef union
{
		uint8_t data;
			struct 
			{
				uint8_t SW0:1;
				uint8_t SW1:1;
				uint8_t SW2:1;
				uint8_t SW3:1;
				uint8_t SW4:1;
				uint8_t SW5:1;
				uint8_t SW6:1;
				uint8_t SW7:1;
			}data_BIT;

		}KEYPAD_SCAN_t;

/// define type for MCU flag		
typedef union
{
		uint8_t flag_byte;
			struct 
			{
				uint8_t key1_flag:1;
				uint8_t key2_flag:1;
				uint8_t SW2:1;
				uint8_t SW3:1;
				uint8_t SW4:1;
				uint8_t SW5:1;
				uint8_t SW6:1;
				uint8_t SW7:1;
			}flag_BIT;

		}MCU_FLAG_t;


		
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
			#define STR(x)  #x;
			#define debug_print(str)     \
				do									     \
				{									       \
					HAL_UART_Transmit(&huart3,STR(str),strlen(str),1000);\
				}while(0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
	
	//define semi function for GPIO OUTPUT
		#define LED_ON 		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET)
		#define LED_OFF		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET)

		#define LCD1_light_ON 	HAL_GPIO_WritePin(LCD1_back_light_GPIO_Port,LCD1_back_light_Pin,GPIO_PIN_RESET)
		#define LCD1_light_OFF	HAL_GPIO_WritePin(LCD1_back_light_GPIO_Port,LCD1_back_light_Pin,GPIO_PIN_SET)		

		#define LCD2_light_ON 	HAL_GPIO_WritePin(LCD2_back_light_GPIO_Port,LCD2_back_light_Pin,GPIO_PIN_RESET)
		#define LCD2_light_OFF	HAL_GPIO_WritePin(LCD2_back_light_GPIO_Port,LCD2_back_light_Pin,GPIO_PIN_SET)			
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

			//global variable for key pad one
			uint8_t keypad1_num =0 ;
			//global variable for key pad two
			uint8_t keypad2_num =0 ;

			//global variable for board address
			uint8_t board_address =0 ;

//MCU flag in bit wise situation
MCU_FLAG_t flag;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

//// function for scan board address
uint8_t Read_board_address( void )
	{
		KEYPAD_SCAN_t address ;
		
		address.data_BIT.SW0 = HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin);
		address.data_BIT.SW1 = HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin);
		address.data_BIT.SW2 = HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin);
		address.data_BIT.SW3 = HAL_GPIO_ReadPin(SW4_GPIO_Port,SW4_Pin);
		address.data_BIT.SW4 = HAL_GPIO_ReadPin(SW5_GPIO_Port,SW5_Pin);
		address.data_BIT.SW5 = HAL_GPIO_ReadPin(SW6_GPIO_Port,SW6_Pin);
		address.data_BIT.SW6 = HAL_GPIO_ReadPin(SW7_GPIO_Port,SW7_Pin);
		address.data_BIT.SW7 = HAL_GPIO_ReadPin(SW8_GPIO_Port,SW8_Pin);
	
		return address.data ; 
	}

// function for scan KEYPAD 1 
	uint8_t Read_key_scan1( void  )
	{
		KEYPAD_SCAN_t key ;
		
		key.data_BIT.SW0 = HAL_GPIO_ReadPin(KEYPAD1_A_GPIO_Port,KEYPAD1_A_Pin);
		key.data_BIT.SW1 = HAL_GPIO_ReadPin(KEYPAD1_B_GPIO_Port,KEYPAD1_B_Pin);
		key.data_BIT.SW2 = HAL_GPIO_ReadPin(KEYPAD1_C_GPIO_Port,KEYPAD1_C_Pin);
		key.data_BIT.SW3 = HAL_GPIO_ReadPin(KEYPAD1_D_GPIO_Port,KEYPAD1_D_Pin);
		key.data_BIT.SW4 = 0;
		key.data_BIT.SW5 = 0;
		key.data_BIT.SW6 = 0;
		key.data_BIT.SW7 = 0;
	
		return key.data ; 
	}	
	
// function for scan KEYPAD 2
	
uint8_t Read_key_scan2( void  )
	{
		KEYPAD_SCAN_t key ;
		
		key.data_BIT.SW0 = HAL_GPIO_ReadPin(KEYPAD2_A_GPIO_Port,KEYPAD2_A_Pin);
		key.data_BIT.SW1 = HAL_GPIO_ReadPin(KEYPAD2_B_GPIO_Port,KEYPAD2_B_Pin);
		key.data_BIT.SW2 = HAL_GPIO_ReadPin(KEYPAD2_C_GPIO_Port,KEYPAD2_C_Pin);
		key.data_BIT.SW3 = HAL_GPIO_ReadPin(KEYPAD2_D_GPIO_Port,KEYPAD2_D_Pin);
		key.data_BIT.SW4 = 0;
		key.data_BIT.SW5 = 0;
		key.data_BIT.SW6 = 0;
		key.data_BIT.SW7 = 0;
	
		return key.data ; 
	}	
	
// external interupt handeler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		 
		if(GPIO_Pin == GPIO_PIN_0)
		{
			LCD1_light_ON;
			TIM6->CNT =0;
			flag.flag_BIT.key1_flag=1;
		}
		else if(GPIO_Pin == GPIO_PIN_4)
		{
			LCD2_light_ON;
			TIM7->CNT =0;
			flag.flag_BIT.key2_flag=1;
		}

}

// timer interupt handeler

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

			if(htim ->Instance == TIM6){
			
						LCD1_light_OFF;
						
			}
	
			if(htim ->Instance == TIM6){
			
						LCD2_light_OFF;
						
			}

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
		// timer start 
			HAL_TIM_Base_Start_IT(&htim6);
			HAL_TIM_Base_Start_IT(&htim7);
		/**********************************/

		flag.flag_byte =0;
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// LCD2 handler
		if(flag.flag_BIT.key2_flag)
		{
				flag.flag_BIT.key2_flag =0;
				keypad2_num = Read_key_scan2();
					
		}
		
		// LCD1 handler		
		if(flag.flag_BIT.key1_flag)
		{
			flag.flag_BIT.key1_flag=0;
			keypad1_num = Read_key_scan1();
		
		
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 63999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 33750;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 63999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 33750;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(KEYPAD1_A_GPIO_Port, KEYPAD1_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD1_RS_Pin|LCD1_RW_Pin|LCD2_EN_Pin|LCD2_D4_Pin
                          |LCD2_D5_Pin|LCD2_D6_Pin|LCD2_D7_Pin|LCD2_back_light_Pin
                          |lCD1_EN_Pin|LCD1_D4_Pin|LCD1_D5_Pin|LCD1_D6_Pin
                          |LCD1_D7_Pin|LCD2_RS_Pin|LCD2_RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD1_back_light_GPIO_Port, LCD1_back_light_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB_1_Pin SW1_Pin SW2_Pin SW3_Pin
                           SW4_Pin SW5_Pin SW6_Pin SW7_Pin
                           SW8_Pin */
  GPIO_InitStruct.Pin = PB_1_Pin|SW1_Pin|SW2_Pin|SW3_Pin
                          |SW4_Pin|SW5_Pin|SW6_Pin|SW7_Pin
                          |SW8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD2_A_Pin KEYPAD2_B_Pin KEYPAD2_C_Pin KEYPAD2_D_Pin
                           KEYPAD1_B_Pin KEYPAD1_C_Pin KEYPAD1_D_Pin */
  GPIO_InitStruct.Pin = KEYPAD2_A_Pin|KEYPAD2_B_Pin|KEYPAD2_C_Pin|KEYPAD2_D_Pin
                          |KEYPAD1_B_Pin|KEYPAD1_C_Pin|KEYPAD1_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD2_EXT_Pin */
  GPIO_InitStruct.Pin = KEYPAD2_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEYPAD2_EXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD1_A_Pin */
  GPIO_InitStruct.Pin = KEYPAD1_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KEYPAD1_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD1_EXT_Pin */
  GPIO_InitStruct.Pin = KEYPAD1_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEYPAD1_EXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD1_RS_Pin LCD1_RW_Pin LCD2_EN_Pin LCD2_D4_Pin
                           LCD2_D5_Pin LCD2_D6_Pin LCD2_D7_Pin LCD2_back_light_Pin
                           lCD1_EN_Pin LCD1_D4_Pin LCD1_D5_Pin LCD1_D6_Pin
                           LCD1_D7_Pin LCD2_RS_Pin LCD2_RW_Pin */
  GPIO_InitStruct.Pin = LCD1_RS_Pin|LCD1_RW_Pin|LCD2_EN_Pin|LCD2_D4_Pin
                          |LCD2_D5_Pin|LCD2_D6_Pin|LCD2_D7_Pin|LCD2_back_light_Pin
                          |lCD1_EN_Pin|LCD1_D4_Pin|LCD1_D5_Pin|LCD1_D6_Pin
                          |LCD1_D7_Pin|LCD2_RS_Pin|LCD2_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD1_back_light_Pin */
  GPIO_InitStruct.Pin = LCD1_back_light_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD1_back_light_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
