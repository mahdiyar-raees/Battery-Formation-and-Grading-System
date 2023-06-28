/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TYPEDEF.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern 					MCU_FLAG_t 	flag;
extern 					uint8_t 		main_board_address;
extern volatile uint16_t   	first_send;
extern					uint8_t    	charger_board_num;
extern					uint8_t 	 	board_channel_get_data;
extern          uint8_t 		usart_recive_rx[2];
extern					uint8_t start_buffer[PARITY_SIZE];
extern					uint8_t end_buffer  [PARITY_SIZE];
extern					void send_handshake(uint8_t command);	

extern					uint8_t rx_normal_board[64] ;
extern					uint8_t finish_reacive;			
extern 					uint8_t	rx_LED[64];
extern          uint8_t send_array[96][8];																		
extern					uint8_t in_counter;
extern 					uint8_t out_counter;																			

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

extern void CHECK_CHARGER_BOARD(void);
extern void iner_resistance_handler(void);

extern void CAN_SEND_DATA(uint8_t *transmit_buffer , uint16_t identifier ,uint8_t lenth);
extern void get_normal_board(void);
extern void SEND_command_handler(void);	
extern void START_handler(void);
extern void STOP_handler(void);
extern void RESET_handler(void);
extern void GET_DATA_handler(void);
extern void LED_CHANNEL_handler(void);
extern void iner_resistance_handler(void);
extern void CHECK_CHARGER_BOARD(void);
extern void CHECK_MAIN_BOARD(void);
extern void READ_SEND(void);
extern void READ_STATUS(void);
extern void formation_end(void);
extern void	formation_start(void);
extern void lcd_1_first_menu(void);
extern void lcd_2_first_menu(void);
extern void lcd_1_not_ready(void);
extern void lcd_2_not_ready(void);

extern uint8_t SET_board_num(uint8_t last_board_address);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(pb1_Pin);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEYPAD2_D_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEYPAD2_EXT_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
		uint8_t x;
	
	
		x = USART1->DR;
TIM2->CNT =0;
											if(flag.flag_BIT.PC_BUS_BUSY==1)
												{
													uint8_t data[2]= {0xFE,usart_recive_rx[1]};
													HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
													HAL_UART_Transmit(&huart1,data,2, 100);
													HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);	
													return;
												
												}
	if(flag.flag_BIT.command_or_data ==0)
		{
			
			static uint8_t counter =	0;
			if(!finish_reacive)
						{
						//command
						switch (counter)
						{
								case 0:

									usart_recive_rx[0] = USART1->DR;
								
									if ( usart_recive_rx[0] == main_board_address)
										{
											counter ++;
											HAL_TIM_Base_Start_IT(&htim2);
										}
									else
										{
											counter = 0;
										}
									
									break;
								case 1:
										HAL_TIM_Base_Stop(&htim2);
										usart_recive_rx[1] = USART1->DR; 
//											if(flag.flag_BIT.PC_BUS_BUSY==1)
//												{
//													uint8_t data[2]= {0xFE,usart_recive_rx[1]};
//													HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
//													HAL_UART_Transmit(&huart1,data,2, 100);
//													HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);	
//													return;
//												
//												}
										
										TIM5->CNT =0;
										flag.flag_BIT.ASK_DATA =0;
										TIM2->CNT =0;
										first_send=0;
										counter=0;
										finish_reacive =1;
										flag.flag_BIT.get_channel_1=0;
										flag.flag_BIT.state_process_1=0;
										flag.flag_BIT.process_1=0;
										flag.flag_BIT.get_channel_2=0;
										flag.flag_BIT.state_process_2=0;
										flag.flag_BIT.process_2=0;					
										
										board_channel_get_data =1 ;
										flag.flag_BIT.check_charger_board =0;
										flag.flag_BIT.LCD2_BUS_BUSY=1;
										flag.flag_BIT.LCD1_BUS_BUSY=1;
										flag.flag_BIT.uart_recive = 1;
								
										out_counter =0;
										in_counter =0;
										
										charger_board_num=0;
										if(usart_recive_rx[1]==11)
											{

																CHECK_CHARGER_BOARD();
														
												}
										else{
														switch(usart_recive_rx[1])
																{
																	//send
																	case 1:

																			
																				send_handshake(usart_recive_rx[1]);
																				if(rx_normal_board[0]==0x11)
																					{
																					charger_board_num=0;
																					}
																					else
																					{
																						uint8_t str[2]= {NOT_FOUND_CHARGER , 0};		
																						str[0] = 	usart_recive_rx[1];
																						str[1] =  0;
																						HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
																						HAL_UART_Transmit(&huart1,str,2,100);
																						HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);	
																							
																						str[0] = 	NOT_FOUND_CHARGER;
																						str[1]=		0 ;
																						HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
																						HAL_UART_Transmit(&huart1,str,2,100);
																						HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
																							charger_board_num =  SET_board_num(charger_board_num);
																						}
																					if(charger_board_num<64){
																					SEND_command_handler();	}
																						else
																						{
																							counter = 0;	
																							
																							HAL_TIM_Base_Stop(&htim5);
																							HAL_TIM_Base_Stop(&htim2);
																							flag.flag_BIT.command_or_data = 0 ;
																							first_send=0;
																							flag.flag_BIT.uart_recive=0;
																							flag.flag_BIT.LCD2_BUS_BUSY=0;
																							flag.flag_BIT.LCD1_BUS_BUSY=0;
																							
																							usart_recive_rx[0] = 0x00;
																							usart_recive_rx[1] = 0x00;	
																							flag.flag_BIT.command_or_data=0;
																							flag.flag_BIT.ASK_DATA=0;
																							finish_reacive =0;
																										if(flag.flag_BIT.key_enter)
																												{
																													lcd_1_first_menu();
																													lcd_2_first_menu();
																													flag.flag_BIT.key_enter=0;
																												}
																						}
																					
																				break;
																	//start
																	case 2:
																		START_handler();
																	break;
																	//stop
																	case 3:
																		STOP_handler();
																	break;
																	//reset
																	case 4:
																		RESET_handler();
																	break;
																	//get data
																	case 5:
																	send_handshake(usart_recive_rx[1]);
	
																				
																					return;
																		
																	break;
																	case 6:

																		break;
																	case 7:
																		send_handshake(usart_recive_rx[1]);
																		flag.flag_BIT.ASK_DATA=1;
																		finish_reacive =0;
																		in_counter=0;
																		TIM2 -> CNT =0 ;
																		HAL_TIM_Base_Start_IT(&htim2);
																		if(rx_normal_board[0]==0x11)
																			{
																			charger_board_num=0;
																			}
																		
																	break;
																	case 8:
										//								flag.flag_BIT.LCD2_BUS_BUSY=0;
										//								flag.flag_BIT.LCD1_BUS_BUSY=0;
										//								huart1.RxState= HAL_UART_STATE_READY;
										//								HAL_UART_Receive_IT(&huart1,usart_recive_rx1,2);
																		break;
																	case 9:
										//								flag.flag_BIT.LCD2_BUS_BUSY=0;
										//								flag.flag_BIT.LCD1_BUS_BUSY=0;
										//								huart1.RxState= HAL_UART_STATE_READY;
										//								HAL_UART_Receive_IT(&huart1,usart_recive_rx1,2);
																		break;
																	case 10: 
																		iner_resistance_handler();
																	break;

																	case 12:				
																		CHECK_MAIN_BOARD();
																	break;
																	case 13:

																	send_handshake(usart_recive_rx[1]);
																	if(rx_normal_board[0]==0x11)
																			{
																			charger_board_num=0;
																			}
																			else
																			{
																				uint8_t str[2]= {NOT_FOUND_CHARGER , 0};		
																				str[0] = 	usart_recive_rx[1];
																				str[1] =  0;
																				HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
																				HAL_UART_Transmit(&huart1,str,2,100);
																				HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);	

																				str[0] = 	NOT_FOUND_CHARGER;
																				str[1]=		0 ;
																				HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
																				HAL_UART_Transmit(&huart1,str,2,100);
																				HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
																				charger_board_num =  SET_board_num(charger_board_num);
																			}
																	if(charger_board_num<64){
																						READ_SEND();}
																	else
																			{
																				counter = 0;	
																				
																				HAL_TIM_Base_Stop(&htim5);
																				HAL_TIM_Base_Stop(&htim2);
																				flag.flag_BIT.command_or_data = 0 ;
																				first_send=0;
																				flag.flag_BIT.uart_recive=0;
																				flag.flag_BIT.LCD2_BUS_BUSY=0;
																				flag.flag_BIT.LCD1_BUS_BUSY=0;
																			
																				
																				usart_recive_rx[0] = 0x00;
																				usart_recive_rx[1] = 0x00;	
																				flag.flag_BIT.command_or_data=0;
																				flag.flag_BIT.ASK_DATA=0;
																				finish_reacive =0;
																							if(flag.flag_BIT.key_enter)
																							{
																								lcd_1_first_menu();
																								lcd_2_first_menu();
																								flag.flag_BIT.key_enter=0;
																							}
																			}
																							
																	break;
																	case 14:
																		READ_STATUS();
																	break;
																	case 15:
																		 send_handshake(usart_recive_rx[1]);
																	break;
																	case 18:
																		formation_start();
																	break;
																	case 19: 
																	formation_end();
																	break;
																	
																		
																	default:
																	usart_recive_rx[0] = 0x00;
																	usart_recive_rx[1] = 0x00;		
																	flag.flag_BIT.uart_recive=0;
																	flag.flag_BIT.LCD2_BUS_BUSY=0;
																	flag.flag_BIT.LCD1_BUS_BUSY=0;
																	flag.flag_BIT.command_or_data=0;
																	flag.flag_BIT.ASK_DATA=0;
																	finish_reacive =0;
																	break;
																	
																}	
												
												
											break;

																
										default:
											counter = 0;
											
											break;
								

							}
						}
					}
		}
		//data
		//***********************//
		else
		{
			
		if(flag.flag_BIT.ASK_DATA)
			{
				
				
					switch(usart_recive_rx[1])
						{
							//send
							case 1:
								if(!finish_reacive)
									{
										send_array[out_counter][in_counter] = USART1->DR;
										in_counter++;
										TIM2->CNT =0;
									}
									if(in_counter ==8)
										{
											out_counter++;
											in_counter =0;
											if(out_counter ==96)
												{
													// we should add variable to nit get command while command activated
													finish_reacive=1;
													flag.flag_BIT.ASK_DATA=0;
											
													HAL_TIM_Base_Stop(&htim2);
													CAN_SEND_DATA(send_array[0],charger_board_num+64, 8);
												}

										}
							break;

							case 7:
								if(!finish_reacive)
									{
										rx_LED[in_counter] = USART1->DR;
										in_counter++;
										
									}
								if(in_counter ==64)
									{
										finish_reacive=1;
										flag.flag_BIT.ASK_DATA=0;
										in_counter =0;
										HAL_TIM_Base_Stop(&htim2);
										TIM2->CNT =0;
										LED_CHANNEL_handler();
										
																			}
									
									
									
							break;
							case 10: 
								iner_resistance_handler();
							break;


							case 15:
								if(!finish_reacive)
									{
										rx_normal_board[in_counter] = USART1->DR;
										in_counter++;
										
									}
								if(in_counter ==64)
									{
										finish_reacive=1;
										flag.flag_BIT.ASK_DATA=0;
										in_counter =0;
										HAL_TIM_Base_Stop(&htim2);
										TIM2->CNT =0;
										TIM5->CNT =0;
										first_send =1 ;
										get_normal_board();
									}


							break;
							default:
									
								flag.flag_BIT.uart_recive=0;
								flag.flag_BIT.command_or_data=0;
								
							break;
						}	
					}
		}
	
		//return;
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

