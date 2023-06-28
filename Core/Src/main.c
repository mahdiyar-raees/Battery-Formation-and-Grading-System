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
#include "LCD1.h"
#include "LCD2.h"
#include "TYPEDEF.h"
#include "DEBUG.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t start_buffer[PARITY_SIZE] = {first_start,seccond_start , third_start};
uint8_t end_buffer  [PARITY_SIZE] = {first_end,seccond_end , third_end};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

			

			//global variable for board address
			uint8_t main_board_address;

			//MCU flag in bit wise situation
			MCU_FLAG_t flag;

			// LCD
			LCD_t lcd[2];
		  uint8_t     first_send_1=0			;
		  uint8_t     first_send_2=0			;			
			uint8_t   	step_count_1 =0	 		;
		  uint8_t   	step_count_2 =0			; 	
		
			// can recive massage header
			CAN_RxHeaderTypeDef rx_CAN_Init;

			
			//PC comunication variable
			volatile uint16_t    first_send;

			uint8_t    charger_board_num=0;
			uint8_t 	 board_channel_get_data=1;

			// recive buffer for pc command 
			uint8_t usart_recive_rx[2];
		
			 
			uint8_t	rx_LED[64];
			uint8_t rx_normal_board[64] = {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11, \
																		 0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11, \
																		 0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11, \
																		 0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11, \
																		 0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11, \
																		 0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11, \
																		 0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11, \
																		 0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11, \
																			};


timer_flag_t timer_flag;
uint8_t finish_reacive;
uint8_t send_counter=0;	
uint8_t in_counter;
uint8_t out_counter;	
																			
																			
																			
////////////////////////////////////////////////////////////////////////////////////////////////////////

	uint8_t get_data_counter=0;																		
  
//test 

uint8_t rx_buffer[8] = {0,0,0,0,0,0,0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */


uint8_t SET_board_num(uint8_t last_board_address);


uint8_t send_array[96][8];
//********necessary function******
uint8_t Read_board_address(void);

//**********CAN FUNCTION***********
void CAN_SEND_DATA(uint8_t *transmit_buffer , uint16_t identifier ,uint8_t lenth);
void CAN_Filtter_config(void);


//*********LCD 1 FUNCTION********
uint8_t Read_key_scan1(void);
void LCD_1_menu(void);

void LCD_1_READ_NUMBER(uint8_t lcd_key,uint8_t position);
void LCD_1_get_data(LCD_t lcd);
void LCD_1_MASTER(void);
void process_handler_handler_1(uint8_t board_number);
void process_state_handler_1(uint8_t board_number);
//*********LCD 2 FUNCTION******
uint8_t Read_key_scan2(void);
void LCD_2_get_data(LCD_t lcd);
void LCD_2_menu(void);
void LCD_2_READ_NUMBER(uint8_t lcd_key,uint8_t position);
void LCD_2_MASTER(void);
void process_handler_handler_2(uint8_t board_number);
void process_state_handler_2(uint8_t board_number);
//*************CUMUNOCATION*********
void RESET_handler(void);
void SEND_command_handler(void);
void START_handler(void);
void STOP_handler(void);
void formation_end(void);
void	formation_start(void);
void GET_DATA_handler(void );
void LED_CHANNEL_handler(void);
void iner_resistance_handler(void);
void CHECK_CHARGER_BOARD(void);
void CHECK_MAIN_BOARD(void);
void READ_SEND(void);
void READ_STATUS(void);
void get_normal_board(void);
void send_handshake(uint8_t command);
void lcd_1_first_menu(void);
void lcd_2_first_menu(void);
void lcd_1_not_ready(void);
void lcd_2_not_ready(void);
void delay_ms(uint16_t delay_ms);
//*************INTERUPT**********
	// receving interupt for FIFO 0
	// only for PC comunication
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
	{
	 flag.flag_BIT.CAN_FIFO0 =1;
			
		switch(usart_recive_rx[1])
				{
					//send
					case 1:
						if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
									{
										Error_Handler();
									}
							
							TIM5->CNT =0;
							
							//HAL_TIM_Base_Stop_IT(&htim5);
							SEND_command_handler();							
							
					break;

					
					//get data
					case 5:
								if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
									{
										Error_Handler();
									}
						TIM5->CNT =0;
						HAL_TIM_Base_Stop_IT(&htim5);
									__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
						GET_DATA_handler();
					break;
					case 6:
					{
						get_data   buffer;
						if (HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_CAN_Init,buffer.rx_buffer)!= HAL_OK)
							{
								Error_Handler();
							}	
							flag.flag_BIT.PC_BUS_BUSY=0;
							if(flag.flag_BIT.get_channel_1)
							{
									HAL_TIM_Base_Stop_IT(&htim4);
								__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
									flag.flag_BIT.get_channel_1=0;
									
									uint8_t str[20];
									lcd_1_clear();
									lcd_1_gotoxy(0,0);
									sprintf((char *)str,"Voltage %d",buffer.param.voltage);
									lcd_1_puts((char *)str);
									
									lcd_1_gotoxy(0,1);
									sprintf((char *)str,"Current %d",buffer.param.current);
									lcd_1_puts((char *)str);
							

									lcd_1_gotoxy(0,2);
									sprintf((char *)str,"Capacity %d",buffer.param.soc);
									lcd_1_puts((char *)str);
									

									first_send_1=0;
									flag.flag_BIT.LCD1_BUS_BUSY=0;
									lcd_2_first_menu();
									
								}
							else if(flag.flag_BIT.get_channel_2)
								{
									HAL_TIM_Base_Stop_IT(&htim3);
									__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);
									flag.flag_BIT.get_channel_2=0;
									
									uint8_t str[20];
									lcd_2_clear();
									lcd_2_gotoxy(0,0);
									sprintf((char *)str,"Voltage %d",buffer.param.voltage);
									lcd_2_puts((char *)str);
									
									lcd_2_gotoxy(0,1);
									sprintf((char *)str,"Current %d",buffer.param.current);
									lcd_2_puts((char *)str);
							

									lcd_2_gotoxy(0,2);
									sprintf((char *)str,"Capacity %d",buffer.param.soc);
									lcd_2_puts((char *)str);
									

									first_send_2=0;
									flag.flag_BIT.LCD2_BUS_BUSY=0;
									lcd_1_first_menu();
								}
							}
					break;
					case 7:
						LED_CHANNEL_handler();
					break;
					case 8:
					if (HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
						{
							Error_Handler();
						}	
						if(flag.flag_BIT.state_process_1)		
								{
									process_state_handler_1(0);
								}
						else if(flag.flag_BIT.state_process_2)		
									{
									process_state_handler_2(0);
									}
					case 9:
					if (HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
						{
							Error_Handler();
						}	
						if(flag.flag_BIT.process_1)		
							{
								process_handler_handler_1(0);
							}
						else if(flag.flag_BIT.process_2)		
							{
								process_handler_handler_2(0);
							}					
						break;
					case 10: 
						iner_resistance_handler();
					break;
					case 11:
								if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
									{
										Error_Handler();
									}
							TIM5->CNT =0;
							HAL_TIM_Base_Stop_IT(&htim5);
									__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
							CHECK_CHARGER_BOARD();
					break;

					
					case 13:
						if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
									{
										Error_Handler();
									}
							TIM5 ->CNT =0;
							first_send= 1;
							READ_SEND();
						
					break;
					case 14:
							TIM5 ->CNT =0;
							READ_STATUS();
						break;
					default:
							if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
									{
										Error_Handler();
									}
					//we should add error statement here 	
					break;
				}	
			


	}





// external interupt handeler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

		if(GPIO_Pin == GPIO_PIN_3)
		{
			
			TIM6->CNT =0;
			lcd[1].key = Read_key_scan1();
		if(flag.flag_BIT.grading_mode)
				{
					lcd_1_clear();
					lcd_1_gotoxy(3,2);
					lcd_1_puts("Grading Mode");
					lcd_2_clear();
					lcd_2_gotoxy(3,2);
					lcd_2_puts("Grading Mode");	
				}
		else if(flag.flag_BIT.LCD2_BUS_BUSY==1)
			{
					lcd[1].lcd_menue=1;
					lcd_1_init();
					lcd_1_clear();					
					lcd_1_gotoxy(4,1);
					lcd_1_puts("Please Wait ");
					flag.flag_BIT.key_enter=1;
					//TIM6->CNT=0;
					//HAL_TIM_Base_Start_IT(&htim6);
			}
		else if(flag.flag_BIT.process_1)
			{
				
					if(lcd[1].key==14)
						{
							flag.flag_BIT.LCD1_BUS_BUSY=0;
							lcd_1_first_menu();
							
							flag.flag_BIT.lcd1_function=0;
							flag.flag_BIT.CELL1_RESTART=0;
							
							flag.flag_BIT.lcd1_function=0;
							flag.flag_BIT.state_process_1=0;
							
							lcd[1].number_position=0;
							flag.flag_BIT.process_1=0;
						}
						// UP key
						else if(lcd[1].key==15)
						{
							step_count_1++;
							process_handler_handler_1(0);
						}
						// down key
						else if(lcd[1].key==16)
						{
							if(step_count_1 >0){step_count_1--;}else {step_count_1 =0;}										
							process_handler_handler_1(0);
						}						
			}
							else if(lcd[1].key==13 && lcd[1].lcd_menue ==1 && flag.flag_BIT.lcd1_function==0)
								{
									flag.flag_BIT.lcd1_function=1;
									TIM7->CNT =0;
									
									flag.flag_BIT.LCD1_BUS_BUSY=1;
									lcd_2_not_ready();
									TIM6->CNT=0;
									HAL_TIM_Base_Start_IT(&htim6);
																		lcd_1_init();
																		lcd_1_clear();
																		lcd_1_gotoxy(0,0);
																		lcd_1_puts("1_Start ");
																		lcd_1_gotoxy(0,1);
																		lcd_1_puts("2_Stop");
																		lcd_1_gotoxy(0,2);
																		lcd_1_puts("3_Restart");
																		lcd_1_gotoxy(0,3);
																		lcd_1_puts("4_Restart Cell");
																		flag.flag_BIT.get_channel_1=0;
																		flag.flag_BIT.state_process_1=0;
																		flag.flag_BIT.process_1=0;
								
								
								}
							else{
							
							LCD_1_MASTER();}	
					
					}

		else if(GPIO_Pin == GPIO_PIN_4)
		{

			TIM7->CNT =0;
			lcd[0].key = Read_key_scan2();
			if(flag.flag_BIT.grading_mode)
				{
								
					lcd_1_clear();
					lcd_1_gotoxy(3,2);
					lcd_1_puts("Grading Mode");
					lcd_2_clear();
					lcd_2_gotoxy(3,2);
					lcd_2_puts("Grading Mode");
				}
			else if(flag.flag_BIT.LCD1_BUS_BUSY==1)
							{
								LCD2_light_ON;
								lcd[0].lcd_menue=1;
								lcd_2_init();
								lcd_2_clear();
								lcd_2_clear();
								lcd_2_gotoxy(4,1);
								lcd_2_puts("Please Wait");
								flag.flag_BIT.key_enter=1;
								//TIM7->CNT=0;
								//HAL_TIM_Base_Start_IT(&htim7);
								
							}	
							else if(flag.flag_BIT.process_2)
								{
									lcd[0].key = Read_key_scan2();
										if(lcd[0].key==14)
											{
					
												flag.flag_BIT.lcd2_function=1;
												flag.flag_BIT.LCD2_BUS_BUSY=0;
												lcd_2_first_menu();
												
												
												flag.flag_BIT.CELL2_RESTART=0;
												
												flag.flag_BIT.lcd2_function=0;
												flag.flag_BIT.state_process_2=0;
												
												lcd[0].number_position=0;
												flag.flag_BIT.process_2=0;

											}
											// UP key
											else if(lcd[0].key==15)
											{
												step_count_2++;
												process_handler_handler_2(0);
											}
											// down key
											else if(lcd[0].key==16)
											{
												if(step_count_2 >0){step_count_2--;}else {step_count_2 =0;}
												process_handler_handler_2(0);
											}						
								}
								else if(/*function key*/lcd[0].key==13 && lcd[0].lcd_menue ==1 && flag.flag_BIT.lcd1_function==0)
								{
									flag.flag_BIT.lcd2_function=1;
									TIM6->CNT =0;
									flag.flag_BIT.LCD2_BUS_BUSY=1;
									
									lcd_1_not_ready();
									TIM7->CNT=0;
									HAL_TIM_Base_Start_IT(&htim7);
																		lcd_2_init();
																		lcd_2_clear();
																		lcd_2_gotoxy(0,0);
																		lcd_2_puts("1_Start ");
																		lcd_2_gotoxy(0,1);
																		lcd_2_puts("2_Stop ");
																		lcd_2_gotoxy(0,2);
																		lcd_2_puts("3_Restart");
																		lcd_2_gotoxy(0,3);
																		lcd_2_puts("4_Restart Cell");
																		flag.flag_BIT.get_channel_2=0;
																		flag.flag_BIT.state_process_2=0;
																		flag.flag_BIT.process_2=0;
								
								
								}
							else{
							
							LCD_2_MASTER();}
					
					}
		
		

		
		/// debug push buttom
		else if(GPIO_Pin== GPIO_PIN_2)
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		
		
		}
	
}


// timer interupt handeler
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		timer_flag.flag_BIT.timer1=1;
			if(htim ->Instance == TIM6)
				{
				timer_flag.flag_BIT.timer6=1;
				}
	
	
		// for checking conncetion of charge board 
		if(htim ->Instance == TIM5)
			{
				TIM5->CNT=0;
				TIM2->CNT =0;
				timer_flag.flag_BIT.timer5=1;
				}	

			if(htim ->Instance == TIM4)
				{
					timer_flag.flag_BIT.timer4=1;
				}
			
				
			if(htim ->Instance == TIM3)
				{
					timer_flag.flag_BIT.timer3=1;
				}	
			if(htim ->Instance == TIM7){
				timer_flag.flag_BIT.timer7=1;
			}
			
			if(htim ->Instance == TIM2){
				timer_flag.flag_BIT.timer2=1;
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
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_CRC_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

		/**********************************/
		LCD1_light_ON;
LCD2_light_ON;

MX_USART1_UART_Init();
//*************CUMUNOCATION*********
 RESET_handler();
 SEND_command_handler();

 START_handler();
 STOP_handler();
 GET_DATA_handler( );
 first_send =0 ;
 LED_CHANNEL_handler();
 iner_resistance_handler();
CHECK_CHARGER_BOARD();
CHECK_MAIN_BOARD();
READ_SEND();
READ_STATUS();

get_normal_board();

	
	HAL_Delay(1000);
				first_send_1=1;
		first_send_2=1;
 process_handler_handler_2(0);
 process_state_handler_2(0);
 process_handler_handler_1(0);
 process_state_handler_1(0);		
HAL_TIM_Base_Start_IT(&htim2);
HAL_TIM_Base_Start_IT(&htim5);
HAL_TIM_Base_Start_IT(&htim7);
HAL_TIM_Base_Start_IT(&htim6);
HAL_TIM_Base_Start_IT(&htim4);
HAL_TIM_Base_Start_IT(&htim2);		
		


		first_send=0;
		first_send_1=0;
		first_send_2=0;
		step_count_1=0;
		step_count_2=0;
		
		
		/*******************************/
		// read board address
	main_board_address = Read_board_address();
	flag.flag_byte =0;	
	CAN_Filtter_config();
	
	HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING |  \
																CAN_IT_RX_FIFO1_MSG_PENDING | \
																CAN_IT_BUSOFF \
																| CAN_IT_TX_MAILBOX_EMPTY);
																



if(HAL_CAN_Start(&hcan)!= HAL_OK)
{
	Error_Handler();
}

flag.flag_byte =0 ;
timer_flag.flag_byte=0;
lcd[1].lcd_menue=0;
lcd[0].lcd_menue=0;
flag.flag_BIT.process_1=0	;
flag.flag_BIT.process_2=0;
flag.flag_BIT.command_or_data =0 ;

HAL_TIM_Base_Stop_IT(&htim3);
HAL_TIM_Base_Stop_IT(&htim4);
HAL_TIM_Base_Stop_IT(&htim5);
HAL_TIM_Base_Stop_IT(&htim6);
HAL_TIM_Base_Stop_IT(&htim7);
HAL_TIM_Base_Stop_IT(&htim2);
__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);
__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
__HAL_TIM_CLEAR_IT(&htim7,TIM_IT_UPDATE);
TIM3->CNT =0;
TIM4->CNT =0;
TIM5->CNT =0;
TIM6->CNT =0;
TIM2->CNT =0;
TIM7->CNT =0;
MX_USART1_UART_Init();
//huart1.RxState= HAL_UART_STATE_READY;	
__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);	
in_counter =0 ;
out_counter =0;


LCD1_light_ON;
LCD2_light_ON;
lcd_1_init();
lcd_1_clear();
lcd_1_gotoxy(5,1);
lcd_1_puts("WELCOME");

lcd_2_init();
lcd_2_clear();
lcd_2_gotoxy(5,1);
lcd_2_puts("WELCOME");
delay_ms(100);
lcd_2_init();
lcd_2_clear();
lcd_2_gotoxy(0,0);
lcd_2_first_menu();
lcd_1_init();
lcd_1_clear();
lcd_1_gotoxy(0,0);
lcd_1_first_menu();

HAL_Delay(500);
LCD1_light_ON;
LCD2_light_ON;
lcd_1_init();
lcd_1_clear();
lcd_1_gotoxy(0,0);
lcd_1_first_menu();
lcd_2_init();
lcd_2_clear();
lcd_2_gotoxy(0,0);
lcd_2_first_menu();
timer_flag.flag_BIT.timer1=1;
timer_flag.flag_BIT.timer6=1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//timer checking
		

		if(timer_flag.flag_BIT.timer1){
			timer_flag.flag_BIT.timer1=0;
			
			if(timer_flag.flag_BIT.timer6){
							flag.flag_BIT.CELL1_RESTART=0;
							flag.flag_BIT.lcd1_function=0;
							timer_flag.flag_BIT.timer6=0;	
							flag.flag_BIT.get_channel_1=0;
							flag.flag_BIT.state_process_1=0;
							flag.flag_BIT.process_1=0;
							lcd[1].lcd_menue=1;
							flag.flag_BIT.command_or_data=0;
							finish_reacive =0;
							flag.flag_BIT.PC_BUS_BUSY=0;
							flag.flag_BIT.ASK_DATA=0;
							flag.flag_BIT.LCD1_BUS_BUSY=0;
							flag.flag_BIT.get_channel_1=0;
							flag.flag_BIT.state_process_1=0;
							flag.flag_BIT.process_1=0;
							first_send_1=0;
							lcd_1_first_menu();
							lcd_2_first_menu();
							HAL_TIM_Base_Stop_IT(&htim6);
				__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
			}
	
	
			if(timer_flag.flag_BIT.timer7){
						flag.flag_BIT.CELL2_RESTART=0;
						flag.flag_BIT.lcd2_function=0;
						timer_flag.flag_BIT.timer7=0;
						flag.flag_BIT.get_channel_2=0;
						flag.flag_BIT.state_process_2=0;
						flag.flag_BIT.process_2=0;
						lcd[0].lcd_menue=1;
						flag.flag_BIT.LCD2_BUS_BUSY=0;
						flag.flag_BIT.command_or_data=0;
						finish_reacive =0;
						flag.flag_BIT.PC_BUS_BUSY=0;
						flag.flag_BIT.ASK_DATA=0;
						flag.flag_BIT.get_channel_2=0;
						flag.flag_BIT.state_process_2=0;
						flag.flag_BIT.process_2=0;
						first_send_2=0;
						lcd_1_first_menu();
						lcd_2_first_menu();
						HAL_TIM_Base_Stop_IT(&htim7);
				__HAL_TIM_CLEAR_IT(&htim7,TIM_IT_UPDATE);
						
				
			}
			
			
			
		// for checking conncetion of charge board 
		if(timer_flag.flag_BIT.timer5)
			{
				timer_flag.flag_BIT.timer5=0;
				// maybe we need to active the recive command ]
				// it means if this timmer triggerd we can get  an new command
						uint8_t str[2] = {NOT_FOUND_CHARGER, 0};
						str[1]=		charger_board_num ;
						HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
						HAL_UART_Transmit(&huart1,str,2,100);
						HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
						
						if(!flag.flag_BIT.check_charger_board)
						{
							charger_board_num = SET_board_num(charger_board_num);
							first_send=0;
							if(charger_board_num< 64)
							{
								switch(usart_recive_rx[1])
														{
															//send
															case 1:
																TIM5->CNT=0;	
																SEND_command_handler();	
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
															{
																TIM5->CNT=0;
																HAL_TIM_Base_Stop_IT(&htim5);
																__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
																get_data_counter=0;
																uint8_t data[] = {5,0,0,0,0,0,0,0};
																if(charger_board_num<64)
																{
																uint8_t data1[2] = {5,0};
																data1[1] = charger_board_num;
																HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
																HAL_UART_Transmit(&huart1 , data1 ,2,100);
																HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
																TIM5->CNT=0;
																HAL_TIM_Base_Start_IT(&htim5);

																CAN_SEND_DATA(data,charger_board_num+64,8);					
																}
																}
															break;
															case 6:
																break;
															case 7:
																LED_CHANNEL_handler();
															break;
															case 8:
																break;
															case 9:
																break;
															case 10: 
																iner_resistance_handler();
															break;
															case 11:
																CHECK_CHARGER_BOARD();
															break;
															case 12:				
																CHECK_MAIN_BOARD();
															break;
															case 13:
																TIM5->CNT=0;	
																READ_SEND();
																break;
															case 14:
																READ_STATUS();
															break;
															case 15:
															//get_normal_board();

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
							}	
						}
						
						
						else
							{
								charger_board_num++;								
								if(charger_board_num==64)
									{
									HAL_TIM_Base_Stop_IT(&htim2);
									HAL_TIM_Base_Stop_IT(&htim5);
										__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
										__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
									board_channel_get_data =1;
									flag.flag_BIT.command_or_data = 0 ;									
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
								else
									{
													uint8_t str[2] = {0x0B , 0x00};
													uint8_t data[] = {11,0,0,0,0,0,0,0};
													str[1] = charger_board_num;
													// we should decode this part to hexadecimal
													HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
													HAL_UART_Transmit(&huart1,str,2,100);
													HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
													TIM5->CNT =0;
													CAN_SEND_DATA(data,charger_board_num+64,8);
									
									}
							}
						
						

			}
			if(timer_flag.flag_BIT.timer4)
				{
						timer_flag.flag_BIT.timer4=0;
						flag.flag_BIT.CELL1_RESTART=0;
						flag.flag_BIT.LCD1_BUS_BUSY=0;
						flag.flag_BIT.process_1 =0;	
						lcd[1].lcd_menue=1;
						lcd_1_clear();
						lcd_1_gotoxy(0,0);
						flag.flag_BIT.command_or_data=0;
						finish_reacive =0;
						flag.flag_BIT.PC_BUS_BUSY=0;
						flag.flag_BIT.ASK_DATA=0;
						lcd_1_puts("Connection Fail");
						HAL_TIM_Base_Stop_IT(&htim4);
					__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
						HAL_Delay(1000);
						lcd_1_first_menu();
						lcd_2_first_menu();
						
				}

			if(timer_flag.flag_BIT.timer3)
				{
						timer_flag.flag_BIT.timer3=0;
						flag.flag_BIT.CELL2_RESTART=0;
							flag.flag_BIT.LCD2_BUS_BUSY=0;
						flag.flag_BIT.process_2 =0;	
						lcd[0].lcd_menue=1;
						lcd_2_clear();
						lcd_2_gotoxy(0,0);
						flag.flag_BIT.command_or_data=0;
						finish_reacive =0;
						flag.flag_BIT.ASK_DATA=0;
						flag.flag_BIT.PC_BUS_BUSY=0;
						lcd_2_puts("Connection Fail");
						HAL_TIM_Base_Stop_IT(&htim3);
					__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);
						HAL_Delay(1000);
						lcd_2_first_menu();
						lcd_1_first_menu();	
				}	
				
			if(timer_flag.flag_BIT.timer2)
				{
					timer_flag.flag_BIT.timer2=0;
					HAL_TIM_Base_Stop_IT(&htim5);
					HAL_TIM_Base_Stop_IT(&htim2);
					__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
					__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  htim2.Init.Prescaler = 20000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 36000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 65000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 6500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 65000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 6500;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 65000;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 6500;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  htim6.Init.Prescaler = 65000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  htim7.Init.Prescaler = 65000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 65535;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD1_back_light_Pin|LED1_Pin|LCD2_back_light_Pin|LCD1_RW_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD2_RS_Pin|LCD2_RW_Pin|LCD2_EN_Pin|LCD1_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD2_D4_Pin|LCD2_D5_Pin|LCD2_D6_Pin|LCD2_D7_Pin
                          |LCD1_D4_Pin|LCD1_D5_Pin|LCD1_D6_Pin|LCD1_D7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD1_RS_GPIO_Port, LCD1_RS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : KEYPAD1_B_Pin KEYPAD1_A_Pin SW4_Pin SW3_Pin
                           SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = KEYPAD1_B_Pin|KEYPAD1_A_Pin|SW4_Pin|SW3_Pin
                          |SW2_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD1_back_light_Pin LCD2_back_light_Pin LCD1_RW_Pin */
  GPIO_InitStruct.Pin = LCD1_back_light_Pin|LCD2_back_light_Pin|LCD1_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : pb1_Pin */
  GPIO_InitStruct.Pin = pb1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(pb1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD2_A_Pin KEYPAD2_B_Pin KEYPAD2_C_Pin KEYPAD2_D_Pin */
  GPIO_InitStruct.Pin = KEYPAD2_A_Pin|KEYPAD2_B_Pin|KEYPAD2_C_Pin|KEYPAD2_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD2_EXT_Pin */
  GPIO_InitStruct.Pin = KEYPAD2_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEYPAD2_EXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD2_RS_Pin LCD2_RW_Pin LCD2_EN_Pin LCD1_EN_Pin */
  GPIO_InitStruct.Pin = LCD2_RS_Pin|LCD2_RW_Pin|LCD2_EN_Pin|LCD1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD2_D4_Pin LCD2_D5_Pin LCD2_D6_Pin LCD2_D7_Pin
                           LCD1_D4_Pin LCD1_D5_Pin LCD1_D6_Pin LCD1_D7_Pin */
  GPIO_InitStruct.Pin = LCD2_D4_Pin|LCD2_D5_Pin|LCD2_D6_Pin|LCD2_D7_Pin
                          |LCD1_D4_Pin|LCD1_D5_Pin|LCD1_D6_Pin|LCD1_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW8_Pin SW7_Pin SW6_Pin SW5_Pin
                           KEYPAD1_D_Pin KEYPAD1_C_Pin */
  GPIO_InitStruct.Pin = SW8_Pin|SW7_Pin|SW6_Pin|SW5_Pin
                          |KEYPAD1_D_Pin|KEYPAD1_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD1_RS_Pin */
  GPIO_InitStruct.Pin = LCD1_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD1_RS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
//// function for scan board address
//*****************************CAN function START*********************************

void CAN_SEND_DATA(uint8_t *transmit_buffer , uint16_t identifier,uint8_t lenth)
{
	CAN_TxHeaderTypeDef tx_CAN_Init;
	uint32_t TX_mailbox;
	
	//how many byte we want to send must be less than a 9
	tx_CAN_Init.DLC = lenth;
	
	
	// the identifiyer of the massage that we want to send 
	tx_CAN_Init.StdId = identifier;


	// what kind of identifier we want to use 
	//	
		//CAN_ID_STD      for		standard comuncation		          
		//CAN_ID_EXT   		for		for extended comunication
	tx_CAN_Init.IDE =  CAN_ID_STD   ; 

  
	// is it data fram or remote frame 
	// CAN_RTR_DATA  for data fram
	// CAN_RTR_REMOTE for remote frame 
	tx_CAN_Init.RTR = CAN_RTR_DATA;
	
	//send massage
	if(HAL_CAN_AddTxMessage(&hcan,&tx_CAN_Init, transmit_buffer , &TX_mailbox) != HAL_OK)
	{
		Error_Handler();
	}

	
// we don't want to use interupt for sending a message
	
}


		
void CAN_Filtter_config(void)
	{
		CAN_FilterTypeDef can1_filter_Init;
		
		
		
		can1_filter_Init.FilterActivation= ENABLE;
		
		//which of 28 fillter we want to use
		can1_filter_Init.FilterBank=0;
		
		//which of 2 FIFO we want to use 
		can1_filter_Init.FilterFIFOAssignment= CAN_RX_FIFO0;
		
		
		// 32 bit identifier high xxxx****
		can1_filter_Init.FilterIdHigh= 0x0040;
	
		// 32 bit identifier low ****xxxx	
		can1_filter_Init.FilterIdLow= 0x0000;
		
		
		// 32 bit mask high xxxx****
		can1_filter_Init.FilterMaskIdHigh=0x0000;
		
		// 32 bit mask low ****xxxx	
		can1_filter_Init.FilterMaskIdLow=0x0000;
		
		// chose filter type it can be
		//  CAN_FILTERMODE_IDLIST 	for ID MODE
		//  CAN_FILTERMODE_IDMASK 	for MASK MODE
		can1_filter_Init.FilterMode= CAN_FILTERMODE_IDLIST;
	
		
		can1_filter_Init.FilterScale= CAN_FILTERSCALE_32BIT;
		
		
	if(HAL_CAN_ConfigFilter(&hcan,&can1_filter_Init)!= HAL_OK)
			{
			Error_Handler();
			}			

	}
	
	



//Erorr handle in CAN
//*************************************CAN function END***********************************
void SEND_command_handler(void)
	{
		


	uint8_t rady[2]= {0x01,0x00};


		
if(first_send ==0)
	{


		uint8_t data[] = {1,0,0,0,0,0,0,0};


		rady[0]=1;
		rady[1]=charger_board_num;
		first_send = first_send +1;
		HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
		HAL_UART_Transmit(&huart1,rady,2, 100);
		HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
		TIM5->CNT =0;
		HAL_TIM_Base_Start_IT(&htim5);
		CAN_SEND_DATA(data,charger_board_num+64, 8);
		
	}
		
		else{
			if(flag.flag_BIT.CAN_FIFO0)
				{
					flag.flag_BIT.CAN_FIFO0=0;
					TIM5->CNT =0;
					
//					if(HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
//									{
//										Error_Handler();
//									}	
						if(rx_buffer[0] ==1 )
								{
									first_send++;
									if(first_send ==2)
										{
												HAL_TIM_Base_Stop_IT(&htim5);
											__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
												rady[1]=charger_board_num;
												finish_reacive=0;
												in_counter=0;
												out_counter=0;
												flag.flag_BIT.ASK_DATA= 1;
												TIM2->CNT =0;
												HAL_TIM_Base_Start_IT(&htim2);
												HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
												HAL_UART_Transmit(&huart1,rady,2, 100);
												HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
											
										}
									else{
										if(first_send < 98 )
											{
												TIM5->CNT =0;
												HAL_TIM_Base_Start_IT(&htim5);
												CAN_SEND_DATA(send_array[first_send-2],charger_board_num+64, 8);

											}
										else if(first_send == 98)
											{
												HAL_TIM_Base_Stop_IT(&htim5);
												__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
												charger_board_num = SET_board_num(charger_board_num);
												if(charger_board_num < 64)
													{
													uint8_t data[] = {1,0,0,0,0,0,0,0};
													rady[0]=1;
													rady[1]=charger_board_num;
													first_send = 1;
													HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
													HAL_UART_Transmit(&huart1,rady,2, 100);
													HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
													TIM5->CNT =0;
													HAL_TIM_Base_Start_IT(&htim5);
													CAN_SEND_DATA(data,charger_board_num+64, 8);
													}
											
											}	
											}
							}
			
			}

		}

	}

	

void START_handler(void)
	{
			uint8_t data[] = {2,0,0,0,0,0,0,0};
				// for validation of start
			HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			HAL_UART_Transmit(&huart1,data,2, 100);
			HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
	
		
				
				for(register uint8_t cnt=0; cnt<64 ; cnt++)
				{
					if(rx_normal_board[cnt] ==0x11)
						{
								CAN_SEND_DATA(data,cnt+64,8);
								for(uint16_t count=0; count<65000 ; count++);
						}

				}

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
			HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			HAL_UART_Transmit(&huart1,data,2, 100);
			HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
	}



void STOP_handler(void)
	{

			uint8_t data[] = {3,0,0,0,0,0,0,0};
			HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			HAL_UART_Transmit(&huart1,data,2, 100);
			HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
			
			for(register uint8_t cnt=0; cnt<64 ; cnt++)
				{
					if(rx_normal_board[cnt] ==0x11)
						{
								CAN_SEND_DATA(data,cnt+64,8);
								for(uint16_t count=0; count<65000 ; count++);
						}

				}	
			flag.flag_BIT.uart_recive=0;
			
			usart_recive_rx[0] = 0x00;
			usart_recive_rx[1] = 0x00;
			flag.flag_BIT.LCD2_BUS_BUSY=0;
			flag.flag_BIT.LCD1_BUS_BUSY=0;
			flag.flag_BIT.command_or_data=0;
			flag.flag_BIT.ASK_DATA=0;
			finish_reacive =0;
			if(flag.flag_BIT.key_enter)
				{
					lcd_1_first_menu();
					lcd_2_first_menu();
					flag.flag_BIT.key_enter=0;
				}
			HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			HAL_UART_Transmit(&huart1,data,2, 100);
			HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
		
	}



void RESET_handler(void)
	{
		uint8_t data[] = {4,0,0,0,0,0,0,0};

		HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
		HAL_UART_Transmit(&huart1,data,2, 100);
		HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);

		HAL_TIM_Base_Stop_IT(&htim2);
		__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
				for(register uint8_t cnt=0; cnt<64 ; cnt++)
				{
					if(rx_normal_board[cnt] ==0x11)
						{
								CAN_SEND_DATA(data,cnt+64,8);
								for(uint16_t count=0; count<65000 ; count++);
						}

				}			

			

				flag.flag_BIT.uart_recive=0;
				usart_recive_rx[0] = 0x00;
				usart_recive_rx[1] = 0x00;
				flag.flag_BIT.command_or_data=0;
				flag.flag_BIT.ASK_DATA=0;
				finish_reacive =0;
				flag.flag_BIT.LCD2_BUS_BUSY=0;
				flag.flag_BIT.LCD1_BUS_BUSY=0;
				if(flag.flag_BIT.key_enter)
				{
					lcd_1_first_menu();
					lcd_2_first_menu();
					flag.flag_BIT.key_enter=0;
				}
				HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			  HAL_UART_Transmit(&huart1,data,2, 100);
			  HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
	
	
	}
	
	
	
	void	formation_start(void)
	{
			uint8_t data[] = {18,0,0,0,0,0,0,0};
				// for validation of start
			HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			HAL_UART_Transmit(&huart1,data,2, 100);
			HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
			flag.flag_BIT.LCD2_BUS_BUSY=1;
			flag.flag_BIT.LCD1_BUS_BUSY=1;
			flag.flag_BIT.grading_mode=1;
				
				for(register uint8_t cnt=0; cnt<64 ; cnt++)
				{

								CAN_SEND_DATA(data,cnt+64,8);
								for(uint16_t count=0; count<65000 ; count++);
						

				}

			flag.flag_BIT.uart_recive=0;

			usart_recive_rx[0] = 0x00;
			usart_recive_rx[1] = 0x00;
			flag.flag_BIT.command_or_data=0;
			flag.flag_BIT.ASK_DATA=0;
			finish_reacive =0;

				
				
					lcd_1_clear();
	lcd_1_gotoxy(3,2);
	lcd_1_puts("Grading Mode");
						lcd_2_clear();
	lcd_2_gotoxy(3,2);
	lcd_2_puts("Grading Mode");
				
				
				
			HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			HAL_UART_Transmit(&huart1,data,2, 100);
			HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
	}

	
	
	void formation_end(void)
	{
			uint8_t data[] = {19,0,0,0,0,0,0,0};
				// for validation of start
			HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			HAL_UART_Transmit(&huart1,data,2, 100);
			HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);

				
				for(register uint8_t cnt=0; cnt<64 ; cnt++)
				{

								CAN_SEND_DATA(data,cnt+64,8);
								for(uint16_t count=0; count<65000 ; count++);


				}

			flag.flag_BIT.uart_recive=0;
			flag.flag_BIT.LCD2_BUS_BUSY=0;
			flag.flag_BIT.LCD1_BUS_BUSY=0;
			flag.flag_BIT.grading_mode=0;
			usart_recive_rx[0] = 0x00;
			usart_recive_rx[1] = 0x00;
			flag.flag_BIT.command_or_data=0;
			flag.flag_BIT.ASK_DATA=0;
			finish_reacive =0;

					lcd_1_first_menu();
					lcd_2_first_menu();
					flag.flag_BIT.key_enter=0;
				
			HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			HAL_UART_Transmit(&huart1,data,2, 100);
			HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
	}




void GET_DATA_handler(void )
	{
		
		static uint8_t PC_buffer[72];
		   
		uint8_t data[] = {5,0,0,0,0,0,0,0};
		
					flag.flag_BIT.CAN_FIFO0=0; 
					

	
//					if (HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
//								{
//									Error_Handler();
//								}	
							
							uint8_t counter_variable = ((get_data_counter)*8);
							//for(register uint8_t cnt=0 ; cnt<2 ; cnt++)
							//{
								PC_buffer[counter_variable]=rx_buffer[0];
								PC_buffer[counter_variable+1]=rx_buffer[1];
								PC_buffer[counter_variable+2]=rx_buffer[2];
								PC_buffer[counter_variable+3]=rx_buffer[3];
								PC_buffer[counter_variable+4]=rx_buffer[4];
								PC_buffer[counter_variable+5]=rx_buffer[5];
								PC_buffer[counter_variable+6]=rx_buffer[6];
								PC_buffer[counter_variable+7]=rx_buffer[7];
							//}

							get_data_counter++;

							TIM5->CNT =0;
								HAL_TIM_Base_Start_IT(&htim5);
							CAN_SEND_DATA(data,charger_board_num+64,8);
							if(get_data_counter==9)
								{
									uint8_t board_stats[1] = {charger_board_num};
									HAL_TIM_Base_Stop_IT(&htim5);
									__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
									HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
									HAL_UART_Transmit(&huart1,board_stats,1,100);
									HAL_UART_Transmit(&huart1,PC_buffer,72,100);
									HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
									charger_board_num = SET_board_num(charger_board_num);
									get_data_counter=0;
									if(charger_board_num<64)
										{
										uint8_t data1[2] = {5,0};
										data1[1] = charger_board_num;
										HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
										HAL_UART_Transmit(&huart1 , data1 ,2,100);
										HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
										TIM5->CNT=0;
										HAL_TIM_Base_Start_IT(&htim5);
										
										CAN_SEND_DATA(data,charger_board_num+64,8);					
										}
																			
								}
							
							
		/*				}
	}	*/

}



	
void LED_CHANNEL_handler(void)
	{
				uint8_t data[8] = {0x07,0x00, 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00};

		

				
				for(register uint8_t cnt=0; cnt<64 ; cnt++)
				{
					if(rx_normal_board[cnt] ==0x11)
						{
								data[1]=rx_LED[cnt];
								CAN_SEND_DATA(data,cnt+64,8);
								for(uint16_t count=0; count<65000 ; count++);
						}

				}

			flag.flag_BIT.uart_recive=0;
			flag.flag_BIT.LCD2_BUS_BUSY=0;
			flag.flag_BIT.LCD1_BUS_BUSY=0;
			usart_recive_rx[0] = 0x00;
			usart_recive_rx[1] = 0x00;
			flag.flag_BIT.command_or_data=0;
			flag.flag_BIT.ASK_DATA=0;
			finish_reacive =0;
				data[1] =0;
			if(flag.flag_BIT.key_enter)
				{
					lcd_1_first_menu();
					lcd_2_first_menu();
					flag.flag_BIT.key_enter=0;
				}
			HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			HAL_UART_Transmit(&huart1,data,2, 100);
			HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
	}



void iner_resistance_handler(void)
	{
		
		usart_recive_rx[0] = 0x00;
		
		usart_recive_rx[1] = 0x00;		
		flag.flag_BIT.uart_recive=0;
		flag.flag_BIT.LCD2_BUS_BUSY=0;
		flag.flag_BIT.LCD1_BUS_BUSY=0;
				flag.flag_BIT.command_or_data = 0 ;
		finish_reacive =0;
		flag.flag_BIT.ASK_DATA=0;
	}



void CHECK_CHARGER_BOARD(void){


uint8_t data[] = {11,0,0,0,0,0,0,0};
		


	
	if(first_send==0)
		{
		if(flag.flag_BIT.check_charger_board==0 && charger_board_num==0 )
			{
						flag.flag_BIT.check_charger_board=1;			
						uint8_t str[2] = {0x0B , 0x00};
						HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
						HAL_UART_Transmit(&huart1,str,2,100);
						HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
	
	
			}
			uint8_t str[2] = {0x0B , 0x00};
			str[1] = charger_board_num;
			HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			HAL_UART_Transmit(&huart1,str,2,100);
			HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
			TIM5->CNT =0;
			HAL_TIM_Base_Start_IT(&htim5);
			CAN_SEND_DATA(data,charger_board_num+64,8);
			first_send =1;
		}
	else{
		if(flag.flag_BIT.CAN_FIFO0){
				flag.flag_BIT.CAN_FIFO0=0;
				TIM5->CNT =0;
				
				
//				if (HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
//								{
//									Error_Handler();
//								}
				if(rx_buffer[0] ==(charger_board_num+64)  )
					{
						uint8_t str[2] = {0x0B , 0x00};
						str[1] = charger_board_num;
						// we should decode this part to hexadecimal
						HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
						HAL_UART_Transmit(&huart1,str,2,100);
						HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);

					}
				else
					{
						uint8_t str[2] = {NOT_FOUND_CHARGER , 0x00};
						str[1] = charger_board_num;
						// we should decode this part to hexadecimal
						HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
						HAL_UART_Transmit(&huart1,str,2,100);
						HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
						

					}
			charger_board_num ++;
			if(charger_board_num < 64)
				{
						uint8_t str[2] = {0x0B , 0x00};
						str[1] = charger_board_num;
						HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
						HAL_UART_Transmit(&huart1,str,2,100);
						HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
						TIM5->CNT =0;
						HAL_TIM_Base_Start_IT(&htim5);
						CAN_SEND_DATA(data,charger_board_num+64,8);
				
				}
				else
				{
									HAL_TIM_Base_Stop_IT(&htim2);
									HAL_TIM_Base_Stop_IT(&htim5);
									__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
									__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
									board_channel_get_data =1;
									flag.flag_BIT.command_or_data = 0 ;
									
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
		}

	
	}
}



void CHECK_MAIN_BOARD(void)
	{
		uint8_t str[2] = {12,main_board_address};

				
		usart_recive_rx[0] = 0x00;
		usart_recive_rx[1] = 0x00;		
		flag.flag_BIT.uart_recive=0;
		flag.flag_BIT.LCD2_BUS_BUSY=0;
		flag.flag_BIT.LCD1_BUS_BUSY=0;
		HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
		HAL_UART_Transmit(&huart1,str,2,100);	
		HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
		flag.flag_BIT.command_or_data =0;
		flag.flag_BIT.ASK_DATA=0;
		finish_reacive =0;
					if(flag.flag_BIT.key_enter)
				{
					lcd_1_first_menu();
					lcd_2_first_menu();
					flag.flag_BIT.key_enter=0;
				}

	

	}
	
	
void READ_SEND(void)
	{
		uint8_t      data1[]        = {9,0,0,0,0,0,0,0};
		

		
		if(first_send==0)
			{
			

				uint8_t data2[] = {13,0};
				data2[1]  = charger_board_num; 
				HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
				HAL_UART_Transmit(&huart1 , data2 ,2,100);
				HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
				
				TIM5->CNT =0;
				HAL_TIM_Base_Start_IT(&htim5);
				first_send =1;
				send_counter =0;
				CAN_SEND_DATA(data1,charger_board_num+64,8);			
			}
		else {
			if(flag.flag_BIT.CAN_FIFO0)
				{
					flag.flag_BIT.CAN_FIFO0=0;
					TIM5->CNT =0;
					HAL_TIM_Base_Stop_IT(&htim5);
					__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
					// get massage
				
//					if (HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
//							{
//								Error_Handler();
//							}	
							

							for(register uint8_t cnt=0 ; cnt<8 ; cnt++)
							{
								send_array[send_counter][cnt]=rx_buffer[cnt];
							}
						

							// i think we should add hand shake signal in here to insure that PC get the massage 
							// and then send data to charge board
							send_counter++;
							//for(uint8_t count=0; count<100 ; count++);
							TIM5->CNT =0;
							HAL_TIM_Base_Start_IT(&htim5);
							CAN_SEND_DATA(data1,charger_board_num+64,8);
							if(send_counter==96)
								{
								TIM5->CNT =0;
								HAL_TIM_Base_Stop_IT(&htim5);
								__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
								uint8_t board_stats[1] = {charger_board_num};
								HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
								HAL_UART_Transmit(&huart1,board_stats,1,100);	
								for(register uint8_t cnt=0 ; cnt<96 ; cnt++)
								{HAL_UART_Transmit(&huart1,send_array[cnt],8,100);}
									HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
								charger_board_num = SET_board_num(charger_board_num);
								if(charger_board_num < 64)
											{
												uint8_t data2[] = {13,0};
												data2[1]  = charger_board_num; 
												HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
												HAL_UART_Transmit(&huart1 , data2 ,2,100);
												HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
												TIM5->CNT =0;
												HAL_TIM_Base_Start_IT(&htim5);
												send_counter =0;
												CAN_SEND_DATA(data1,charger_board_num+64,8);									
											}
									
																	
								
								}
					  
							
						}
	}	




	}

void READ_STATUS(void)
	{
	
		
		
		uint8_t data[] = {14,0,0,0,0,0,0,0};
			
		
		HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
		HAL_UART_Transmit(&huart1,data,2, 100);
		HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
	
			


		data[1]=1;
		HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
		HAL_UART_Transmit(&huart1,data,2, 100);
		HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
		
		usart_recive_rx[0] = 0x00;
		usart_recive_rx[1] = 0x00;		
		flag.flag_BIT.uart_recive=0;
		flag.flag_BIT.LCD2_BUS_BUSY=0;
		flag.flag_BIT.LCD1_BUS_BUSY=0;		flag.flag_BIT.command_or_data =0;
		flag.flag_BIT.ASK_DATA=0;
		finish_reacive =0;
					if(flag.flag_BIT.key_enter)
				{
					lcd_1_first_menu();
					lcd_2_first_menu();
					flag.flag_BIT.key_enter=0;
				}



	}	
//*************************************CAN function END***********************************
uint8_t Read_board_address(void)
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
uint8_t Read_key_scan1(void)
	{
		const uint8_t key_matrix[16] = {1 ,0,3,13, \
														7, 8, 9,15, \
													 4, 5, 6 , 16,\
														1,2,3,14 };
		KEYPAD_SCAN_t key ;


		key.data_BIT.SW0 = HAL_GPIO_ReadPin(KEYPAD1_A_GPIO_Port,KEYPAD1_A_Pin);
		key.data_BIT.SW1 = HAL_GPIO_ReadPin(KEYPAD1_B_GPIO_Port,KEYPAD1_B_Pin);
		key.data_BIT.SW2 = HAL_GPIO_ReadPin(KEYPAD1_C_GPIO_Port,KEYPAD1_C_Pin);
		key.data_BIT.SW3 = HAL_GPIO_ReadPin(KEYPAD1_D_GPIO_Port,KEYPAD1_D_Pin);
		key.data_BIT.SW4 = 0;
		key.data_BIT.SW5 = 0;
		key.data_BIT.SW6 = 0;
		key.data_BIT.SW7 = 0;
	
		return key_matrix[key.data] ; 
	}	
	
// function for scan KEYPAD 2
uint8_t Read_key_scan2(void)
	{
		KEYPAD_SCAN_t key ;
		const uint8_t key_matrix[16] = {1 ,0,3,13, \
														7, 8, 9,15, \
													 4, 5, 6 , 16,\
														1,2,3,14 };
		
		
		
		key.data_BIT.SW0 = HAL_GPIO_ReadPin(KEYPAD2_A_GPIO_Port,KEYPAD2_A_Pin);
		key.data_BIT.SW1 = HAL_GPIO_ReadPin(KEYPAD2_B_GPIO_Port,KEYPAD2_B_Pin);
		key.data_BIT.SW2 = HAL_GPIO_ReadPin(KEYPAD2_C_GPIO_Port,KEYPAD2_C_Pin);
		key.data_BIT.SW3 = HAL_GPIO_ReadPin(KEYPAD2_D_GPIO_Port,KEYPAD2_D_Pin);
		key.data_BIT.SW4 = 0;
		key.data_BIT.SW5 = 0;
		key.data_BIT.SW6 = 0;
		key.data_BIT.SW7 = 0;
	
		return key_matrix[key.data] ;
	}	




//////////////////////////////////////////////////////////////////////////////////
////////////////////LCD 1 FUNCTION////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////	
void LCD_1_menu(void){
										switch (lcd[1].lcd_menue)
														{
															case  1:
																		lcd_1_init();
																		lcd_1_clear();
																		lcd_1_gotoxy(0,0);
																		lcd_1_puts("Choose");
																		lcd_1_gotoxy(0,1);
																		lcd_1_puts("1_Cell Parameter");
																		lcd_1_gotoxy(0,2);
																		lcd_1_puts("2_Process State");
																		lcd_1_gotoxy(0,3);
																		lcd_1_puts("3_Process");
																		flag.flag_BIT.LCD1_BUS_BUSY=0;
																		flag.flag_BIT.get_channel_1=0;
																		flag.flag_BIT.state_process_1=0;
																		flag.flag_BIT.process_1=0;
																		first_send_1=0;
															break ;
															
															case 2:
																		lcd_1_clear();
																		lcd[1].number_position =0;
																		lcd_1_gotoxy(0,0);
																		lcd[1].counter =0;
																		if(lcd[1].first_menu_choose==1)
																		{
																		lcd_1_puts("Enter Cell Number");
																		}
																		else
																		{
																		lcd_1_puts("Enter Board Number");
																		}
																		TIM6->CNT =0;
																		HAL_TIM_Base_Start_IT(&htim6);
																break;
															case 3:
																		lcd_1_clear();
																		flag.flag_BIT.PC_BUS_BUSY=1;
																		lcd_1_gotoxy(0,1);
																		lcd_1_puts("Wait For Getting A");
																		lcd_1_gotoxy(0,2);
																		lcd_1_puts("Data");
																		LCD_1_get_data(lcd[1]);
																break;
															default:
																		lcd_1_clear();
																		lcd_1_gotoxy(0,0);
																		lcd_1_puts("System Error Please ");
																		lcd_1_gotoxy(0,1);
																		lcd_1_puts("Wait One Moment For Restart");
															break;
														}
	}
/////////  return to back menu 

////////////////  
////////  show number on screne and increse cursur for next number
void LCD_1_READ_NUMBER(uint8_t lcd_key,uint8_t position)
	{
			char str[1];
			lcd[1].board_number[lcd[1].counter]= (lcd[1].key -1);
			lcd[1].counter++;
			
			lcd_1_gotoxy(position,2);
			sprintf(str,"%d",lcd_key);
			lcd_1_puts(str);
	}		


/************************************NOT READY********************/	
void LCD_1_get_data(LCD_t lcd)
	{
		
		//lcd.board_number[1] = board left or right
		//lcd.board_command   = voard command
		//define variable
		uint8_t board_address;
		uint8_t board_channel;
		
		//define function
		board_address = ( 32* lcd.board_number[0] + 4*lcd.board_number[1] + lcd.board_number[2] );
		board_channel = lcd.board_number[3];
		board_address = board_address + 64;
		uint8_t    data[8]        = {6,board_channel,0,0,0,0,0,0};
		first_send_1=0;
		switch (lcd.first_menu_choose)
			{
				// display of each command comes in it,s function 
				// call command => requst from specefic board => get DATA		   } if both of the flag are set display DATA  	
				//                 set request flag								set get data }
				case 1:  // get channel command 
					

				TIM4->CNT =0;
				HAL_TIM_Base_Start_IT(&htim4);
				#if 1
						flag.flag_BIT.uart_recive =1;
						usart_recive_rx[1] = 0x06;		
				#endif
				flag.flag_BIT.CAN_FIFO0=0;
				flag.flag_BIT.get_channel_1=1;
				flag.flag_BIT.LCD1_BUS_BUSY=1;
				CAN_SEND_DATA(data,board_address,8);
					
					break;
				case 2: // board process state command 
					process_state_handler_1( board_address);
					break;
				case 3:	// board process command 
					process_handler_handler_1( board_address);
					break;
				default:
				lcd.lcd_menue =5;
				break;
			
			}
	}			

/************************************NOT READY********************/		
void LCD_1_MASTER(void)
	{	
		if (lcd[1].key==14)
			{
										TIM4->CNT =0;
										HAL_TIM_Base_Stop_IT(&htim4);
										HAL_TIM_Base_Stop_IT(&htim6);
										__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
										__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
										flag.flag_BIT.get_channel_1=0;
										flag.flag_BIT.CELL1_RESTART=0;
										flag.flag_BIT.lcd1_function=0;
										lcd[1].lcd_menue =1;
										flag.flag_BIT.process_1=0;
										flag.flag_BIT.state_process_1=0;
										flag.flag_BIT.get_channel_1=0;
										lcd[1].number_position=0;
										flag.flag_BIT.LCD1_BUS_BUSY=0;
										lcd_1_first_menu();
										lcd_2_first_menu();
										
			}
		else if(flag.flag_BIT.lcd1_function)
		{
		if(flag.flag_BIT.CELL1_RESTART)
			{
			switch(lcd[1].number_position)
					{
					case 0:
						if(lcd[1].key<3 && lcd[1].key>0)
								{
									lcd[1].number_position++;	
									LCD_1_READ_NUMBER(lcd[1].key , lcd[1].number_position);
								}
						break;
					
					case 1:
							if(lcd[1].key<9  && lcd[1].key>0)
								{
									lcd[1].number_position++;	
									LCD_1_READ_NUMBER(lcd[1].key , lcd[1].number_position);
								}
						break;
					
					case 2:
							if(lcd[1].key<5  && lcd[1].key>0)
								{
									lcd[1].number_position++;	
									LCD_1_READ_NUMBER(lcd[1].key , lcd[1].number_position);

								}
						break;
					
					case 3: 
							if(lcd[1].key<9 && lcd[1].key>0)
								{
									lcd[1].number_position++;	
									LCD_1_READ_NUMBER(lcd[1].key , lcd[1].number_position);
									lcd[1].number_position =0;
											uint8_t board_address;
											uint8_t board_channel;
											flag.flag_BIT.CELL1_RESTART=0;
											HAL_TIM_Base_Stop_IT(&htim6);
											__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
											flag.flag_BIT.PC_BUS_BUSY=1;
											//define function
											board_address = ( 32* lcd[1].board_number[0] + 4*lcd[1].board_number[1] + lcd[1].board_number[2] );
											board_channel = lcd[1].board_number[3];
											board_address = board_address + 64;
											uint8_t    data[8]        = {17,board_channel,0,0,0,0,0,0};
											CAN_SEND_DATA(data,board_address,8);
											lcd_1_clear();
											lcd_1_gotoxy(2,1);
											lcd_1_puts("Cell Restarted");
											flag.flag_BIT.PC_BUS_BUSY=0;
											delay_ms(250);
											flag.flag_BIT.LCD1_BUS_BUSY=0;
											lcd_2_first_menu();
											lcd_1_first_menu();
											flag.flag_BIT.lcd1_function=0;
								
								}
						
						break;
					default:
							// we can use this part when the ENTER KEY

								
						break;
					}

			
			
			
			}
			else if(flag.flag_BIT.CELL1_RESTART==0){
				switch(lcd[1].key)
				{
					case 1:
					{
					uint8_t data[] = {2,0,0,0,0,0,0,0};		
		
					flag.flag_BIT.PC_BUS_BUSY=1;
					for(register uint8_t cnt=0; cnt<64 ; cnt++)
						{
							if(rx_normal_board[cnt] ==0x11)
								{
										CAN_SEND_DATA(data,cnt+64,8);
										for(uint16_t count=0; count<65000 ; count++);
								}

						}
					}	
					flag.flag_BIT.PC_BUS_BUSY=0;
					flag.flag_BIT.LCD1_BUS_BUSY=0;
					HAL_TIM_Base_Stop_IT(&htim6);
					__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
					flag.flag_BIT.CELL1_RESTART=0;
					lcd_1_clear();
					lcd_1_gotoxy(4,1);
					lcd_1_puts("Unit Started");
					delay_ms(250);
					lcd_1_first_menu();
					lcd_2_first_menu();
					flag.flag_BIT.lcd1_function=0;
						break;
					case 2:
					{
					uint8_t data[] = {3,0,0,0,0,0,0,0};		
						
					flag.flag_BIT.PC_BUS_BUSY=1;					
					for(register uint8_t cnt=0; cnt<64 ; cnt++)
						{
							if(rx_normal_board[cnt] ==0x11)
								{
										CAN_SEND_DATA(data,cnt+64,8);
										for(uint16_t count=0; count<65000 ; count++);
								}

						}
						flag.flag_BIT.PC_BUS_BUSY=0;
						flag.flag_BIT.CELL1_RESTART=0;
						flag.flag_BIT.LCD1_BUS_BUSY=0;
						HAL_TIM_Base_Stop_IT(&htim6);
						__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
					lcd_1_clear();
					lcd_1_gotoxy(4,1);
					lcd_1_puts("Unit Stop");
					delay_ms(250);
					lcd_1_first_menu();
					lcd_2_first_menu();
					flag.flag_BIT.lcd1_function=0;
						break;}
					case 3:
					{
					uint8_t data[] = {16,0,0,0,0,0,0,0};
		
					flag.flag_BIT.PC_BUS_BUSY=1;
					for(register uint8_t cnt=0; cnt<64 ; cnt++)
						{
							if(rx_normal_board[cnt] ==0x11)
								{
										CAN_SEND_DATA(data,cnt+64,8);
										for(uint16_t count=0; count<65000 ; count++);
								}

						}
					flag.flag_BIT.PC_BUS_BUSY=0;
					flag.flag_BIT.LCD1_BUS_BUSY=0;
					lcd_1_clear();
					lcd_1_gotoxy(2,1);
					lcd_1_puts("Unit Restarded");
					delay_ms(250);
					lcd_1_first_menu();
					lcd_2_first_menu();
					flag.flag_BIT.lcd1_function=0;
						break;}	
					case 4:
					flag.flag_BIT.CELL1_RESTART=1;
					lcd_1_clear();
					lcd_1_gotoxy(0,0);
					
					lcd_1_puts("Enter Cell Number");
					lcd[1].number_position =0;lcd[1].counter =0;
						break;
					default:
						break;
				}
			
			}
		
		}
		else if(lcd[1].lcd_menue==2)
			{
				switch(lcd[1].number_position)
					{
					case 0:
						if(lcd[1].key<3 && lcd[1].key>0)
								{
									lcd[1].number_position++;	
									LCD_1_READ_NUMBER(lcd[1].key , lcd[1].number_position);
								}
						break;
					
					case 1:
							if(lcd[1].key<9  && lcd[1].key>0)
								{
									lcd[1].number_position++;	
									LCD_1_READ_NUMBER(lcd[1].key , lcd[1].number_position);
								}
						break;
					
					case 2:
							if(lcd[1].key<5  && lcd[1].key>0)
								{
									lcd[1].number_position++;	
									LCD_1_READ_NUMBER(lcd[1].key , lcd[1].number_position);
									if(lcd[1].first_menu_choose >1)
										{
													lcd[1].number_position =0;
													lcd[1].lcd_menue++;
													LCD_1_menu();
										}	
								}
						break;
					
					case 3: 
							if(lcd[1].key<9 && lcd[1].key>0)
								{
									lcd[1].number_position++;	
									LCD_1_READ_NUMBER(lcd[1].key , lcd[1].number_position);
									lcd[1].number_position =0;
									lcd[1].lcd_menue++;
									
									LCD_1_menu();
								}
						
						break;
					default:
							// we can use this part when the ENTER KEY

								
						break;
					}

			}
		else if(lcd[1].lcd_menue==1)
		 {
				if(lcd[1].key<4 && lcd[1].key>0)
				{
					lcd[1].first_menu_choose =lcd[1].key;
					flag.flag_BIT.LCD1_BUS_BUSY=1;
					lcd_2_not_ready();
					lcd[1].lcd_menue++;
					TIM6->CNT =0;
					HAL_TIM_Base_Start_IT(&htim6);
					LCD_1_menu();					
				}
		 }
		else if(lcd[1].lcd_menue==0)
		 {
			// timer start 
			 LCD1_light_ON;	
			 lcd[1].lcd_menue++;

			 LCD_1_menu();

		 }	 
	}	


	
	
	
	
void process_state_handler_1(uint8_t board_number)
	{
		// this command call with LCD_MASTER AND request DATA from charger board 
		// it worth to mention that this command only Send DATA to charger board
		// when DATA receive WE check that this is GET CHANNEL command and if so that writh that on LCD 

		             uint8_t    data[]        = {8,0,0,0,0,0,0,0};
		static       process_state_t    buffer;
		static 			 uint8_t    board_first_choose;	
					       
					       uint8_t    cnt;
					 
					 
					 
		if(first_send_1==0)
			{
				board_first_choose =	board_number;
				first_send_1=1;
				TIM4->CNT =0;
				HAL_TIM_Base_Start_IT(&htim4);
				flag.flag_BIT.CAN_FIFO0=0;
				flag.flag_BIT.state_process_1=1;
				flag.flag_BIT.LCD1_BUS_BUSY=1;
					#if 1
						flag.flag_BIT.uart_recive =1;
						usart_recive_rx[1] = 0x08;		
				#endif
				CAN_SEND_DATA(data,board_number,8);
			}
		else{
			if(flag.flag_BIT.CAN_FIFO0)
			{
				flag.flag_BIT.CAN_FIFO0=0;
						// get massage
//						if (HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
//						{
//							Error_Handler();
//						}	
						TIM4->CNT =0;

						 switch (first_send_1)
							 {
								case 1:
									for(cnt=0; cnt<8;cnt++)
									{buffer.main[cnt] =rx_buffer[cnt];}
									CAN_SEND_DATA(data,board_first_choose,8);
									first_send_1++;
								
									break;
								case 2:
									for(cnt=0; cnt<8;cnt++)
									{buffer.main[cnt+8] =rx_buffer[cnt];}
									CAN_SEND_DATA(data,board_first_choose,8);
									first_send_1++;
								break;									
								case 3:
									for(cnt=0; cnt<2;cnt++)
									{buffer.main[cnt+16] =rx_buffer[cnt];}
									first_send_1=0;
									flag.flag_BIT.state_process_1=0;
									uint8_t str[20];
									flag.flag_BIT.PC_BUS_BUSY=0;
									lcd_1_clear();
									lcd_1_gotoxy(0,0);	
									flag.flag_BIT.LCD1_BUS_BUSY=0;
									lcd_2_first_menu();
									HAL_TIM_Base_Stop_IT(&htim4);
									__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
									switch(buffer.param.command)
										{	
										case 0:
											sprintf((char *)str,"There Is No Process");
											lcd_1_puts((char *)str);
											break;
										case 1:													
												sprintf((char *)str,"CC");
												lcd_1_puts((char *)str);
												lcd_1_gotoxy(0,1);
										sprintf((char *)str,"Time:%d:%d",(buffer.param.time/60),(buffer.param.time%60));
												lcd_1_puts((char *)str);	
												lcd_1_gotoxy(11,1);
												sprintf((char *)str,"I:%d",buffer.param.current);
												lcd_1_puts((char *)str);
												lcd_1_gotoxy(0,2);
												sprintf((char *)str,"MaxV:%d",buffer.param.maxV);
												lcd_1_puts((char *)str);
												lcd_1_gotoxy(11,2);
												sprintf((char *)str,"DV:%d",buffer.param.deltav);
												lcd_1_puts((char *)str);
												lcd_1_gotoxy(0,3);
												sprintf((char *)str,"Cend:%d",buffer.param.Cend);
												lcd_1_puts((char *)str);
										break;
										case 2:
												sprintf((char *)str,"CCCV");
												lcd_1_puts((char *)str);												
												lcd_1_gotoxy(0,1);
										sprintf((char *)str,"Time:%d:%d",(buffer.param.time/60),(buffer.param.time%60));
												lcd_1_puts((char *)str);											
												lcd_1_gotoxy(11,1);
												sprintf((char *)str,"I:%d",buffer.param.current);
												lcd_1_puts((char *)str);
												lcd_1_gotoxy(0,2);
												sprintf((char *)str,"MaxV:%d",buffer.param.maxV);
												lcd_1_puts((char *)str);
												lcd_1_gotoxy(11,2);
												sprintf((char *)str,"DV:%d",buffer.param.deltav);
												lcd_1_puts((char *)str);
												lcd_1_gotoxy(0,3);
												sprintf((char *)str,"IendV:%d",buffer.param.Iend);
												lcd_1_puts((char *)str);
												lcd_1_gotoxy(11,3);
												sprintf((char *)str,"Cend:%d",buffer.param.Cend);
												lcd_1_puts((char *)str);
										break;
										case 3:
												sprintf((char *)str,"Rest");
												lcd_1_puts((char *)str);												
												lcd_1_gotoxy(0,1);
										sprintf((char *)str,"Time:%d:%d",(buffer.param.time/60),(buffer.param.time%60));
												lcd_1_puts((char *)str);										
										break;
										case 4:
												sprintf((char *)str,"CD");
												lcd_1_puts((char *)str);												
												lcd_1_gotoxy(0,1);
												sprintf((char *)str,"Time:%d:%d",(buffer.param.time/60),(buffer.param.time%60));
												lcd_1_puts((char *)str);										
												lcd_1_gotoxy(11,1);
												sprintf((char *)str,"I:%d",buffer.param.current);
												lcd_1_puts((char *)str);
												lcd_1_gotoxy(0,2);
												sprintf((char *)str,"MinV:%d",buffer.param.minV);
												lcd_1_puts((char *)str);
												lcd_1_gotoxy(11,2);
												sprintf((char *)str,"DV:%d",buffer.param.deltav);
												lcd_1_puts((char *)str);
												lcd_1_gotoxy(11,3);
												sprintf((char *)str,"Cend:%d",buffer.param.Cend);
												lcd_1_puts((char *)str);
										break;
										case 5:
												sprintf((char *)str,"Cycle");
												lcd_1_puts((char *)str);												
												lcd_1_gotoxy(0,2);
												sprintf((char *)str,"Cycle:%d",buffer.param.cycle);
												lcd_1_puts((char *)str);										

										break;
										case 6:
												sprintf((char *)str,"END");
												lcd_1_puts((char *)str);												
										
										break;											
										default:
												sprintf((char *)str,"Error Frame");
												lcd_1_puts((char *)str);
										break;
										}
								break;									
							 }
						
						
						}
	
			
			}
	}	


void process_handler_handler_1(uint8_t board_num)
	{
				uint8_t      data[]        = {9,0,0,0,0,0,0,0};
				static  		 process_t  process_buffer;
				static 			 uint8_t    board_first_choose;
			  		 						 
								
										 uint8_t    cnt;
				
				
			if(first_send_1==0)
				{
					board_first_choose =	board_num;
						#if 1
						
						flag.flag_BIT.uart_recive =1;

						usart_recive_rx[1] = 0x09;		
						#endif
					first_send_1=1;
					step_count_1 =0;
					TIM4->CNT =0;
					HAL_TIM_Base_Start_IT(&htim4);
					flag.flag_BIT.CAN_FIFO0=0;
					flag.flag_BIT.process_1=1;
					flag.flag_BIT.LCD1_BUS_BUSY=1;
				
					CAN_SEND_DATA(data,board_num,8);
				}
		
				else
				{
					if(flag.flag_BIT.CAN_FIFO0)
					{
						flag.flag_BIT.CAN_FIFO0=0;
								// get massage
//								if (HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
//								{
//									Error_Handler();
//								}	
									TIM4->CNT =0;
									 switch (first_send_1)
										 {
											case 1:
												for(cnt=0; cnt<8;cnt++)
												{process_buffer.step[step_count_1].main[cnt] =rx_buffer[cnt];}
												first_send_1++;
												CAN_SEND_DATA(data,board_first_choose,8);
												break;
											case 2:
												for(cnt=0; cnt<8;cnt++)
												{process_buffer.step[step_count_1].main[cnt+8] =rx_buffer[cnt];}
												CAN_SEND_DATA(data,board_first_choose,8);
												first_send_1++;
											break;									
											case 3:
												for(cnt=0; cnt<2;cnt++)
												{process_buffer.step[step_count_1].main[cnt+16] =rx_buffer[cnt];}

												if(step_count_1==31)
													{																				
														lcd_1_clear();
														lcd_1_gotoxy(0,0);
														step_count_1=0;
														first_send_1=4;
														flag.flag_BIT.LCD1_BUS_BUSY=0;
														HAL_TIM_Base_Stop_IT(&htim4);
														__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
													}
												else
													{
														first_send_1=1;
														step_count_1++;
														CAN_SEND_DATA(data,board_first_choose,8);
													
													}
												break;

											
										 }
							}
						if(first_send_1==4)
							{
										
									uint8_t str[20];
									flag.flag_BIT.LCD1_BUS_BUSY=0;
								flag.flag_BIT.PC_BUS_BUSY=0;
								lcd_2_first_menu();
								lcd_1_clear();
								
								if(process_buffer.step[(step_count_1-1)].param.command ==6){step_count_1 = (step_count_1 -1);}
									switch(process_buffer.step[step_count_1].param.command)
													{	
													case 0:
														sprintf((char *)str,"There Is No Process");
														lcd_1_puts((char *)str);
													break;															
													case 1:													
															sprintf((char *)str,"CC");
															lcd_1_puts((char *)str);
															lcd_1_gotoxy(0,1);
													sprintf((char *)str,"Time:%d:%d",(process_buffer.step[step_count_1].param.time/60),(process_buffer.step[step_count_1].param.time%60));
															lcd_1_puts((char *)str);	
															lcd_1_gotoxy(11,1);
															sprintf((char *)str,"I:%d",process_buffer.step[step_count_1].param.current);
															lcd_1_puts((char *)str);
															lcd_1_gotoxy(0,2);
															sprintf((char *)str,"MaxV:%d",process_buffer.step[step_count_1].param.maxV);
															lcd_1_puts((char *)str);
															lcd_1_gotoxy(11,2);
															sprintf((char *)str,"DV:%d",process_buffer.step[step_count_1].param.deltav);
															lcd_1_puts((char *)str);
															lcd_1_gotoxy(0,3);
															sprintf((char *)str,"Cend:%d",process_buffer.step[step_count_1].param.Cend);
															lcd_1_puts((char *)str);
													break;
													case 2:
															sprintf((char *)str,"CCCV");
															lcd_1_puts((char *)str);												
															lcd_1_gotoxy(0,1);
													sprintf((char *)str,"Time:%d:%d",(process_buffer.step[step_count_1].param.time/60),(process_buffer.step[step_count_1].param.time%60));
															lcd_1_puts((char *)str);											
															lcd_1_gotoxy(11,1);
															sprintf((char *)str,"I:%d",process_buffer.step[step_count_1].param.current);
															lcd_1_puts((char *)str);
															lcd_1_gotoxy(0,2);
															sprintf((char *)str,"MaxV:%d",process_buffer.step[step_count_1].param.maxV);
															lcd_1_puts((char *)str);
															lcd_1_gotoxy(11,2);
															sprintf((char *)str,"DV:%d",process_buffer.step[step_count_1].param.deltav);
															lcd_1_puts((char *)str);
															lcd_1_gotoxy(0,3);
															sprintf((char *)str,"IendV:%d",process_buffer.step[step_count_1].param.Iend);
															lcd_1_puts((char *)str);
															lcd_1_gotoxy(11,3);
															sprintf((char *)str,"Cend:%d",process_buffer.step[step_count_1].param.Cend);
															lcd_1_puts((char *)str);
													break;
													case 3:
															sprintf((char *)str,"Rest");
															lcd_1_puts((char *)str);												
															lcd_1_gotoxy(0,1);
													sprintf((char *)str,"Time:%d:%d",(process_buffer.step[step_count_1].param.time/60),(process_buffer.step[step_count_1].param.time%60));
															lcd_1_puts((char *)str);										
													break;
													case 4:
															sprintf((char *)str,"CD");
															lcd_1_puts((char *)str);												
															lcd_1_gotoxy(0,1);
													sprintf((char *)str,"Time:%d:%d",(process_buffer.step[step_count_1].param.time/60),(process_buffer.step[step_count_1].param.time%60));
															lcd_1_puts((char *)str);										
															lcd_1_gotoxy(11,1);
															sprintf((char *)str,"I:%d",process_buffer.step[step_count_1].param.current);
															lcd_1_puts((char *)str);
															lcd_1_gotoxy(0,2);
															sprintf((char *)str,"MinV:%d",process_buffer.step[step_count_1].param.minV);
															lcd_1_puts((char *)str);
															lcd_1_gotoxy(11,2);
															sprintf((char *)str,"DV:%d",process_buffer.step[step_count_1].param.deltav);
															lcd_1_puts((char *)str);
															lcd_1_gotoxy(11,3);
															sprintf((char *)str,"Cend:%d",process_buffer.step[step_count_1].param.Cend);
															lcd_1_puts((char *)str);
													break;
													case 5:
															sprintf((char *)str,"Cycle");
															lcd_1_puts((char *)str);												
															lcd_1_gotoxy(0,2);
															sprintf((char *)str,"Cycle:%d",process_buffer.step[step_count_1].param.cycle);
															lcd_1_puts((char *)str);										

													break;
													case 6:
															sprintf((char *)str,"END");
															lcd_1_puts((char *)str);												
													
													break;											
													default:
															sprintf((char *)str,"Error Frame");
															lcd_1_puts((char *)str);
													break;
													}

								
							}
					
							
						}
				}
//////////////////////////////////////////////////////////////////////////////////
////////////////////LCD 2 FUNCTION////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void LCD_2_menu(void){
										switch (lcd[0].lcd_menue)
														{
															case  1:
																		lcd_2_init();
																		lcd_2_clear();
																		lcd_2_gotoxy(0,0);
																		lcd_2_puts("Choose");
																		lcd_2_gotoxy(0,1);
																		lcd_2_puts("1_Cell Parameter");
																		lcd_2_gotoxy(0,2);
																		lcd_2_puts("2_Process State");
																		lcd_2_gotoxy(0,3);
																		lcd_2_puts("3_Process");
																		flag.flag_BIT.LCD2_BUS_BUSY=0;
																		flag.flag_BIT.get_channel_2=0;
																		flag.flag_BIT.state_process_2=0;
																		flag.flag_BIT.process_2=0;
																		first_send_2=0;
															break ;
															
															case 2:
																		lcd_2_clear();
																		lcd[0].number_position =0;
																		lcd_2_gotoxy(0,0);
																		lcd[0].counter =0;
																		if(lcd[0].first_menu_choose==1)
																		{
																		lcd_2_puts("Enter Cell Number");
																		}
																		else
																		{
																		lcd_2_puts("Enter Board Number");
																		}
																		TIM7->CNT =0;
																		HAL_TIM_Base_Start_IT(&htim7);
																break;
															case 3:
																		lcd_2_clear();
																		flag.flag_BIT.PC_BUS_BUSY=1;
																		lcd_2_gotoxy(0,1);
																		lcd_2_puts("Wait For Getting A");
																		lcd_2_gotoxy(0,2);
																		lcd_2_puts("Data");
																		LCD_2_get_data(lcd[0]);
																break;
															default:
																		lcd_2_clear();
																		lcd_2_gotoxy(0,0);
																		lcd_2_puts("System Error Please ");
																		lcd_2_gotoxy(0,1);
																		lcd_2_puts("Wait One Moment For Restart");
															break;
														}
	}
/////////  return to back menu 

////////////////  
////////  show number on screne and increse cursur for next number
void LCD_2_READ_NUMBER(uint8_t lcd_key,uint8_t position)
	{
			char str[1];
			lcd[0].board_number[lcd[0].counter]= (lcd[0].key -1);
			lcd[0].counter++;
			
			lcd_2_gotoxy(position,2);
			sprintf(str,"%d",lcd_key);
			lcd_2_puts(str);
	}		


/************************************NOT READY********************/	
void LCD_2_get_data(LCD_t lcd)
	{
		
		//lcd.board_number[1] = board left or right
		//lcd.board_command   = voard command
		//define variable
		uint8_t board_address;
		uint8_t board_channel;
		
		//define function
		board_address = ( 32* lcd.board_number[0] + 4*lcd.board_number[1] + lcd.board_number[2] );
		board_channel = lcd.board_number[3];
		board_address = board_address + 64;
		uint8_t    data[8]        = {6,board_channel,0,0,0,0,0,0};
		first_send_2=0;
		switch (lcd.first_menu_choose)
			{
				// display of each command comes in it,s function 
				// call command => requst from specefic board => get DATA		   } if both of the flag are set display DATA  	
				//                 set request flag								set get data }
				case 1:  // get channel command 
					

				TIM3->CNT =0;
				HAL_TIM_Base_Start_IT(&htim3);
				#if 1
						flag.flag_BIT.uart_recive =1;
						usart_recive_rx[1] = 0x06;		
				#endif
				flag.flag_BIT.CAN_FIFO0=0;
				flag.flag_BIT.get_channel_2=1;
				flag.flag_BIT.LCD2_BUS_BUSY=1;
				CAN_SEND_DATA(data,board_address,8);
					
					break;
				case 2: // board process state command 
					process_state_handler_2( board_address);
					break;
				case 3:	// board process command 
					process_handler_handler_2( board_address);
					break;
				default:
				lcd.lcd_menue =5;
				break;
			
			}
	}			

/************************************NOT READY********************/		
void LCD_2_MASTER(void)
	{	
		if (lcd[0].key==14)
			{
										TIM3->CNT =0;
										HAL_TIM_Base_Stop_IT(&htim3);
										HAL_TIM_Base_Stop_IT(&htim7);
										__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);
										__HAL_TIM_CLEAR_IT(&htim7,TIM_IT_UPDATE);
										flag.flag_BIT.get_channel_2=0;
										flag.flag_BIT.CELL2_RESTART=0;
										flag.flag_BIT.lcd2_function=0;
										lcd[0].lcd_menue =1;
										flag.flag_BIT.process_2=0;
										flag.flag_BIT.state_process_2=0;
										flag.flag_BIT.get_channel_2=0;
										lcd[0].number_position=0;
										flag.flag_BIT.LCD2_BUS_BUSY=0;
										lcd_1_first_menu();
										lcd_2_first_menu();
										
			}
		else if(flag.flag_BIT.lcd2_function)
		{
		if(flag.flag_BIT.CELL2_RESTART)
			{
			switch(lcd[0].number_position)
					{
					case 0:
						if(lcd[0].key<3 && lcd[0].key>0)
								{
									lcd[0].number_position++;	
									LCD_2_READ_NUMBER(lcd[0].key , lcd[0].number_position);
								}
						break;
					
					case 1:
							if(lcd[0].key<9  && lcd[0].key>0)
								{
									lcd[0].number_position++;	
									LCD_2_READ_NUMBER(lcd[0].key , lcd[0].number_position);
								}
						break;
					
					case 2:
							if(lcd[0].key<5  && lcd[0].key>0)
								{
									lcd[0].number_position++;	
									LCD_2_READ_NUMBER(lcd[0].key , lcd[0].number_position);

								}
						break;
					
					case 3: 
							if(lcd[0].key<9 && lcd[0].key>0)
								{
									lcd[0].number_position++;	
									LCD_2_READ_NUMBER(lcd[0].key , lcd[0].number_position);
									lcd[0].number_position =0;
											uint8_t board_address;
											uint8_t board_channel;
											flag.flag_BIT.CELL2_RESTART=0;
											HAL_TIM_Base_Stop_IT(&htim7);
											__HAL_TIM_CLEAR_IT(&htim7,TIM_IT_UPDATE);
											flag.flag_BIT.PC_BUS_BUSY=1;
											//define function
											board_address = ( 32* lcd[0].board_number[0] + 4*lcd[0].board_number[1] + lcd[0].board_number[2] );
											board_channel = lcd[0].board_number[3];
											board_address = board_address + 64;
											uint8_t    data[8]        = {17,board_channel,0,0,0,0,0,0};
											CAN_SEND_DATA(data,board_address,8);
											lcd_2_clear();
											lcd_2_gotoxy(2,1);
											lcd_2_puts("Cell Restarted");
											flag.flag_BIT.PC_BUS_BUSY=0;
											flag.flag_BIT.lcd2_function=0;
											delay_ms(250);
											flag.flag_BIT.LCD2_BUS_BUSY=0;
											lcd_2_first_menu();
											lcd_1_first_menu();
											
								
								}
						
						break;
					default:
							// we can use this part when the ENTER KEY

								
						break;
					}

			
			
			
			}
			else if(flag.flag_BIT.CELL2_RESTART==0 &&flag.flag_BIT.lcd2_function){
				switch(lcd[0].key)
				{
					case 1:
					{
					uint8_t data[] = {2,0,0,0,0,0,0,0};		
		
					flag.flag_BIT.PC_BUS_BUSY=1;
					for(register uint8_t cnt=0; cnt<64 ; cnt++)
						{
							if(rx_normal_board[cnt] ==0x11)
								{
										CAN_SEND_DATA(data,cnt+64,8);
										for(uint16_t count=0; count<65000 ; count++);
								}

						}
					}	
					flag.flag_BIT.PC_BUS_BUSY=0;
					flag.flag_BIT.LCD2_BUS_BUSY=0;
					HAL_TIM_Base_Stop_IT(&htim7);
					__HAL_TIM_CLEAR_IT(&htim7,TIM_IT_UPDATE);
					
					lcd_2_clear();
					lcd_2_gotoxy(4,1);
					flag.flag_BIT.lcd2_function=0;
					lcd_2_puts("Unit Started");
					delay_ms(250);
					lcd_1_first_menu();
					lcd_2_first_menu();
					
						break;
					case 2:
					{
					uint8_t data[] = {3,0,0,0,0,0,0,0};		
						
					flag.flag_BIT.PC_BUS_BUSY=1;					
					for(register uint8_t cnt=0; cnt<64 ; cnt++)
						{
							if(rx_normal_board[cnt] ==0x11)
								{
										CAN_SEND_DATA(data,cnt+64,8);
										for(uint16_t count=0; count<65000 ; count++);
								}

						}
						flag.flag_BIT.PC_BUS_BUSY=0;
						
						flag.flag_BIT.LCD2_BUS_BUSY=0;
						HAL_TIM_Base_Stop_IT(&htim7);
						__HAL_TIM_CLEAR_IT(&htim7,TIM_IT_UPDATE);
					lcd_2_clear();
					lcd_2_gotoxy(4,1);
					flag.flag_BIT.lcd2_function=0;
					lcd_2_puts("Unit Stop");
					delay_ms(250);
					lcd_1_first_menu();
					lcd_2_first_menu();
					
						break;}
					case 3:
					{
					uint8_t data[] = {16,0,0,0,0,0,0,0};
		
					flag.flag_BIT.PC_BUS_BUSY=1;
					for(register uint8_t cnt=0; cnt<64 ; cnt++)
						{
							if(rx_normal_board[cnt] ==0x11)
								{
										CAN_SEND_DATA(data,cnt+64,8);
										for(uint16_t count=0; count<65000 ; count++);
								}

						}
					flag.flag_BIT.PC_BUS_BUSY=0;
					flag.flag_BIT.LCD2_BUS_BUSY=0;
					lcd_2_clear();
					lcd_2_gotoxy(2,1);
						flag.flag_BIT.lcd2_function=0;
					lcd_2_puts("Unit Restarded");
					delay_ms(250);
					lcd_1_first_menu();
					lcd_2_first_menu();
					
						break;}	
					case 4:
					flag.flag_BIT.CELL2_RESTART=1;
					lcd_2_clear();
					lcd_2_gotoxy(0,0);
					
					lcd_2_puts("Enter Cell Number");
					lcd[0].number_position =0;lcd[0].counter =0;
						break;
					default:
						break;
				}
			
			}
		
		}
		else if(lcd[0].lcd_menue==2)
			{
				switch(lcd[0].number_position)
					{
					case 0:
						if(lcd[0].key<3 && lcd[0].key>0)
								{
									lcd[0].number_position++;	
									LCD_2_READ_NUMBER(lcd[0].key , lcd[0].number_position);
								}
						break;
					
					case 1:
							if(lcd[0].key<9  && lcd[0].key>0)
								{
									lcd[0].number_position++;	
									LCD_2_READ_NUMBER(lcd[0].key , lcd[0].number_position);
								}
						break;
					
					case 2:
							if(lcd[0].key<5  && lcd[0].key>0)
								{
									lcd[0].number_position++;	
									LCD_2_READ_NUMBER(lcd[0].key , lcd[0].number_position);
									if(lcd[0].first_menu_choose >1)
										{
													lcd[0].number_position =0;
													lcd[0].lcd_menue++;
													LCD_2_menu();
										}	
								}
						break;
					
					case 3: 
							if(lcd[0].key<9 && lcd[0].key>0)
								{
									lcd[0].number_position++;	
									LCD_2_READ_NUMBER(lcd[0].key , lcd[0].number_position);
									lcd[0].number_position =0;
									lcd[0].lcd_menue++;
									
									LCD_2_menu();
								}
						
						break;
					default:
							// we can use this part when the ENTER KEY

								
						break;
					}

			}
		else if(lcd[0].lcd_menue==1)
		 {
				if(lcd[0].key<4 && lcd[0].key>0)
				{
					lcd[0].first_menu_choose =lcd[0].key;
					flag.flag_BIT.LCD2_BUS_BUSY=1;
					lcd_1_not_ready();
					lcd[0].lcd_menue++;
					TIM7->CNT =0;
					HAL_TIM_Base_Start_IT(&htim7);
					LCD_2_menu();					
				}
		 }
		else if(lcd[0].lcd_menue==0)
		 {
			// timer start 
			 	
			 lcd[0].lcd_menue++;

			 LCD_2_menu();

		 }	 
	}	


	
	
	
	
void process_state_handler_2(uint8_t board_number)
	{
		// this command call with LCD_MASTER AND request DATA from charger board 
		// it worth to mention that this command only Send DATA to charger board
		// when DATA receive WE check that this is GET CHANNEL command and if so that writh that on LCD 

		             uint8_t    data[]        = {8,0,0,0,0,0,0,0};
		static       process_state_t    buffer;
		static 			 uint8_t    board_first_choose;	
					       
					       uint8_t    cnt;
					 
					 
					 
		if(first_send_2==0)
			{
				board_first_choose =	board_number;
				first_send_2=1;
				TIM3->CNT =0;
				HAL_TIM_Base_Start_IT(&htim3);
				flag.flag_BIT.CAN_FIFO0=0;
				flag.flag_BIT.state_process_2=1;
				flag.flag_BIT.LCD2_BUS_BUSY=1;
					#if 1
						flag.flag_BIT.uart_recive =1;
						usart_recive_rx[1] = 0x08;		
				#endif
				CAN_SEND_DATA(data,board_number,8);
			}
		else{
			if(flag.flag_BIT.CAN_FIFO0)
			{
				flag.flag_BIT.CAN_FIFO0=0;
						// get massage
//						if (HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
//						{
//							Error_Handler();
//						}	
						TIM4->CNT =0;

						 switch (first_send_2)
							 {
								case 1:
									for(cnt=0; cnt<8;cnt++)
									{buffer.main[cnt] =rx_buffer[cnt];}
									CAN_SEND_DATA(data,board_first_choose,8);
									first_send_2++;
								
									break;
								case 2:
									for(cnt=0; cnt<8;cnt++)
									{buffer.main[cnt+8] =rx_buffer[cnt];}
									CAN_SEND_DATA(data,board_first_choose,8);
									first_send_2++;
								break;									
								case 3:
									for(cnt=0; cnt<2;cnt++)
									{buffer.main[cnt+16] =rx_buffer[cnt];}
									first_send_2=0;
									flag.flag_BIT.state_process_2=0;
									uint8_t str[20];
									flag.flag_BIT.PC_BUS_BUSY=0;
									lcd_2_clear();
									lcd_2_gotoxy(0,0);	
									flag.flag_BIT.LCD2_BUS_BUSY=0;
									lcd_1_first_menu();
									HAL_TIM_Base_Stop_IT(&htim3);
									__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);
									switch(buffer.param.command)
										{	
										case 0:
											sprintf((char *)str,"There Is No Process");
											lcd_2_puts((char *)str);
											break;
										case 1:													
												sprintf((char *)str,"CC");
												lcd_2_puts((char *)str);
												lcd_2_gotoxy(0,1);
										sprintf((char *)str,"Time:%d:%d",(buffer.param.time/60),(buffer.param.time%60));
												lcd_2_puts((char *)str);	
												lcd_2_gotoxy(11,1);
												sprintf((char *)str,"I:%d",buffer.param.current);
												lcd_2_puts((char *)str);
												lcd_2_gotoxy(0,2);
												sprintf((char *)str,"MaxV:%d",buffer.param.maxV);
												lcd_2_puts((char *)str);
												lcd_2_gotoxy(11,2);
												sprintf((char *)str,"DV:%d",buffer.param.deltav);
												lcd_2_puts((char *)str);
												lcd_2_gotoxy(0,3);
												sprintf((char *)str,"Cend:%d",buffer.param.Cend);
												lcd_2_puts((char *)str);
										break;
										case 2:
												sprintf((char *)str,"CCCV");
												lcd_2_puts((char *)str);												
												lcd_2_gotoxy(0,1);
										sprintf((char *)str,"Time:%d:%d",(buffer.param.time/60),(buffer.param.time%60));
												lcd_2_puts((char *)str);											
												lcd_2_gotoxy(11,1);
												sprintf((char *)str,"I:%d",buffer.param.current);
												lcd_2_puts((char *)str);
												lcd_2_gotoxy(0,2);
												sprintf((char *)str,"MaxV:%d",buffer.param.maxV);
												lcd_2_puts((char *)str);
												lcd_2_gotoxy(11,2);
												sprintf((char *)str,"DV:%d",buffer.param.deltav);
												lcd_2_puts((char *)str);
												lcd_2_gotoxy(0,3);
												sprintf((char *)str,"IendV:%d",buffer.param.Iend);
												lcd_2_puts((char *)str);
												lcd_2_gotoxy(11,3);
												sprintf((char *)str,"Cend:%d",buffer.param.Cend);
												lcd_2_puts((char *)str);
										break;
										case 3:
												sprintf((char *)str,"Rest");
												lcd_2_puts((char *)str);												
												lcd_2_gotoxy(0,1);
										sprintf((char *)str,"Time:%d:%d",(buffer.param.time/60),(buffer.param.time%60));
												lcd_2_puts((char *)str);										
										break;
										case 4:
												sprintf((char *)str,"CD");
												lcd_2_puts((char *)str);												
												lcd_2_gotoxy(0,1);
												sprintf((char *)str,"Time:%d:%d",(buffer.param.time/60),(buffer.param.time%60));
												lcd_2_puts((char *)str);										
												lcd_2_gotoxy(11,1);
												sprintf((char *)str,"I:%d",buffer.param.current);
												lcd_2_puts((char *)str);
												lcd_2_gotoxy(0,2);
												sprintf((char *)str,"MinV:%d",buffer.param.minV);
												lcd_2_puts((char *)str);
												lcd_2_gotoxy(11,2);
												sprintf((char *)str,"DV:%d",buffer.param.deltav);
												lcd_2_puts((char *)str);
												lcd_2_gotoxy(11,3);
												sprintf((char *)str,"Cend:%d",buffer.param.Cend);
												lcd_2_puts((char *)str);
										break;
										case 5:
												sprintf((char *)str,"Cycle");
												lcd_2_puts((char *)str);												
												lcd_2_gotoxy(0,2);
												sprintf((char *)str,"Cycle:%d",buffer.param.cycle);
												lcd_2_puts((char *)str);										

										break;
										case 6:
												sprintf((char *)str,"END");
												lcd_2_puts((char *)str);												
										
										break;											
										default:
												sprintf((char *)str,"Error Frame");
												lcd_2_puts((char *)str);
										break;
										}
								break;									
							 }
						
						
						}
	
			
			}
	}	


void process_handler_handler_2(uint8_t board_num)
	{
				uint8_t      data[]        = {9,0,0,0,0,0,0,0};
				static  		 process_t  process_buffer;
				static 			 uint8_t    board_first_choose;
			  		 						 
								
										 uint8_t    cnt;
				
				
			if(first_send_2==0)
				{
					board_first_choose =	board_num;
						#if 1
						
						flag.flag_BIT.uart_recive =1;

						usart_recive_rx[1] = 0x09;		
						#endif
					first_send_2=1;
					step_count_2 =0;
					TIM3->CNT =0;
					HAL_TIM_Base_Start_IT(&htim3);
					flag.flag_BIT.CAN_FIFO0=0;
					flag.flag_BIT.process_2=1;
					flag.flag_BIT.LCD2_BUS_BUSY=1;
				
					CAN_SEND_DATA(data,board_num,8);
				}
		
				else
				{
					if(flag.flag_BIT.CAN_FIFO0)
					{
						flag.flag_BIT.CAN_FIFO0=0;
								// get massage
//								if (HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
//								{
//									Error_Handler();
//								}	
									TIM3->CNT =0;
									 switch (first_send_2)
										 {
											case 1:
												for(cnt=0; cnt<8;cnt++)
												{process_buffer.step[step_count_2].main[cnt] =rx_buffer[cnt];}
												first_send_2++;
												CAN_SEND_DATA(data,board_first_choose,8);
												break;
											case 2:
												for(cnt=0; cnt<8;cnt++)
												{process_buffer.step[step_count_2].main[cnt+8] =rx_buffer[cnt];}
												CAN_SEND_DATA(data,board_first_choose,8);
												first_send_2++;
											break;									
											case 3:
												for(cnt=0; cnt<2;cnt++)
												{process_buffer.step[step_count_2].main[cnt+16] =rx_buffer[cnt];}

												if(step_count_2==31)
													{																				
														lcd_2_clear();
														lcd_2_gotoxy(0,0);
														step_count_2=0;
														first_send_2=4;
														flag.flag_BIT.LCD2_BUS_BUSY=0;
														HAL_TIM_Base_Stop_IT(&htim3);
														__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);
													}
												else
													{
														first_send_2=1;
														step_count_2++;
														CAN_SEND_DATA(data,board_first_choose,8);
													
													}
												break;

											
										 }
							}
						if(first_send_2==4)
							{
										
									uint8_t str[20];
									flag.flag_BIT.LCD2_BUS_BUSY=0;
								flag.flag_BIT.PC_BUS_BUSY=0;
								lcd_1_first_menu();
								lcd_2_clear();
								
								if(process_buffer.step[(step_count_2-1)].param.command ==6){step_count_2 = (step_count_2 -1);}
									switch(process_buffer.step[step_count_2].param.command)
													{	
													case 0:
														sprintf((char *)str,"There Is No Process");
														lcd_2_puts((char *)str);
													break;															
													case 1:													
															sprintf((char *)str,"CC");
															lcd_2_puts((char *)str);
															lcd_2_gotoxy(0,1);
													sprintf((char *)str,"Time:%d:%d",(process_buffer.step[step_count_2].param.time/60),(process_buffer.step[step_count_2].param.time%60));
															lcd_2_puts((char *)str);	
															lcd_2_gotoxy(11,1);
															sprintf((char *)str,"I:%d",process_buffer.step[step_count_2].param.current);
															lcd_2_puts((char *)str);
															lcd_2_gotoxy(0,2);
															sprintf((char *)str,"MaxV:%d",process_buffer.step[step_count_2].param.maxV);
															lcd_2_puts((char *)str);
															lcd_2_gotoxy(11,2);
															sprintf((char *)str,"DV:%d",process_buffer.step[step_count_2].param.deltav);
															lcd_2_puts((char *)str);
															lcd_2_gotoxy(0,3);
															sprintf((char *)str,"Cend:%d",process_buffer.step[step_count_2].param.Cend);
															lcd_2_puts((char *)str);
													break;
													case 2:
															sprintf((char *)str,"CCCV");
															lcd_2_puts((char *)str);												
															lcd_2_gotoxy(0,1);
													sprintf((char *)str,"Time:%d:%d",(process_buffer.step[step_count_2].param.time/60),(process_buffer.step[step_count_2].param.time%60));
															lcd_2_puts((char *)str);											
															lcd_2_gotoxy(11,1);
															sprintf((char *)str,"I:%d",process_buffer.step[step_count_2].param.current);
															lcd_2_puts((char *)str);
															lcd_2_gotoxy(0,2);
															sprintf((char *)str,"MaxV:%d",process_buffer.step[step_count_2].param.maxV);
															lcd_2_puts((char *)str);
															lcd_2_gotoxy(11,2);
															sprintf((char *)str,"DV:%d",process_buffer.step[step_count_2].param.deltav);
															lcd_2_puts((char *)str);
															lcd_2_gotoxy(0,3);
															sprintf((char *)str,"IendV:%d",process_buffer.step[step_count_2].param.Iend);
															lcd_2_puts((char *)str);
															lcd_2_gotoxy(11,3);
															sprintf((char *)str,"Cend:%d",process_buffer.step[step_count_2].param.Cend);
															lcd_2_puts((char *)str);
													break;
													case 3:
															sprintf((char *)str,"Rest");
															lcd_2_puts((char *)str);												
															lcd_2_gotoxy(0,1);
													sprintf((char *)str,"Time:%d:%d",(process_buffer.step[step_count_2].param.time/60),(process_buffer.step[step_count_2].param.time%60));
															lcd_2_puts((char *)str);										
													break;
													case 4:
															sprintf((char *)str,"CD");
															lcd_2_puts((char *)str);												
															lcd_2_gotoxy(0,1);
													sprintf((char *)str,"Time:%d:%d",(process_buffer.step[step_count_2].param.time/60),(process_buffer.step[step_count_2].param.time%60));
															lcd_2_puts((char *)str);										
															lcd_2_gotoxy(11,1);
															sprintf((char *)str,"I:%d",process_buffer.step[step_count_2].param.current);
															lcd_2_puts((char *)str);
															lcd_2_gotoxy(0,2);
															sprintf((char *)str,"MinV:%d",process_buffer.step[step_count_2].param.minV);
															lcd_2_puts((char *)str);
															lcd_2_gotoxy(11,2);
															sprintf((char *)str,"DV:%d",process_buffer.step[step_count_2].param.deltav);
															lcd_2_puts((char *)str);
															lcd_2_gotoxy(11,3);
															sprintf((char *)str,"Cend:%d",process_buffer.step[step_count_2].param.Cend);
															lcd_2_puts((char *)str);
													break;
													case 5:
															sprintf((char *)str,"Cycle");
															lcd_2_puts((char *)str);												
															lcd_2_gotoxy(0,2);
															sprintf((char *)str,"Cycle:%d",process_buffer.step[step_count_2].param.cycle);
															lcd_2_puts((char *)str);										

													break;
													case 6:
															sprintf((char *)str,"END");
															lcd_2_puts((char *)str);												
													
													break;											
													default:
															sprintf((char *)str,"Error Frame");
															lcd_2_puts((char *)str);
													break;
													}

								
							}
					
							
						}
				}
				
				




uint8_t SET_board_num(uint8_t last_board_address)
{
 			uint8_t counter=0;
			 
		 counter = last_board_address +1;
			if(counter ==64)
							{
			
									HAL_TIM_Base_Stop_IT(&htim2);
									HAL_TIM_Base_Stop_IT(&htim5);
									__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
								__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
									board_channel_get_data =1;
									flag.flag_BIT.command_or_data = 0 ;
									first_send=0;
									flag.flag_BIT.uart_recive=0;
									flag.flag_BIT.LCD2_BUS_BUSY=0;
									flag.flag_BIT.LCD1_BUS_BUSY=0;
									usart_recive_rx[0] = 0x00;
									usart_recive_rx[1] = 0x00;		
									
									flag.flag_BIT.command_or_data = 0 ;
									finish_reacive =0;
									flag.flag_BIT.ASK_DATA=0;
									if(flag.flag_BIT.key_enter)
										{
										lcd_1_first_menu();
										lcd_2_first_menu();
										flag.flag_BIT.key_enter=0;
										}
									return counter;
							}
							
			if(rx_normal_board[counter] ==0x11)
					{
					//first_send = 0;
					return counter;
				
					}
			else
				{
					uint32_t timeout =0;
					uint8_t str[2]= {NOT_FOUND_CHARGER , 0};
					while(timeout < 100)
					{
						timeout++;
					str[0] = 	usart_recive_rx[1];
					str[1] =  counter;
					HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
					HAL_UART_Transmit(&huart1,str,2,100);
					HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);	
						
					str[0] = 	NOT_FOUND_CHARGER;
					str[1]=		counter ;
					HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
					HAL_UART_Transmit(&huart1,str,2,100);
					HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
					counter ++;				
						if(counter ==64)
							{
			
									HAL_TIM_Base_Stop_IT(&htim2);
									HAL_TIM_Base_Stop_IT(&htim5);
									__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
									__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
									board_channel_get_data =1;
									flag.flag_BIT.command_or_data = 0 ;
									//first_send=0;
									flag.flag_BIT.uart_recive=0;
									flag.flag_BIT.LCD2_BUS_BUSY=0;
									flag.flag_BIT.LCD1_BUS_BUSY=0;
									usart_recive_rx[0] = 0x00;
									usart_recive_rx[1] = 0x00;		
									
									flag.flag_BIT.command_or_data = 0 ;
									finish_reacive =0;
									flag.flag_BIT.ASK_DATA=0;
									if(flag.flag_BIT.key_enter)
										{
										lcd_1_first_menu();
										lcd_2_first_menu();
										flag.flag_BIT.key_enter=0;
										}
									
									break;
							}
						else
							{
								if(rx_normal_board[counter] ==0x11)
									{			
										//first_send= 0;
										break;
									}
							}
						}
				return counter;
				}
}



void get_normal_board(void)
{

 if(first_send)
		{
			usart_recive_rx[0] = 0x00;
			usart_recive_rx[1] = 0x00;	
			first_send =0;
			flag.flag_BIT.command_or_data =0;
			
			uint8_t rady[2] = {0x0F , 0x00};
			
			flag.flag_BIT.uart_recive=0;
			flag.flag_BIT.LCD2_BUS_BUSY=0;
			flag.flag_BIT.LCD1_BUS_BUSY=0;
			
			finish_reacive =0;
			flag.flag_BIT.ASK_DATA=0;
			HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			HAL_UART_Transmit(&huart1,rady,2, 100);
			HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
		}




}


void send_handshake(uint8_t command)
{
	uint8_t data[2]= {0x00,0x00};
	
	switch(command)
	{
		case 1:
				data[0] =1;
				flag.flag_BIT.command_or_data = 1 ;
				// send to pc for validation of send
				HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
				HAL_UART_Transmit(&huart1,data,2, 100);
				HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
		break;
		case 5:	
		{
				data[0]= 5;
				HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
				HAL_UART_Transmit(&huart1 , data ,2,100);
				HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
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
																			if(charger_board_num<64)
																				{
																				uint8_t data1[2] = {5,charger_board_num};
																			
																				HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
																				HAL_UART_Transmit(&huart1 , data1 ,2,100);
																				HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
																				get_data_counter=0;
																				TIM5->CNT=0;
																				HAL_TIM_Base_Start_IT(&htim5);
																				//first_send =1;
																				CAN_SEND_DATA(data,charger_board_num+64,8);
																				return;
																				}	
																			else
																			{
																				//counter = 0;	
																				
																				HAL_TIM_Base_Stop_IT(&htim5);
																				HAL_TIM_Base_Stop_IT(&htim2);
																				__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
																				__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
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
		}
			break;
		case 7:
							data[0] =7;
				flag.flag_BIT.command_or_data = 1 ;
				// send to pc for validation of send
				HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
				HAL_UART_Transmit(&huart1,data,2, 100);
				HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
			break;
		case 13:
																								
			data[0] = 13;
			HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			HAL_UART_Transmit(&huart1 , data ,2,100);
			HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
break;
		case 15:
			data[0] = 15;
			flag.flag_BIT.command_or_data = 1 ;
			finish_reacive =0;
			flag.flag_BIT.ASK_DATA=1;
			TIM2->CNT =0;
			HAL_TIM_Base_Start_IT(&htim2);
			HAL_UART_Transmit(&huart1,start_buffer,PARITY_SIZE,100);
			HAL_UART_Transmit(&huart1,data,2, 100);
			HAL_UART_Transmit(&huart1,end_buffer,PARITY_SIZE,100);
			break;
		default :
			break;
		
	
	}



}
void lcd_1_first_menu(void)
{
	flag.flag_BIT.CELL1_RESTART=0;
	flag.flag_BIT.lcd1_function=0;
	lcd[1].lcd_menue=1;
	lcd_1_init();
	lcd_1_clear();
	lcd_1_gotoxy(0,0);
	lcd_1_puts("Choose");
	lcd_1_gotoxy(0,1);
	lcd_1_puts("1_Cell Parameter");
	lcd_1_gotoxy(0,2);
	lcd_1_puts("2_Process State");
	lcd_1_gotoxy(0,3);
	lcd_1_puts("3_Process");
	flag.flag_BIT.get_channel_1=0;
	flag.flag_BIT.state_process_1=0;
	flag.flag_BIT.process_1=0;
	first_send_1=0;
	HAL_TIM_Base_Stop_IT(&htim6);
	__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);

}
void lcd_2_first_menu(void)
{
	lcd[0].lcd_menue=1;
	flag.flag_BIT.CELL2_RESTART=0;
	flag.flag_BIT.lcd2_function=0;
	lcd_2_init();
	lcd_2_clear();
	lcd_2_gotoxy(0,0);
	lcd_2_puts("Choose");
	lcd_2_gotoxy(0,1);
	lcd_2_puts("1_Cell Parameter");
	lcd_2_gotoxy(0,2);
	lcd_2_puts("2_Process State");
	lcd_2_gotoxy(0,3);
	lcd_2_puts("3_Process");
	flag.flag_BIT.get_channel_2=0;
	flag.flag_BIT.state_process_2=0;
	flag.flag_BIT.process_2=0;
	first_send_2=0;
	HAL_TIM_Base_Stop_IT(&htim7);
	__HAL_TIM_CLEAR_IT(&htim7,TIM_IT_UPDATE);
} 
void lcd_1_not_ready(void)
{
									lcd[1].lcd_menue=1;
									lcd_1_init();
									lcd_1_clear();
									
									lcd_1_gotoxy(4,1);
									lcd_1_puts("Please Wait");

									TIM6->CNT=0;
									HAL_TIM_Base_Start_IT(&htim6);
}
void lcd_2_not_ready(void)
{
									lcd[0].lcd_menue=1;
									lcd_2_init();
									lcd_2_clear();
											
									lcd_2_gotoxy(4,1);
									lcd_2_puts("Please Wait ");

									TIM6->CNT=0;
									HAL_TIM_Base_Start_IT(&htim7);
}
 void delay_ms(uint16_t delay_ms)
 {
 for(uint16_t cnt=0; cnt<delay_ms;cnt++)
	 {
		for(uint16_t inner_counter=0 ; inner_counter<65000; inner_counter++);
		for(uint16_t inner_counter=0 ; inner_counter<7000; inner_counter++);
	 
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

