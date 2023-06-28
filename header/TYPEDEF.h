

#ifndef TYPEDEF
	#define TYPEDEF
		#include "stdint.h"
		#define PARITY_SIZE 				3
		#define first_start   		 54 
 		#define seccond_start			 56
		#define third_start				 55
		#define first_end    			 83 
 		#define seccond_end				 77
		#define third_end					 67
		
		
		#define test 				0
	
	
		#define NOT_FOUND_CHARGER 0xFF
		#define NOT_VALID_COMMAND 0xFE
	
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
			uint32_t flag_byte;
				struct 
				{

					uint8_t    key_enter:1;
					uint8_t    CAN_FIFO0:1;
					uint8_t    CAN_FIFO1:1;
					uint8_t    check_charger_board:1;
					uint8_t    uart_recive:1;
					uint8_t    stop_first:1;
					uint8_t    get_DATA_command:1;
					uint8_t    get_channel_1:1;
					uint8_t    process_1:1;
					uint8_t    state_process_1:1;
					uint8_t    get_channel_2:1;
					uint8_t    process_2:1;
					uint8_t    state_process_2:1;
					uint8_t    command_or_data:1;
					uint8_t		 LCD1_BUS_BUSY:1;
					uint8_t		 LCD2_BUS_BUSY:1;
					uint8_t		 PC_BUS_BUSY:1;
					uint8_t		 process_data_ready:1;
					uint8_t		 finish_command:1;
					uint8_t		 ASK_DATA:1;
					uint8_t		grading_mode:1;
					uint8_t		 lcd1_function:1;
					uint8_t		 lcd2_function:1;		
					uint8_t		 CELL1_RESTART:1;	
					uint8_t		 CELL2_RESTART:1;
					
				}flag_BIT;

			}MCU_FLAG_t;

			
	/// define type for MCU flag		
	typedef union
	{
			uint32_t flag_byte;
				struct 
				{
					uint8_t    timer1:1;
					uint8_t    timer2:1;
					uint8_t    timer3:1;
					uint8_t    timer4:1;
					uint8_t    timer5:1;
					uint8_t    timer6:1;
					uint8_t    timer7:1;
					uint8_t    timer8:1;
				
				}flag_BIT;

			}timer_flag_t;
			
		
			
	// structure for LCD Vartiable
	typedef struct
		{
			uint8_t 			lcd_menue;
			uint8_t 			key;
			uint8_t				number_position;
			uint8_t      	first_menu_choose;
			uint8_t   		board_number[4];
			uint8_t 			counter;
		}LCD_t;
			

		
typedef union
	{
			uint64_t data;
			uint8_t rx_buffer[8];	
			struct 
				{
					uint16_t voltage; 
					uint16_t current;
					uint16_t soc;
					uint16_t temp;

				}param;

			}get_data;
	
typedef union
	{

			uint8_t    main[24];
		struct 
				{
					uint16_t command; 
					uint16_t time;
					uint16_t current;
					uint16_t maxV;
					uint16_t minV; 
					uint16_t deltav;
					uint16_t Iend;
					uint16_t Cend;
					uint16_t cycle;
				}param;

			}process_state_t;		
		
typedef union
	{

			process_state_t   step[32];

			}process_t;		

		typedef enum 
			{
				SEND,
				GET_DATA,
				SEND_PROCESS,
				decharge,
				emergancy,
}SYSTEM_MODE_t;	
#endif
