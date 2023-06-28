


#ifndef DEBUG
	#define DEBUG
	
			#define STR(x)  #x;
			
			#define debug_print(str)     \
				do									     \
				{									       \
					HAL_UART_Transmit(&huart3,STR(str),strlen(str),1000);\
				}while(0)

#define DEBUG_VALUE(value)     \
do                         \
{                          \
uint8_t str[10];  \
sprintf((char *)str ,"%u",value); \
HAL_UART_Transmit(&huart1,str,strlen((char *)str),1000); \
}while(0)				
					//define semi function for GPIO OUTPUT
		#define LED_ON 		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET)
		#define LED_OFF		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET)

		#define LCD1_light_OFF 	HAL_GPIO_WritePin(LCD1_back_light_GPIO_Port,LCD1_back_light_Pin,GPIO_PIN_RESET)
		#define LCD1_light_ON		HAL_GPIO_WritePin(LCD1_back_light_GPIO_Port,LCD1_back_light_Pin,GPIO_PIN_SET)		
								
		#define LCD2_light_OFF 	HAL_GPIO_WritePin(LCD2_back_light_GPIO_Port,LCD2_back_light_Pin,GPIO_PIN_RESET)
		#define LCD2_light_ON		HAL_GPIO_WritePin(LCD2_back_light_GPIO_Port,LCD2_back_light_Pin,GPIO_PIN_SET)			
#endif
