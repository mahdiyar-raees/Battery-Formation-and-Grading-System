#include "stm32f1xx_hal.h"
#include  <stdbool.h>
//(2): Constants definition
#define T  10000  

#define D0_PIN_NUMBER  0

#define s_DATA_PORT_D4  GPIOB
#define s_DATA_PORT_D5  GPIOB
#define s_DATA_PORT_D6  GPIOB
#define s_DATA_PORT_D7  GPIOB
#define s_D4_PIN_NUMBER  GPIO_PIN_1
#define s_D5_PIN_NUMBER  GPIO_PIN_2
#define s_D6_PIN_NUMBER  GPIO_PIN_10
#define s_D7_PIN_NUMBER  GPIO_PIN_11


#define s_CTRL_PORT_RS  GPIOA
#define s_CTRL_PORT_E   GPIOA
#define s_CTRL_PORT_RW  GPIOA
#define s_RS_PIN_NUMBER  GPIO_PIN_5
#define s_E_PIN_NUMBER   GPIO_PIN_7
#define s_RW_PIN_NUMBER  GPIO_PIN_6


void lcd_2_init(void);
void lcd_2_clear(void);
void lcd_2_gotoxy(unsigned char x, unsigned char y);
void lcd_2_puts(char *str);
void DelayUS(uint32_t us);

