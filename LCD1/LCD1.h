#include "stm32f1xx_hal.h"
#include  <stdbool.h>
//(2): Constants definition
#define T  10000  

#define D0_PIN_NUMBER  0

#define DATA_PORT_D4  GPIOB
#define DATA_PORT_D5  GPIOB
#define DATA_PORT_D6  GPIOB
#define DATA_PORT_D7  GPIOB
#define D4_PIN_NUMBER  GPIO_PIN_4
#define D5_PIN_NUMBER  GPIO_PIN_5
#define D6_PIN_NUMBER  GPIO_PIN_6
#define D7_PIN_NUMBER  GPIO_PIN_7


#define CTRL_PORT_RS  GPIOD
#define CTRL_PORT_E   GPIOA
#define CTRL_PORT_RW  GPIOC
#define RS_PIN_NUMBER  GPIO_PIN_2
#define E_PIN_NUMBER   GPIO_PIN_15
#define RW_PIN_NUMBER  GPIO_PIN_12


void lcd_1_init(void);
void lcd_1_clear(void);
void lcd_1_gotoxy(unsigned char x, unsigned char y);
void lcd_1_puts(char *str);
void DelayUS(uint32_t us);

