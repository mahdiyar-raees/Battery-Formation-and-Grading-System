#include "stm32f1xx_hal.h"
#include "LCD1.h"
#include  <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
void send_1_command(unsigned char data)
{
 // char L,H;
  char L1=0,L2=0,L3=0,L4=0,H1=0,H2=0,H3=0,H4=0;
  H1 = data  & 0x10;
  H2 = data  & 0x20;
  H3 = data  & 0x40;
  H4 = data  & 0x80;
  L1 = data  & 0x01;
  L2 = data  & 0x02;
  L3 = data  & 0x04;
  L4 = data  & 0x08;
  
  //HAL_Delay(T);
for(uint32_t cnt=0 ; cnt<T; cnt++);
  HAL_GPIO_WritePin(CTRL_PORT_RS,RS_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CTRL_PORT_RW,RW_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CTRL_PORT_E,E_PIN_NUMBER,GPIO_PIN_SET);
  
  HAL_GPIO_WritePin(DATA_PORT_D4,D4_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT_D5,D5_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT_D6,D6_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT_D7,D7_PIN_NUMBER,GPIO_PIN_RESET);
  if(H1 == 0x10)
     HAL_GPIO_WritePin(DATA_PORT_D4,D4_PIN_NUMBER,GPIO_PIN_SET);
  if(H2 == 0x20)
     HAL_GPIO_WritePin(DATA_PORT_D5,D5_PIN_NUMBER,GPIO_PIN_SET);
  if(H3 == 0x40)
     HAL_GPIO_WritePin(DATA_PORT_D6,D6_PIN_NUMBER,GPIO_PIN_SET);
  if(H4 == 0x80)
     HAL_GPIO_WritePin(DATA_PORT_D7,D7_PIN_NUMBER,GPIO_PIN_SET);


  
  
  HAL_GPIO_WritePin(CTRL_PORT_E,E_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CTRL_PORT_E,E_PIN_NUMBER,GPIO_PIN_SET);
  
  HAL_GPIO_WritePin(DATA_PORT_D4,D4_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT_D5,D5_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT_D6,D6_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT_D7,D7_PIN_NUMBER,GPIO_PIN_RESET);
  if(L1 == 0x01)
     HAL_GPIO_WritePin(DATA_PORT_D4,D4_PIN_NUMBER,GPIO_PIN_SET);
  if(L2 == 0x02)
     HAL_GPIO_WritePin(DATA_PORT_D5,D5_PIN_NUMBER,GPIO_PIN_SET);
  if(L3 == 0x04)
     HAL_GPIO_WritePin(DATA_PORT_D6,D6_PIN_NUMBER,GPIO_PIN_SET);
  if(L4 == 0x08)
     HAL_GPIO_WritePin(DATA_PORT_D7,D7_PIN_NUMBER,GPIO_PIN_SET);
  
  HAL_GPIO_WritePin(CTRL_PORT_E,E_PIN_NUMBER,GPIO_PIN_RESET);

}

void lcd_1_putchar(unsigned char data)
{
  char L1=0,L2=0,L3=0,L4=0,H1=0,H2=0,H3=0,H4=0;
  H1 = data  & 0x10;
  H2 = data  & 0x20;
  H3 = data  & 0x40;
  H4 = data  & 0x80;
  L1 = data  & 0x01;
  L2 = data  & 0x02;
  L3 = data  & 0x04;
  L4 = data  & 0x08;
  //HAL_Delay(T);
for(uint32_t cnt=0 ; cnt<T; cnt++);
  HAL_GPIO_WritePin(CTRL_PORT_RS,RS_PIN_NUMBER,GPIO_PIN_SET);
  HAL_GPIO_WritePin(CTRL_PORT_RW,RW_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CTRL_PORT_E,E_PIN_NUMBER,GPIO_PIN_SET);
  
  HAL_GPIO_WritePin(DATA_PORT_D4,D4_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT_D5,D5_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT_D6,D6_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT_D7,D7_PIN_NUMBER,GPIO_PIN_RESET);
  if(H1 == 0x10)
     HAL_GPIO_WritePin(DATA_PORT_D4,D4_PIN_NUMBER,GPIO_PIN_SET);
  if(H2 == 0x20)
     HAL_GPIO_WritePin(DATA_PORT_D5,D5_PIN_NUMBER,GPIO_PIN_SET);
  if(H3 == 0x40)
     HAL_GPIO_WritePin(DATA_PORT_D6,D6_PIN_NUMBER,GPIO_PIN_SET);
  if(H4 == 0x80)
     HAL_GPIO_WritePin(DATA_PORT_D7,D7_PIN_NUMBER,GPIO_PIN_SET);

  HAL_GPIO_WritePin(CTRL_PORT_E,E_PIN_NUMBER,GPIO_PIN_RESET);
  
  HAL_GPIO_WritePin(CTRL_PORT_E,E_PIN_NUMBER,GPIO_PIN_SET);
  
  HAL_GPIO_WritePin(DATA_PORT_D4,D4_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT_D5,D5_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT_D6,D6_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT_D7,D7_PIN_NUMBER,GPIO_PIN_RESET);
  if(L1 == 0x01)
     HAL_GPIO_WritePin(DATA_PORT_D4,D4_PIN_NUMBER,GPIO_PIN_SET);
  if(L2 == 0x02)
     HAL_GPIO_WritePin(DATA_PORT_D5,D5_PIN_NUMBER,GPIO_PIN_SET);
  if(L3 == 0x04)
     HAL_GPIO_WritePin(DATA_PORT_D6,D6_PIN_NUMBER,GPIO_PIN_SET);
  if(L4 == 0x08)
     HAL_GPIO_WritePin(DATA_PORT_D7,D7_PIN_NUMBER,GPIO_PIN_SET);
  
  HAL_GPIO_WritePin(CTRL_PORT_E,E_PIN_NUMBER,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CTRL_PORT_E,E_PIN_NUMBER,GPIO_PIN_RESET);

}

void lcd_1_init(void)
{
 // HAL_Delay(T);
for(uint32_t cnt=0 ; cnt<T; cnt++);
	send_1_command(0x03);
  send_1_command(0x28);  //0x20 for 1 line 4 pin, 0x38 for 2 line 8pin, 0x30 for 1 line 8pin
  send_1_command(0x06);
  send_1_command(0x0c);
}

void lcd_1_puts(char *str)
{
  //HAL_Delay(T);
for(uint32_t cnt=0 ; cnt<T; cnt++);
  while(*str != 0)
  {
    lcd_1_putchar(*str);
    str++;
  }
}

void lcd_1_gotoxy(unsigned char x, unsigned char y)
{
 // HAL_Delay(T); 
for(uint32_t cnt=0 ; cnt<T; cnt++);
    switch(y){
    case 0:
      send_1_command( 0x80 + x );
    break;
    case 1:
      send_1_command( 0xC0 + x );
      break;
    case 2:
      send_1_command( 0x94 + x );
      break;
    case 3:
      send_1_command( 0xD4 + x );
  }
}

void lcd_1_clear(void)
{
for(uint32_t cnt=0 ; cnt<T; cnt++);
	// HAL_Delay(T);
	
  send_1_command(0x01);
  send_1_command(0x02);
}
