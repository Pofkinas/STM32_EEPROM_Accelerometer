//#include "stm32l4xx_hal.h"

#include "stm32l0xx_hal.h"

#define HD44780_CMD_SET_CG_RAM_ADD 0x40
//#define SLAVE_ADDRESS_LCD (0x3F<<1) // change this according to your setup
#define SLAVE_ADDRESS_LCD (0x27<<1) // change this according to your setup

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void gotoXY(int x);

 int intToStr(int value, char str[], int d);
 void reverse(char *str, int len);
 void LCDGotoXY(uint8_t x,uint8_t y);
// enum selectcount{none1=0,second=1,minutes=2,hours=3,none=4};//to be not used in other projetcs
void lcd_string_draw(char *msg, uint8_t x,uint8_t y);

void lcd_clear(void);
