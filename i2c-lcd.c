
/** Put this in the src folder **/

#include "i2c-lcd.h"
#include "custom_chars.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
	
}

void lcd_init (void)
{
	lcd_send_cmd (0x02);
	lcd_send_cmd (0x28);
	lcd_send_cmd (0x0c);
	lcd_send_cmd (0x80);
	lcd_send_cmd(HD44780_CMD_SET_CG_RAM_ADD);


		uint8_t __i;
		for(__i=0;__i<sizeof(__cgram);__i++)
			lcd_send_data(__cgram[__i]);//commented on 02 august

}

void lcd_send_string (char *msg)
{
	if(*msg=='\0'){LCDGotoXY(2,0);
			lcd_send_data(32);}
	else{
	while(*msg!='\0')
	 {
	 	//Custom Char Support
		if(*msg=='%')
		{
			//lcd_send_cmd(HD44780_CMD_SET_CG_RAM_ADD);
			msg++;
			int8_t cc=*msg-'0';

			if(cc>=0 && cc<=10)
			{
				lcd_send_data (cc)	;//LCDData(cc);
			}
			else
			{
				lcd_send_data ('%');
				lcd_send_data (*msg);/*LCDData('%');
				LCDData(*msg);*/
			}
		}
		else
		{
			lcd_send_data(*msg);//LCDData(*msg);
		}
		msg++;
	 }

	}
	//while (*str) lcd_send_data (*str++);

}
void gotoXY(int x)
{
	int a=1;
	 while(a<x)
	 {
		 lcd_send_cmd(0x06);
		 lcd_send_string(" ");
		 a++;
	 }
 }	 
 int intToStr(int x, char str[], int d)
  {
    int i = 0;
    while (x)
    {
      str[i++] = (x % 10) + '0';
      x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
      str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
  }
	 void reverse(char *str, int len)
  {
    int i = 0, j = len - 1, temp;
    while (i < j)
    {
      temp = str[i];
      str[i] = str[j];
      str[j] = temp;
      i++; j--;
    }
  }
	void LCDGotoXY(uint8_t x,uint8_t y)
{
 	if(x>=16) return;

	

	switch(y)
	{
		case 0:
			break;
		case 1:
			x|=0x40;//0b01000000;
			break;
		/*case 2:
			x+=0x10;
			break;
		case 3:
			x+=0x50;
			break;*/
	}

	

	x|=0x80;//0b10000000;
  	//LCDCmd(x);
	lcd_send_cmd (x);
}


void lcd_string_draw(char *msg, uint8_t x,uint8_t y)
{
	LCDGotoXY(x,y);
	lcd_send_string(msg);
}

void lcd_clear(void)
{
	LCDGotoXY(0,0);
  lcd_send_string("                ");
	LCDGotoXY(0,1);
	lcd_send_string("                ");
}

