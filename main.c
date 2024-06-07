/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "i2c.h"
#include "i2c-lcd.h"
#include "usart.h"
#include "gpio.h"
#include "MPU6050.h"
#include "EEPROM.h"

#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void accel_read(void);

void delete_eeprom(void);
void print_eeprom(void);

void update_page_info(char logic);
void write_to_eeprom(float xdata, float ydata, long time);
void read_from_eeprom(void);

void timer(char logic, long curr_time, u_int16_t* hrs, u_int16_t* min, u_int16_t* sec);

void total(float xdata, float ydata);
void day(char logic);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum {DisplayState_Init, Main_sceen, Display_clear, Eeprom_clear, Display_total_max, Display_total_min, Display_day_max, Display_day_min} TDisplayState;

// redirect printf to uart
int _write(int file, uint8_t* p, int len)
{
	if(HAL_UART_Transmit(&huart2, p, len, len) == HAL_OK )
	{
		return len;
	}
	return 0;
}
TDisplayState DisplayState = DisplayState_Init;

volatile unsigned long counter;

// string used to display data on lcd
char my_str[17];

// accelerometer data to display on lcd
char acc_x[17];
char acc_y[17];

// data read from eeeprom
float acc_x_read, acc_y_read, eeprom_x_read, eeprom_y_read;

//uint16_t page;
uint8_t max_page = 0;

/*
Main page structure:
Page size - 64 bytes; 4 bytes per entry

  0 - current page number     12 - total register   32 - day register
  4 - current place in page   16 - total max x      36 - day max x
  8 - write time              20 - total max y      40 - day max y
                              24 - total min x      44 - day min x
                              28 - total min y      48 - day min y
*/

uint32_t curr_page_num = 1;
uint32_t curr_place = 0;
uint32_t write_time = 0;
uint32_t log_timer = 0;

uint32_t total_reg = 0;
float total_max_x = 0;
float total_max_y = 0;
float total_min_x = 0;
float total_min_y = 0;

uint32_t day_reg = 0;
float day_max_x = 0;
float day_max_y = 0;
float day_min_x = 0;
float day_min_y = 0;

/*
Define eeprom first page number, sample rate, reading in a day
To read every 30 seconds for 2 days, set sample rate to (30*1000) = 30000 and day reading to (2*60*24) = 2880
*/ 
#define first_page 0

#define sample_rate 1000

#define day_readings 8
#define page_ofset (day_readings / 8)
#define pos_ofset ((day_readings - page_ofset * 8) * 8)

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  unsigned long ulNextTickDisplay = 0;

  unsigned long read_timer = 0;

  // elapsed time debounce variables
  unsigned long et[5] = {0, 0, 0, 0, 0};

  // timer
  u_int16_t h = 0, min = 0, secs = 0;

	// temporary buffer for display line 

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
	lcd_clear();
	MPU6050_Initialization();

  // used for manual reset of eeprom
  /*for(int i = 0; i <= 512; i++){
    EEPROM_PageErase(i);
    printf("delete page: %d\n", i);
  }
  printf("delete eeprom finish\n");*/

  update_page_info(1);

  // set timer from eeprom
  timer(1, write_time, &h, &min, &secs);

  log_timer = write_time;

  if(log_timer != 0){
    log_timer += sample_rate;
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
   {
    // read accel value every defined secs and write to eeprom
    if(counter >= read_timer){
      accel_read();
      write_to_eeprom(acc_x_read, acc_y_read, log_timer);
      timer(0, write_time, &h, &min, &secs);

      log_timer += sample_rate;
      read_timer += sample_rate; // read inteval
    }

    // ff display interval has elapsed, then update display state
    if(counter >= ulNextTickDisplay)
    {            
      switch(DisplayState)
      {
        // initial state, show initial strings for 3 seconds
        case DisplayState_Init:
          lcd_string_draw("Semestrinis",2,0);
					lcd_string_draw("projektas",3,1);									  
													
					ulNextTickDisplay += 2000;
          DisplayState = Display_clear;

        break;

        // clear display and go to main screen
        case Display_clear:
          lcd_clear();

          ulNextTickDisplay +=50;
          DisplayState = Main_sceen;
        
        break;

        // main screen, show accelerometer data and time
        case Main_sceen:

          lcd_string_draw(acc_x,0,0);
          lcd_string_draw(acc_y,8,0);
          if(acc_x_read > 0){
            lcd_string_draw(" ",7,0);
          }
          if(acc_y_read > 0){
            lcd_string_draw(" ",15,0);
          }

          sprintf(my_str, "%02hu:%02hu:%02hu", h, min, secs);
          lcd_string_draw(my_str, 4, 1);

          ulNextTickDisplay += 500;

        break;

        // display when eeprom is cleared
        case Eeprom_clear:
          lcd_clear();
          lcd_string_draw("EEPROM clear", 0, 1);
          timer(1, write_time, &h, &min, &secs);

          ulNextTickDisplay = counter + 1000;
          DisplayState = Display_clear;

        break;

        // display total max values
        case Display_total_max:
          lcd_clear();
          sprintf(my_str, "t max x:%f", EEPROM_Read_NUM(first_page, 16));
          lcd_string_draw(my_str, 0, 0);
          sprintf(my_str, "t max y:%f", EEPROM_Read_NUM(first_page, 20));
          lcd_string_draw(my_str, 0, 1);
          printf("total max x: %f, total max y: %f\n", EEPROM_Read_NUM(first_page, 16), EEPROM_Read_NUM(first_page, 20));

          DisplayState = Display_clear;
          ulNextTickDisplay +=3000;

        break;

        // display total min values
        case Display_total_min:
          lcd_clear();
          sprintf(my_str, "t min x:%f", EEPROM_Read_NUM(first_page, 24));
          lcd_string_draw(my_str, 0, 0);
          sprintf(my_str, "t min y:%f", EEPROM_Read_NUM(first_page, 28));
          lcd_string_draw(my_str, 0, 1);
          printf("total min x: %f, total min y: %f\n", EEPROM_Read_NUM(first_page, 24), EEPROM_Read_NUM(first_page, 28));

          DisplayState = Display_clear;
          ulNextTickDisplay +=3000;

        break;

        // display day max values
        case Display_day_max:
          lcd_clear();
          sprintf(my_str, "d max x:%f", EEPROM_Read_NUM(first_page, 36));
          lcd_string_draw(my_str, 0, 0);
          sprintf(my_str, "d max y:%f", EEPROM_Read_NUM(first_page, 40));
          lcd_string_draw(my_str, 0, 1);
          printf("day max x: %f, day max y: %f\n", EEPROM_Read_NUM(first_page, 36), EEPROM_Read_NUM(first_page, 40));

          DisplayState = Display_clear;
          ulNextTickDisplay +=3000;

        break;

        // display day min values
        case Display_day_min:
          lcd_clear();
          sprintf(my_str, "d min x:%f", EEPROM_Read_NUM(first_page, 44));
          lcd_string_draw(my_str, 0, 0);
          sprintf(my_str, "d min y:%f", EEPROM_Read_NUM(first_page, 48));
          lcd_string_draw(my_str, 0, 1);
          printf("day min x: %f, day min y: %f\n", EEPROM_Read_NUM(first_page, 44), EEPROM_Read_NUM(first_page, 48));

          DisplayState = Display_clear;
          ulNextTickDisplay +=3000;

        break;
        // default state.  Should never get here, but if so just go back to starting state.
        default:
          DisplayState = DisplayState_Init;
        break;
      } // end switch
    } // end if

    if(curr_page_num == PAGE_NUM){
      max_page = 1;
    }

    //----------------------------------- buttons -----------------------------------------
    //----------------------------------- 7a ----------------------------------------------
    if (HAL_GPIO_ReadPin(S0_GPIO_Port, S0_Pin) == GPIO_PIN_RESET){
      if (counter - et[0] > 15){
        day(1);
        DisplayState = Display_day_max;
      }

      et[0] = counter; 
    }
    //----------------------------------- 7b -----------------------------------------------
    if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == GPIO_PIN_RESET){
      if (counter - et[1] > 15){
        DisplayState = Display_total_max;
      }

      et[1] = counter; 
    }
    //----------------------------------- 7c ------------------------------------------------
    if (HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == GPIO_PIN_RESET){
      if (counter - et[2] > 15){
        day(1);
        DisplayState = Display_day_min;
      }
      et[2] = counter; 
    }
    //----------------------------------- 7d ------------------------------------------------
    if (HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == GPIO_PIN_RESET){
      if (counter - et[3] > 15){
        DisplayState = Display_total_min;
      }

      et[3] = counter; 
    }
    //----------------------------------- 7e ------------------------------------------------
    if (HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == GPIO_PIN_RESET){
      if (counter - et[4] > 15){
        delete_eeprom();
      }

      et[4] = counter; 
    }
    //----------------------------------- Print eeprom --------------------------------------
    if((HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == GPIO_PIN_RESET) && HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == GPIO_PIN_RESET){
      print_eeprom();
    }

  }// end while(1)
  /* USER CODE END 3 */
}

//------------------------------------------------------------------------------
// Function to read accelerometer data
void accel_read(void){

  if(MPU6050_DataReady() == 1){
    MPU6050_ProcessData(&MPU6050); // when data is ready, process it and read it
    acc_x_read = MPU6050.acc_x;
    acc_y_read = MPU6050.acc_y;
    sprintf(acc_x, "x:%3.3f", acc_x_read);
    sprintf(acc_y, "y:%3.3f", acc_y_read);
  }
  else{
    accel_read();
  }
}

//------------------------------------------------------------------------------
// Function to delete eeprom
void delete_eeprom(void){
  printf("delete eeprom start\n");
  lcd_string_draw("EEPROM erase", 0, 1);

  int del;

  // check if current page is the last page and set delete page number 
  if(curr_page_num < PAGE_NUM){
    del = curr_page_num + 1;
  }
  else{
    del = PAGE_NUM;
  }

  // delete pages
  for(int i = 0; i < del; i++){
    EEPROM_PageErase(i);
  }
  printf("delete eeprom finish\n");

  // reset all variables
  curr_page_num = 1;
  curr_place = 0;
  write_time = 0;
  log_timer = 0;

  max_page = 0;

  total_reg = 0;
  total_max_x = 0;
  total_max_y = 0;
  total_min_x = 0;
  total_min_y = 0;

  day_reg = 0;
  day_max_x = 0;
  day_max_y = 0;
  day_min_x = 0;
  day_min_y = 0;
  //-----------------------------------------------
  // reset cursor to default possition
  EEPROM_Write_NUM(first_page, 0, curr_page_num);
  EEPROM_Write_NUM(first_page, 4, curr_place);
  EEPROM_Write_NUM(first_page, 8, write_time);
  // reset total x and y min max values in eeprom
  EEPROM_Write_NUM(first_page, 12, total_reg);
  EEPROM_Write_NUM(first_page, 16, total_max_x);
  EEPROM_Write_NUM(first_page, 20, total_max_y);
  EEPROM_Write_NUM(first_page, 24, total_min_x);
  EEPROM_Write_NUM(first_page, 28, total_max_y);
  // reset day x and y min max values in eeprom
  EEPROM_Write_NUM(first_page, 32, day_reg);
  EEPROM_Write_NUM(first_page, 36, day_max_x);
  EEPROM_Write_NUM(first_page, 40, day_max_y);
  EEPROM_Write_NUM(first_page, 44, day_min_x);
  EEPROM_Write_NUM(first_page, 48, day_max_y);

  DisplayState = Eeprom_clear;
}

//------------------------------------------------------------------------------
// Function to print eeprom
void print_eeprom(void){
  char temp[16];
  int read_page;

  printf("eeprom print\n");

  // check if current page is the last page and set read page number
  if(max_page == 1){
    read_page = PAGE_NUM;
  }
  else{
    read_page = curr_page_num;
  }

  // print used pages
  for(int i = 0; i <= read_page; i++){
    printf("Page: %d\n", i);
    for(int j = 0; j < PAGE_SIZE; j += 4){
      sprintf(temp, "%f\n",EEPROM_Read_NUM(i, j));
      printf(temp);
    }
  }
}

//------------------------------------------------------------------------------
// Function to update main (first) page info
void update_page_info(char logic){
  // read first page info
  curr_page_num = EEPROM_Read_NUM(first_page, 0);
  curr_place = EEPROM_Read_NUM(first_page, 4);
  write_time = EEPROM_Read_NUM(first_page, 8);

  total_reg = EEPROM_Read_NUM(first_page, 12);
  total_max_x = EEPROM_Read_NUM(first_page, 16);
  total_max_y = EEPROM_Read_NUM(first_page, 20);
  total_min_x = EEPROM_Read_NUM(first_page, 24);
  total_min_y = EEPROM_Read_NUM(first_page, 28);

  day_reg = EEPROM_Read_NUM(first_page, 32);
  day_max_x = EEPROM_Read_NUM(first_page, 36);
  day_max_y = EEPROM_Read_NUM(first_page, 40);
  day_min_x = EEPROM_Read_NUM(first_page, 44);
  day_min_y = EEPROM_Read_NUM(first_page, 48);
  //-----------------------------------------------

  /*if(logic == 1){
    if((curr_page_num == 1) && (curr_place == 0)){
      printf("no data in eeprom\n");
    }
    else{
      read_from_eeprom();
      printf("last entry: x: %f, y: %f, curr page: %ld, curr place: %ld, write time: %ld\n", eeprom_x_read, eeprom_y_read, curr_page_num, curr_place, write_time);
    }
  }*/   
}

//------------------------------------------------------------------------------
// Function to write data to eeprom
void write_to_eeprom(float xdata, float ydata, long time){
  
  // page and cursor possition logic
  update_page_info(0);

  if(curr_place >= PAGE_SIZE){
    if(curr_page_num >= PAGE_NUM){
      curr_page_num = 1;
    }
    else{
      curr_page_num += 1;
    }
    curr_place = 0;
  }

  // write accelerometed readings to eeprom
  EEPROM_Write_NUM(curr_page_num, curr_place, xdata);
  curr_place += 4;
  EEPROM_Write_NUM(curr_page_num, curr_place, ydata);
  curr_place += 4;

  
  // echo data to console
  read_from_eeprom();
  printf("data write: x: %f, y: %f, curr page: %ld, curr place: %ld, write time: %ld\n", eeprom_x_read, eeprom_y_read, curr_page_num, curr_place, time);

  total(xdata, ydata);

  // write last entry info to first page
  EEPROM_Write_NUM(first_page, 0, curr_page_num);
  EEPROM_Write_NUM(first_page, 4, curr_place);
  EEPROM_Write_NUM(first_page, 8, time);
}

//------------------------------------------------------------------------------
// Function to read data from eeprom
void read_from_eeprom(void){
  eeprom_y_read = EEPROM_Read_NUM(curr_page_num, (curr_place - 8));
  eeprom_x_read = EEPROM_Read_NUM(curr_page_num, (curr_place - 4));
}

//------------------------------------------------------------------------------
// Function to calculate total max and min values
void total(float xdata, float ydata){
  static float x_max, y_max, x_min, y_min;

  // read total register
  float total_read = EEPROM_Read_NUM(first_page, 12);

  // if total register is not empty, read total max and min values and compare with current values
  if(total_read == total_reg){
    total_reg = EEPROM_Read_NUM(first_page, 12);
    x_max = EEPROM_Read_NUM(first_page, 16);
    y_max = EEPROM_Read_NUM(first_page, 20);
    x_min = EEPROM_Read_NUM(first_page, 24);
    y_min = EEPROM_Read_NUM(first_page, 28);
    if(xdata > x_max){
      x_max = xdata;
    }
    else if(xdata < x_min){
      x_min = xdata;
    }
    
    if(ydata > y_max){
      y_max = ydata;
    }
    else if(ydata < y_min){
      y_min = ydata;
    }
  }
  else{
    x_max = xdata;
    x_min = xdata;
    y_max = ydata;
    y_min = ydata;
  }

  // if total register is not full, increment it
  if(total_reg != 4088){
    total_reg += 1;
  }

  // write total max and min values to eeprom
  EEPROM_Write_NUM(first_page, 12, total_reg);

  EEPROM_Write_NUM(first_page, 16, x_max);
  EEPROM_Write_NUM(first_page, 20, y_max);
  EEPROM_Write_NUM(first_page, 24, x_min);
  EEPROM_Write_NUM(first_page, 28, y_min);

  // write total max and min values to variables
  total_max_x = x_max;
  total_max_y = y_max;
  total_min_x = x_min; 
  total_min_y = y_min;
}

//------------------------------------------------------------------------------
// Function to calculate day max and min values
void day(char logic){
  // read total entries
  float total = EEPROM_Read_NUM(first_page, 12);

  // if total entries are less than defined day readings, set day min max values to total register
  if(total <= day_readings){
    day_reg = total;
    day_max_x = EEPROM_Read_NUM(first_page, 16);
    day_max_y = EEPROM_Read_NUM(first_page, 20);
    day_min_x = EEPROM_Read_NUM(first_page, 24);
    day_min_y = EEPROM_Read_NUM(first_page, 28);

    // write day min max values to eeprom
    EEPROM_Write_NUM(first_page, 32, day_reg);
    EEPROM_Write_NUM(first_page, 36, day_max_x);
    EEPROM_Write_NUM(first_page, 40, day_max_y);
    EEPROM_Write_NUM(first_page, 44, day_min_x);
    EEPROM_Write_NUM(first_page, 48, day_min_y);
  }

  // if total entries are more than defined day readings, calculate min max values in a day
  if(logic == 1){
    static float last_x, last_y, cur_x, cur_y;
    int last_page, last_pos;

    day_reg = EEPROM_Read_NUM(first_page, 32);

    // calculate last entry page and position
    last_page = curr_page_num - page_ofset;
    last_pos = curr_place - pos_ofset;

    if(last_pos < 0){
      last_page -= 1;
      last_pos = PAGE_SIZE + last_pos;
    }
    
    // read last x and y values
    last_x = EEPROM_Read_NUM(last_page, (last_pos - 4));
    last_y = EEPROM_Read_NUM(last_page, (last_pos - 8));

    char complete = 0;

    // if last x and y values are out of range or last entry is not complete, calculate new day min max values
    if((last_x >= total_max_x || last_y >= total_max_y || last_x <= total_min_x || last_y <= total_min_y) || (complete == 0)){
      day_max_x = 0;
      day_max_y = 0;
      day_min_x = 0;
      day_min_y = 0;

      // calculate day min max values in a day
      for(int i = last_page; i <= curr_page_num; i++){
        // if last page is the same as current page, calculate min max values from last position to end of page
        if(i == last_page){
          for(int j = last_pos; j <=PAGE_SIZE; j += 8){
            // read x and y values
            cur_x = EEPROM_Read_NUM(i, (j - 8));
            cur_y = EEPROM_Read_NUM(i, (j - 4));
            
            if(cur_x > day_max_x){
              day_max_x = cur_x;
            }
            else if(cur_x < day_min_x){
              day_min_x = cur_x;
            }

            if(cur_y > day_max_y){
              day_max_y = cur_y;
            }
            else if(cur_y < day_min_y){
              day_min_y = cur_y;
            }
          }
          continue;
        }

        // if current iteration is not the last page nor start page, calculate min max values from start to end of page
        if(i != last_page && i != curr_page_num){
          for(int k = 0; k <= (PAGE_SIZE - 8); k += 8){
            // read x and y values
            cur_x = EEPROM_Read_NUM(i, k);
            cur_y = EEPROM_Read_NUM(i, (k + 4));
            
            if(cur_x > day_max_x){
              day_max_x = cur_x;
            }
            else if(cur_x < day_min_x){
              day_min_x = cur_x;
            }

            if(cur_y > day_max_y){
              day_max_y = cur_y;
            }
            else if(cur_y < day_min_y){
              day_min_y = cur_y;
            }
          }
        }

        // if current iteration is the last page, calculate min max values from start to current position
        if(i == curr_page_num){
          for(int m = 0; m <= (curr_place - 8); m += 8){
            // read x and y values
            cur_x = EEPROM_Read_NUM(i, m);
            cur_y = EEPROM_Read_NUM(i, m + 4);
            
            if(cur_x > day_max_x){
              day_max_x = cur_x;
            }
            else if(cur_x < day_min_x){
              day_min_x = cur_x;
            }

            if(cur_y > day_max_y){
              day_max_y = cur_y;
            }
            else if(cur_y < day_min_y){
              day_min_y = cur_y;
            }
          }
        }  
      }

      // write day min max values to eeprom
      EEPROM_Write_NUM(first_page, 36, day_max_x);
      EEPROM_Write_NUM(first_page, 40, day_max_y);
      EEPROM_Write_NUM(first_page, 44, day_min_x);
      EEPROM_Write_NUM(first_page, 48, day_max_y);
      complete = 1;
    } // end if
    else{
      // if last x and y values are in range, set day min max values to last values
      day_max_x = EEPROM_Read_NUM(first_page, 36);
      day_max_y = EEPROM_Read_NUM(first_page, 40);
      day_min_x = EEPROM_Read_NUM(first_page, 44);
      day_min_y = EEPROM_Read_NUM(first_page, 48);
    }
  }
}

//------------------------------------------------------------------------------
// Function to calculate time
void timer(char logic, long curr_time, u_int16_t* hrs, u_int16_t* min, u_int16_t* sec){
  if(logic == 1){
    *sec = curr_time / 1000;
    *min = *sec / 60;
    *hrs = *min / 60;

    while(*sec > 59 || *min > 59 || *hrs > 23){
      if(*sec >= 59){
        *sec -= 59;
      }
      else if(*min > 59){
        *min -= 59;
      }
      else if(*hrs > 23){
        *hrs = 0;
      }
    }
  }
  
  else if(logic == 0){
  if (*sec >= 59){
    *min += 1;
    *sec = 0;
    if (*min >= 59){
      *hrs += 1;
      *min = 0;
      if (*hrs >= 59){
        *hrs = 0;
      }   
    }
  }
  else
    *sec += 1;
  }
}
//------------------------------------------------------------------------------

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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