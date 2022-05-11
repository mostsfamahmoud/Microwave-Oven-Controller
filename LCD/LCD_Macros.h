/******************************************************************************
 *
 * Module: LCD Macros
 *
 * File Name: LCD_Macros.h
 *
 * Description: Commonly Used Macros
 *
 * Author: Sherin Sameh Abd El-Samad Ali 
 *         Esraa Amr Abdelmoneam Hassan Kandil
 *
 *******************************************************************************/
#include "gpio.h"


// data port of lcd is connected to PORT B

#define DATA_PORT 'PORTB_ID'

// control port of lcd is connected to port D

#define CTRL_PORT 'PORTD_ID'

// LCD control pins use the first 3 pins of port D 

#define Enable 'PD0'
#define RS 'PD1'
#define RW 'PD2'

//**********************LCD COMMNANDS**********************

#define DIS_ON_C_ON 0x0F // display on cursor on
#define DIS_OFF_C_OFF 0x08 // display off cursor off
#define DIS_ON_C_BLINK 0x0E // display  on but cursor is blinking
#define DIS_ON_C_OFF 0x0C // display on cursor off
#define LCD_8BITS_M 0x38 // use LCD in 8 bits mode
#define ENTRY_MODE 0x06 // shift cursor to the right ( increment cursor )
#define Clear_Screen 0x01 
#define Return_Home 0x02
#define shift_C_left 0x04 // shift cursor to the left ( decrement cursor)
#define shift_display_right 0x05 
#define shift_display_left 0x07
#define C_FIRST_LINE 0x80  // force cursor to begining of first line
#define C_SEC_LINE 0xC0 // force cursor to beggining of second line
#define SEC_LINE_ACTIVE 0x3C // activate second line
#define J_LINE2_P1 0xC1 // jump to the second line first position
#define J_LINE2_P2 0xC2 // jump to the second line second position
#define C_LINE1_P3 0x83 // cursor to line 1 position 3



//******************************************************************************


