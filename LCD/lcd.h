/******************************************************************************
 *
 * Module: LCD
 *
 * File Name: LCD.h
 *
 * Description: Header file for The LCD Driver.
 *
 * Author: Sherin Sameh Abd El-Samad Ali
 *         Esraa Amr Abdelmoneam Hassan Kandil
 *******************************************************************************/
#ifndef LCD_H_
#define LCD_H_

#include "gpio.h"
#include "timer.h"
#include "portF.h"
#include "Button_Buzzer.h"

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

#define DATA_PORT 1 // (PORTB_ID)
#define CTRL_PORT 0 // (PORTA_ID)

// LCD control pins use the first 3 pins of CTRL PORT

#define RS        2
#define Enable    3

//**********************LCD COMMNANDS**********************

#define clear_display     0x01
#define returnHome        0x02
#define moveCursorRight   0x06
#define moveCursorLeft    0x08
#define shiftDisplayRight 0x1C
#define shiftDisplayLeft  0x18
#define cursorBlink       0x0F
#define cursorOff         0x0C
#define cursorOn          0x0E
#define Function_set_4bit 0x28
#define Function_set_8bit 0x38
#define Entry_mode        0x06
#define Function_8_bit    0x32
#define Set5x7FontSize    0x20
#define FirstRow          0x80
#define SecondRow         0xC0

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * Description :
 * Initialization of the LCD.
 * Register(s) : GPIO_DIR , GPIO_DATA
 */
void LCD_INIT(void);
/*
 * Description :
 * send commands to LCD.
 * Register(s) : GPIO_DATA.
 */
void lcd_command(unsigned char cmd);
/*
 * Description :
 * Sending Char.
 * Register(s) : CTRL_PORT,DATA_PORT.
 */
void send_char(unsigned char data);
/*
 * Description :
 * Sending String.
 * Register(s) : CTRL_PORT,DATA_PORT.
 */
void send_String(unsigned char *data);
/*
 * Description :
 * move cursor to write data on preffered row and column
 * Register(s) : CTRL_PORT,DATA_PORT.
 */
void LCD_Move_Curser(unsigned char row, unsigned char col);
