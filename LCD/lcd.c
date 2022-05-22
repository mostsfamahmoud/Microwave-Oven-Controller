/******************************************************************************
 *
 * Module: LCD
 *
 * File Name: LCD.c
 *
 * Description: source file for The LCD Driver.
 *
 * Author: Sherin Sameh Abd El-Samad Ali
 *         Esraa Amr Abdelmoneam Hassan Kandil
 *******************************************************************************/

#include "lcd.h"

/*******************************************************************************
 *                           Functions Definitions                             *
 *******************************************************************************/

/*
 * Description :
 * Initialization of the LCD.
 * Register(s) : GPIO_DIR , GPIO_DATA
 */


void LCD_INIT(void)
{
    GPIO_Init(PORTA_ID);
    GPIO_Init(PORTB_ID);
    GPIO_setPortDirection(DATA_PORT, 0xFF);
    GPIO_setPinDirection(CTRL_PORT, Enable, PIN_OUTPUT);
    GPIO_setPinDirection(CTRL_PORT, RS, PIN_OUTPUT);
    lcd_command(Function_set_8bit);
    Generic_delay_m_sec(1);
    lcd_command(moveCursorRight); // set to entry mode
    Generic_delay_m_sec(1);
    lcd_command(cursorBlink); //  display is on and cursor blinking
    Generic_delay_m_sec(1);
    lcd_command(clear_display); // clear screen
    Generic_delay_m_sec(2)
    lcd_command(returnHome); // return home
    Generic_delay_m_sec(2);
}
/*
 * Description :
 * send commands to LCD.
 * Register(s) : GPIO_DATA.
 */
void lcd_command(unsigned char cmd)
{
    GPIO_writePort(DATA_PORT, cmd);
    GPIO_writePort(CTRL_PORT, 0x08);
    Generic_delay_m_sec(1);
    GPIO_writePort(CTRL_PORT, 0x00); 
    Generic_delay_m_sec(50);

    return;
}
/*
 * Description :
 * Sending Char.
 * Register(s) : CTRL_PORT,DATA_PORT.
 */
void send_char(unsigned char data) //-----> recive only one char
{
    GPIO_writePort(DATA_PORT, data);
    // GPIO_PORTB_DATA_R=data;
    GPIO_writePort(CTRL_PORT, 0x0C);
    // GPIO_PORTE_DATA_R=0X05;
    Generic_delay_m_sec(1);
    GPIO_writePort(CTRL_PORT, 0x04);
    // GPIO_PORTE_DATA_R=0X01;
    Generic_delay_m_sec(50);

    return;
}
/*
 * Description :
 * Sending String.
 * Register(s) : CTRL_PORT,DATA_PORT.
 */
void send_String(unsigned char *data)
{
    while (*data)
    {
        send_char(*data);
        data++;
    }
}
/*
 * Description :
 * move cursor to write data on preffered row and column
 * Register(s) : CTRL_PORT,DATA_PORT.
 */
void LCD_Move_Curser(unsigned char row, unsigned char col)
{
    unsigned char position = 0;

    if (row == 1)
    {
        position = (FirstRow) + col - 1;
    }
    else if (row == 2)
    {
        position = (SecondRow) + col - 1;
    }
    else
    {
        position = FirstRow;
    }
    lcd_command(position);
    Generic_delay_m_sec(1);
}
