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
