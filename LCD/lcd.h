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

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * Description :
 * Initialization the required port for the LCD.
 * Register(s) : GPIO_DIR , GPIO_DATA 
 */
void LCD_init(void);
/*
 * Description :
 * send commands to LCD.
 * Register(s) : GPIO_DATA.
 */
void LCD_COMMAND(char command);
/*
 * Description :
 * Sending The pulse.
 * Register(s) : CTRL_PORT.
 */

static void sendPulse(void);
/*
 * Description :
 * Sending Char.
 * Register(s) : CTRL_PORT,DATA_PORT.
 */

void send_Char(char char);
/*
 * Description :
 * Sending String.
 * Register(s) : CTRL_PORT,DATA_PORT.
 */

void send_String(char *data);
/*
 * Description :
 * Clear Screen.
 * Register(s) : Clear_Screen.
 */

void LCD_Clear_Screen();
/*
 * Description :
 * Move Curser.
 * Register(s) : C_FIRST_LINE,C_SEC_LINE.
 */

void LCD_Move_Curser(char row,char col);

#endif /* LCD_H_ */
