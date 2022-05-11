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