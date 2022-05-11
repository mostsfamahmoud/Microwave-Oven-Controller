/***********************
 *
 * Module: LCD
 *
 * File Name: LCD.c
 *
 * Description: Source file for THE LCD Driver.
 * 
 * Authors: Sherin Sameh Abd El-Samad Ali 
 *          Esraa Amr Abdelmoneam Hassan Kandil
 *         
 ***************************/
#include "gpio.h"
#include "Macros.h"
#include "std_types.h"
#include "LCD_Macros.h"
#include "timer.h"
/***************************
 *                       Functions Definitions                             *
 ***************************/

/*
 * Description :
 * Initialization the required port for the LCD.
 * Register(s) : GPIO_DIR , GPIO_DATA 
 */
void LCD_init(void)
{
PORT_INIT(DATA_PORT); 
PORT_INIT(CTRL_PORT);
 GPIO_setPortDirection(DATA_PORT, 0xFF);
 GPIO_setPinDirection(CTRL_PORT, Enable,1);
 GPIO_setPinDirection(CTRL_PORT, RS, 1);
 GPIO_setPinDirection(CTRL_PORT, RW, 1);
 GPIO_writePin(CTRL_PORT, RW, 0);
 LCD_COMMAND(LCD_8BITS_M);
 Generic_delay_m_sec(1);
 LCD_COMMAND(DIS_ON_C_OFF);
 Generic_delay_m_sec(1);
 LCD_COMMAND(Clear_Screen);
 Generic_delay_m_sec(10);
LCD_COMMAND(ENTRY_MODE);
Generic_delay_m_sec(1);
}
/*
 * Description :
 * Initialization the required port for the Keypad.
 * Register(s) : GPIO_DATA
 */
void LCD_COMMAND(char command){
 GPIO_writePin(DATA_PORT,command);
    GPIO_writePin(CTRL_PORT,RS,0);
    sendPulse();
    Generic_delay_m_sec(1);
}
/*
 * Description :
 * Sending The pulse.
 * Register(s) : CTRL_PORT.
 */

static void sendPulse(void){
    GPIO_writePin(CTRL_PORT,Enable,1);
    Generic_delay_m_sec(2);
    GPIO_writePin(CTRL_PORT,Enable,0);
    Generic_delay_m_sec(2);
}

/*
 * Description :
 * Sending Char.
 * Register(s) : CTRL_PORT,DATA_PORT.
 */

void send_Char(char char){
    GPIO_writePin(DATA_PORT,char);
    GPIO_writePin(CTRL_PORT,RS,1);
    sendPulse();
    Generic_delay_m_sec(1);
}
/*
 * Description :
 * Sending String.
 * Register(s) : CTRL_PORT,DATA_PORT.
 */
void send_String(char *data){
    while((*data)!='0'){
        send_Char((*data));
        data++;
    }
}
/*
 * Description :
 * Clear Screen.
 * Register(s) : Clear_Screen.
 */

void LCD_Clear_Screen(){
    LCD_COMMAND(Clear_Screen);
    Generic_delay_m_sec(10);
}
/*
 * Description :
 * Move Curser.
 * Register(s) : C_FIRST_LINE,C_SEC_LINE.
 */

void LCD_Move_Curser(char row,char col){
    char position =0;
    if(row=0){
        position = (C_FIRST_LINE)+col-1;
    }else if(row =2){
        position = (C_SEC_LINE)+col-1;
    }else{
        position = C_FIRST_LINE;
    }
    LCD_COMMAND(position);
    Generic_delay_m_sec(1);
}
