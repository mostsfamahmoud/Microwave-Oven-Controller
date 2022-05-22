/***********************
 *
 * Module: KEYPAD
 *
 * File Name: keypad.c
 *
 * Description: Source file for THE KEYPAD Driver.
 * 
 * Authors: Huda Abdelnasser Taalap Haridy 
 *          Abdelrahman Ali Mohamed Ali
 ***************************/

#include "keypad.h"

/***************************
 *                       Functions Definitions                             *
 ***************************/

/*
 * Description :
 * Initialization the required port for the Keypad.
 */

uint8_t array[4][4] =
    {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };

/*
 * Description :
 * Initialization the required port for the Keypad.
 */

void KEYPAD_INIT(void)
{

    GPIO_Init(PORTE_ID);
    GPIO_Init(PORTA_ID);
    GPIO_setPortDirection(PORTE_ID, 0x0F);
    GPIO_PORTA_PDR_R |= 0xF0;
    GPIO_PORTA_DATA_R &= 0xF0;
    GPIO_PORTE_DATA_R &=0X0F;
	
	
}

