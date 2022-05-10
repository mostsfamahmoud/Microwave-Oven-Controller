/***********************
 *
 * Module: KEYPAD
 *
 * File Name: keypad.c
 *
 * Description: Source file for THE KEYPAD Driver.
 * 
 * Authors: Huda Abbdelnasser Taalap Haridy 
 *          Abdelrahman Ali Mohamed Ali
 ***************************/
#include "gpio.h"
#include "Macros.h"
#include "tm4c123gh6pm.h"
#include "std_types.h"
#include "keypad.h"

/***************************
 *                       Functions Definitions                             *
 ***************************/

/*
 * Description :
 * Initialization the required port for the Keypad.
 * Register(s) : GPIO_PUR, GPIO_DIR.
 */

void KEYPAD_INIT(void)
{
    PORT_INIT(PORTA_ID); 
    GPIO_setPortDirection(PORTA_ID, 0XF0); // ROWS>> outputs , columns>> inputs
    GPIO_enableInternalResistance(PORTA_ID, PIN0_ID, PULL_UP);
    GPIO_enableInternalResistance(PORTA_ID, PIN1_ID, PULL_UP);
    GPIO_enableInternalResistance(PORTA_ID, PIN2_ID, PULL_UP);
    GPIO_enableInternalResistance(PORTA_ID, PIN3_ID, PULL_UP);
}

