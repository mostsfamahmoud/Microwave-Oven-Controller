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

/*
 * Description :
 * Read the Keypad.
 * Register(s) : GPIO_DATA.
 */

uint8_t KEYPAD_READ(void)
{
    uint8_t array[4][4] = 
    {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };
    uint8_t row, column, x;
    uint8_t return_value =ERROR_VALUE; // NO_INPUTS FROM KEYPAD
    for(row=0; row<4; row++)
    {
        GPIO_writePort(PORTA_ID, 0XFF);
        GPIO_writePin(PORTA_ID , row+4, 0);
        for(column=0; column<4; column++)
        {
            x=GPIO_readPin(PORTA_ID ,column);
            if(x==0)
            {
                return_value = array[row][column];
                break;
            }
        }
        if(x==0)
        {
            break;  
        }      
    }
    return return_value;
}
