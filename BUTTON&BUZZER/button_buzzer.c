/******************************************************************************
 *
 * Module: BUTTON&BUZZER
 *
 * File Name: button_buzzer.c
 *
 * Description: Button and Buzzer c file
 *
 * Authors: Abdelrahman Ali Mohamed Ali
            Huda Abdelnasser Taalap Haridy
 *
 *******************************************************************************/

#include "Button_Buzzer.h"

/*******************************************************************************
 *                           Functions Definitions                             *
 *******************************************************************************/

/* Description :

    Initialization of Buzzer by initialize a choosen pinNum
    and initialize the data of this pin as OFF (0)
	
*/

void Buzzer_init(void)
{
    GPIO_configurePortClock(PORTD_ID);
    REG_UNLOCK(GPIO_PORTD_LOCK_R);
    GPIO_PORTD_AMSEL_R &= ~0x40; // disable analog function on PD6
    GPIO_PORTD_CR_R |= 0x40;
    GPIO_PORTD_PCTL_R &= ~0x0F000000;
    GPIO_PORTD_AFSEL_R &= ~0x40; // disable alternate func
    GPIO_PORTD_DIR_R |= 0x40;
    GPIO_PORTD_DEN_R |= 0x40;
    GPIO_PORTD_DATA_R &= ~0x40;
}

/* Description :
    Setting the buzzer ON 
*/

void Buzzer_ON()
{
    GPIO_writePin(PORTD_ID, PD6, LOGIC_HIGH);
}

/* Description :
    Setting the buzzer OFF
*/

void Buzzer_OFF()
{
    GPIO_writePin(PORTD_ID, PD6, LOGIC_LOW);
}
