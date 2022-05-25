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
    SYSCTL_RCGCGPIO_R |= 0x10;
    while ((SYSCTL_PRGPIO_R & 0x10) == 0);
    GPIO_PORTE_LOCK_R = 0x4C4F434B;
    GPIO_PORTE_CR_R |= 0x10; 
    GPIO_PORTE_PCTL_R &= ~0x000F0000;
    GPIO_PORTE_AFSEL_R &= ~0x10; 
    GPIO_PORTE_AMSEL_R &= ~0x10;
    GPIO_PORTE_DIR_R |= 0x10; 
    GPIO_PORTE_DEN_R |= 0x10;
    GPIO_PORTE_DATA_R &= ~0x10; 
}

/* Description :
    Setting the buzzer pin to ON (1)
*/

void Buzzer_ON()
{
   
    GPIO_PORTE_DATA_R |= 0x10;  
}

void Buzzer_OFF()
{
    GPIO_PORTE_DATA_R &= ~0x10; 
}

/*
 * Description :
 * initalization the Button Pin.
 *
 */
void BUTTON_INIT(void)
{
    GPIO_configurePortClock(PORTD_ID);
    REG_UNLOCK(GPIO_PORTD_LOCK_R);
    GPIO_PORTD_AMSEL_R &= ~0x04; // disable analog function on PD2
    GPIO_PORTD_CR_R |= 0x04;
    GPIO_PORTD_PCTL_R &= ~0x00000F00;
    GPIO_PORTD_AFSEL_R &= ~0x04; // disable alternate func
    GPIO_PORTD_DIR_R &= ~0x04;
    GPIO_PORTD_DEN_R |= 0x04;
    GPIO_PORTD_PUR_R |= 0x04;
}
/*
 * Description :
 * Function that reads the Button.
 *
 */
uint8_t BUTTON_READ()
{
    return GPIO_readPin(PORTD_ID, PD2);
}

