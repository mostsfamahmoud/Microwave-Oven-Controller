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


#include "gpio.h"
#include "button_buzzer.h"


/* Description :

	Initialization of Buzzer by initialize a choosen pinNum
	and initialize the data of this pin as OFF (0) 
	*/
	
void Buzzer_init(uint8_t portNum , uint8_t pinNum){
	GPIO_ConfigurationType = {portNum , pinNum , DIGITAL , NORMAL_MODE, GPIO_PORT_MODE , PIN_OUTPUT , NONE};
	GPIO_writePin( portNum , pinNum , 0 );
}

/* Description :
	Setting the buzzer pin to ON (1) 
	*/
	
void Buzzer_call(){
	GPIO_writePin( portNum , pinNum , 1 );
}

/*
 * Description :
 * initalization the Button Pin.
 * 
 */
void BUTTON_INIT(uint8_t portNum, uint8_t pinNum)
{
    GPIO_ConfigurationType button = {portNum, pinNum, DIGITAL, NORMAL_MODE, GPIO_PORT_MODE, PIN_INPUT, PULL_UP};
}
/*
 * Description :
 * Function that reads the Button.
 * 
 */
uint8_t BUTTON_READ (uint8_t portNum, uint8_t pinNum)
{
   return GPIO_readPin(portNum, pinNum);
}

