/******************************************************************************
 *
 * Module: BUTTON&BUZZER
 *
 * File Name: button_buzzer.c
 *
 * Description: Button and Buzzer c file
 *
 * Authors: Abdelrahman Ali Mohamed Ali
			Huda Abbdelnasser Taalap Haridy
 *
 *******************************************************************************/


#include "gpio.h"


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

