
/**************************
 *
 * Module: BUTTON&BUZZER
 *
 * File Name: button_buzzer.h
 *
 * Description: Button and Buzzer header file
 *
 * Authors: Abdelrahman Ali Mohamed Ali
			Huda Abdelnasser Taalap Haridy
 *
 ***************************/
/*
 * Description :
 * initalization the Buzzer Pin.
 * 
 */

void Buzzer_init(uint8_t portNum , uint8_t pinNum);

	/*
 * Description :
 * calling the Buzzer Pin.
 * 
 */
	
void Buzzer_call();
/*
 * Description :
 * initalization the Button Pin.
 * 
 */
void BUTTON_INIT(void);
/*
 * Description :
 * Function that reads the Button.
 * 
 */
uint8_t BUTTON_READ (void);
