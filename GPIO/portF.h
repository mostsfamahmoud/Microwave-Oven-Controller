/******************************************************************************
 *
 * Module: GPIO
 *
 * File Name: portF.h
 *
 * Description: Header file for LEDs & Built-in Switches Functions.
 *
 * Author: Mostafa Mahmoud Ali 
 *
 *******************************************************************************/

#ifndef PORTF_H_
#define PORTF_H_

#include "tm4c123gh6pm.h"
#include "Macros.h"
#include "gpio.h"

/*******************************************************************************
 *                                 Definitions                                  *
 *******************************************************************************/

#define RED 0x02
#define BLUE 0x04
#define GREEN 0x08
#define PF123_mask 0x0E
#define PF04_mask 0x11
#define PF_mask 0x20


/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/


/*
 * Description :
 * initialize LED (PORTF-->123).
 * Note: 1) Should be used before initalize SW1 or SW2 as GPIO_writePort clears DATA reg before writing to it 
 *       2) LEDs will be ON when function is called
 * 
 * If the input port number is not correct, The function will not handle the request.
 */

void RBGLED_init();


/*
 * Description :
 * clears PF123 then update the mentioned pins with new values of data in PORTF
 * 
 */

void RBG_OUTPUT(uint8_t Data);


/*
 * Description :
 *       function that initializes PortF Pins(0,4) as Digital Input that will be connected to a switch.
 * Note:
 *       1) Initializtion will depend on the value of SwitchNum that will be passed as a parameter.
 *       2) SO we can initalize one only or both of them at the same time.
 */


void SW_init(uint8_t SwitchNum);


/*
 * Description : Function that reads PORTF pin4.
 */

uint8_t SW1_INPUT(void);


/*
 * Description : Function that reads PORTF pin0.
 */

uint8_t SW2_INPUT(void);


/*
 * Description : 
                Function that reads PORTF PINS(0,4).
                Tells me that switch is pressed or not.

 * Expected return values:  
                        0x00 --> Both Pressed
                        0x01 --> SW1 Pressed
                        0x10 --> SW2 Pressed
                        0x11 --> Both UnPressed 
 */

uint8_t SW_INPUT(void);


#endif /* PORTF_H_ */