/******************************************************************************
 *
 * Module: GPIO
 *
 * File Name: portF.h
 *
 * Description: LEDs & Built-in Switches Driver
 *
 * Authors: Mostafa Mahmoud Ali
 *
 *******************************************************************************/

#include "portF.h"

/*******************************************************************************
 *                           Functions Definitions                             *
 *******************************************************************************/


/*
 * Description :
 * initialize LED (PORTF-->123).
 * Note: 1) Should be used before initalize SW1 or SW2 as GPIO_writePort clears DATA reg before writing to it 
 *       2) LEDs will be ON when function is called
 * 
 * If the input port number is not correct, The function will not handle the request.
 */

void RBGLED_init(){

    GPIO_ConfigurationType red = {PORTF_ID , PIN1_ID, DIGITAL,
			NORMAL_MODE , GPIO_PORT_MODE, PIN_OUTPUT , NONE };

    GPIO_ConfigurationType blue = {PORTF_ID , PIN2_ID, DIGITAL,
			NORMAL_MODE , GPIO_PORT_MODE, PIN_OUTPUT , NONE };

    GPIO_ConfigurationType green = {PORTF_ID , PIN3_ID, DIGITAL,
			NORMAL_MODE , GPIO_PORT_MODE, PIN_OUTPUT , NONE };
    
    GPIO_ConfigurationType LEDs[] = { red , blue , green };


    for (uint8_t i = 0 ; i < 3; i++){
        GPIO_init(&LEDs[i]);
    }

    GPIO_writePort(PORTF_ID,PF123_mask);
}


/*
 * Description :
 * clears PF123 then update the mentioned pins with new values of data in PORTF
 * 
 */

void RBG_OUTPUT(uint8_t Data){
    GPIO_PORTF_DATA_R &= ~0x0E;         //reset PF123
    GPIO_PORTF_DATA_R |= Data;          //WRITE NEW DATA ON PF123
}



/*
 * Description :
 *       function that initializes PortF Pins(0,4) as Digital Input that will be connected to a switch.
 * Note:
 *       1) Initializtion will depend on the value of SwitchNum that will be passed as a parameter.
 *       2) SO we can initalize one only or both of them at the same time.
 */


void SW_init(uint8_t SwitchNum){

    GPIO_ConfigurationType SW1 = { PORTF_ID , PIN4_ID, DIGITAL,
			NORMAL_MODE , GPIO_PORT_MODE, PIN_INPUT , PULL_UP };

    GPIO_ConfigurationType SW2 = { PORTF_ID , PIN0_ID, DIGITAL,
			NORMAL_MODE , GPIO_PORT_MODE, PIN_INPUT , PULL_UP };

    GPIO_ConfigurationType SW[] = { SW1 , SW2 }; 


    switch(SwitchNum){
        
        //enable SW1 only
        case 1 :

            GPIO_init(&SW1);
            CLEAR_BIT(GPIO_PORTF_DATA_R,PIN4_ID);
            break;

        //enable SW2 only
        case 2:

            GPIO_init(&SW2);
            CLEAR_BIT(GPIO_PORTF_DATA_R,PIN0_ID);
            break;

        // enable Both
        case 12:

            for (uint8_t i = 0; i < 2; i++){
                GPIO_init(&SW[i]);
            }

            CLEAR_BIT(GPIO_PORTF_DATA_R,PF04_mask);
            break;

        default :
            break;
    
    }
}



/*
 * Description : Function that reads PORTF pin4.
 */

uint8_t SW1_INPUT(void){                           // tells me that switch is pressed or not
    return GPIO_readPin(PORTF_ID,PIN4_ID);         // pressed (return 0) unpressed ( return 1) 
}



/*
 * Description : Function that reads PORTF pin0.
 */

uint8_t SW2_INPUT(void){                           // tells me that switch is pressed or not
    return GPIO_readPin(PORTF_ID,PIN0_ID);         // pressed (return 0) unpressed ( return 1) 
}



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

uint8_t SW_INPUT(void){                           
    return GPIO_PORTF_DATA_R & PF04_mask;         // pressed (return 0x00) unpressed ( return 0x11)
                                                                                          
}














