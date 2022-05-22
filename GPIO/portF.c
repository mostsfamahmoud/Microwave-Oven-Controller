/******************************************************************************
 *
 * Module: GPIO
 *
 * File Name: portF.h
 *
 * Description: LEDs & Built-in Switches Driver
 *
 * Authors: Mostafa Mahmoud Ali
            Mai Esmail Gamal
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

void RBGLED_init(void)
{
    GPIO_configurePortClock(PORTF_ID);
    REG_UNLOCK(GPIO_PORTF_LOCK_R);
    GPIO_PORTF_CR_R |= 0x0E; // allow change to pf123
    GPIO_PORTF_PCTL_R &= ~0x0000FFF0;
    GPIO_PORTF_AFSEL_R &= ~0x0E; // disable alternate func 00001110
    GPIO_PORTF_AMSEL_R &= ~0x0E;
    GPIO_PORTF_DIR_R |= 0x0E; // pf4=0 pf0=0
    GPIO_PORTF_DEN_R |= 0x0E;
    GPIO_PORTF_DATA_R &= ~0x0E;
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
 Description :
       function that initializes PortF Pin(4) as Digital Input that will be connected to a switch.
 Note:
       1) Initializtion will depend on the value of SwitchNum that will be passed as a parameter.
       2) SO we can initalize one only or both of them at the same time.
*/

void SW1_Init(void)
{

    GPIO_configurePortClock(PORTF_ID);
    REG_UNLOCK(GPIO_PORTF_LOCK_R);
    GPIO_PORTF_AMSEL_R &= ~0x10; // disable analog function on PF4
    GPIO_PORTF_CR_R |= 0x10;
    GPIO_PORTF_PCTL_R &= ~0x000F0000;
    GPIO_PORTF_AFSEL_R &= ~0x10; // disable alternate func
    GPIO_PORTF_DIR_R &= ~0x10;   // pf4=0
    GPIO_PORTF_DEN_R |= 0x10;
    GPIO_PORTF_PUR_R |= 0x10;
}

/*
Description :
   function that initializes PortF Pin(0) as Digital Input that will be connected to a switch.
Note:
   1) Initializtion will depend on the value of SwitchNum that will be passed as a parameter.
   2) SO we can initalize one only or both of them at the same time.
*/

void SW2_Init(void)
{

    GPIO_configurePortClock(PORTF_ID);
    REG_UNLOCK(GPIO_PORTF_LOCK_R);
    GPIO_PORTF_AMSEL_R &= ~0x01; // disable analog function on PF0
    GPIO_PORTF_CR_R |= 0x01;
    GPIO_PORTF_PCTL_R &= ~0x0000000F;
    GPIO_PORTF_AFSEL_R &= ~0x01; // disable alternate func
    GPIO_PORTF_DIR_R &= ~0x01;   // pf0=0
    GPIO_PORTF_DEN_R |= 0x01;
    GPIO_PORTF_PUR_R |= 0x01;
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














