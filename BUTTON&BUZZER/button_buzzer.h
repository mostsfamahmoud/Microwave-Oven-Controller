/**
 *
 * Module: BUTTON&BUZZER
 *
 * File Name: button_buzzer.h
 *
 * Description: Button and Buzzer Header file
 *
 * Authors: Abdelrahman Ali Mohamed Ali
            Huda Abdelnasser Taalap Haridy
 *
 **/
#ifndef Button_BuzzerH
#define Button_BuzzerH

#include "gpio.h"

/**
 *                              Functions Prototypes                           *
 **/

/ Description :

    Initialization of Buzzer 

/

void Buzzer_init(void);

/ Description :
    Setting the buzzer ON 
/

void Buzzer_ON(void);

/ Description :
    Setting the buzzer OFF 
/

void Buzzer_OFF(void);

/
