/******************************************************************************
 *
 * Module: KEYPAD
 *
 * File Name: keypad.h
 *
 * Description: Header file for The KEYPAD Driver.
 *
 * Author: Huda Abbdelnasser Taalap Haridy
 *         Abdelrahman Ali Mohamed Ali
 *******************************************************************************/
#ifndef KEYPAD_H_
#define KEYPAD_H_
#include "std_types.h"
/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * Description :
 * Initialization the required port for the Keypad.
 * Register(s) : GPIO_PUR, GPIO_DIR.
 */
void KEYPAD_INIT(void);


/*
 * Description :
 * Read the Keypad.
 * Register(s) : GPIO_DATA.
 */
uint8_t KEYPAD_READ(void);


#endif /* KEYPAD_H_ */
