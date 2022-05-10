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

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * Description :
 * Initialization the required port for the Keypad.
 * Register(s) : GPIO_PUR, GPIO_DIR.
 */
void KEYPAD_INIT(uint8_t portnum);

/*
 * Description :
 * Read the Keypad.
 * Register(s) : GPIO_DATA.
 */
uint8_t KEYPAD_READ(uint8_t portnum);

#endif /* KEYPAD_H_ */
