/******************************************************************************
 *
 * Module: GPIO
 *
 * File Name: gpio.h
 *
 * Description: Header file for The TivaC GPIO Driver.
 *
 * Author: Mostafa Mahmoud Ali
 *         Mai Esmail Gamal
 
 *******************************************************************************/

#ifndef GPIO_H_
#define GPIO_H_

#include "std_types.h"
#include "altFunc.h"
#include "tm4c123gh6pm.h"
#include "Macros.h"

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

#define NUM_OF_PORTS 			6
#define NUM_OF_PINS_PER_PORT 	8

// To Select Between PORTs (PORT NUM)

#define PORTA_ID         0
#define PORTB_ID         1
#define PORTC_ID         2
#define PORTD_ID         3
#define PORTE_ID         4
#define PORTF_ID         5

// To Select which PIN to operate on (PIN NUM)

#define PIN0_ID          0
#define PIN1_ID          1
#define PIN2_ID          2
#define PIN3_ID          3
#define PIN4_ID          4
#define PIN5_ID          5
#define PIN6_ID          6
#define PIN7_ID          7

// Each PIN has 8 bits from (0 to 7)

#define PA0		0
#define PA1		1
#define PA2		2
#define PA3		3
#define PA4		4
#define PA5		5
#define PA6		6
#define PA7		7

#define PB0		0
#define PB1		1
#define PB2		2
#define PB3		3
#define PB4		4
#define PB5		5
#define PB6		6
#define PB7		7


#define PC0		0
#define PC1		1
#define PC2		2
#define PC3		3
#define PC4		4
#define PC5		5
#define PC6		6
#define PC7		7


#define PD0		0
#define PD1		1
#define PD2		2
#define PD3		3
#define PD4		4
#define PD5		5
#define PD6		6
#define PD7		7


#define PE0		0
#define PE1		1
#define PE2		2
#define PE3		3
#define PE4		4
#define PE5		5
#define PE6		6
#define PE7		7

#define PF0		0
#define PF1		1
#define PF2		2
#define PF3		3
#define PF4		4
#define PF5		5
#define PF6		6
#define PF7		7

#define UNLOCKING_VALUE  0x4C4F434B
#define REG_UNLOCK(LOCK_REG) (LOCK_REG = UNLOCKING_VALUE)

#define ERROR_VALUE 0xFF


/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/

typedef enum
{
	PIN_OUTPUT , PIN_INPUT

}GPIO_PinDirectionType;

typedef enum{

	NONE , PULL_UP

}GPIO_PinInternalResistance;


/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * Description :
 * Configures the clock of a given port.
 * Register(s) : SYSCTL_RCGCGPIO_R & SYSCTL_PRGPIO_R.
 * If the input port number is not correct, The function will not handle the request.
 */
void GPIO_configurePortClock(uint8_t portNum);

/*
 * Description :
 * Initialization the required Port as Digital I/O.
 * Register(s) : RCGC2, GPIO_LOCK, GPIO_CR, GPIO_DEN, GPIO_AMSEL, GPIO_AFSEL, GPIO_PCTL.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */

void GPIO_Init(uint8_t PortNum);

/*
 * Description :
 * Setup the direction of the required Port.
 * Register(s) : GPIO_DIR.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */
void GPIO_setPortDirection(uint8_t portNum, uint8_t direction);

/*
 * Description :
 * Setup the direction of the required pin input/output.
 * Register(s) : GPIO_DIR.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */

void GPIO_setPinDirection(uint8_t portNum, uint8_t pinNum, GPIO_PinDirectionType direction);

/*
 * Description :
 * Write the value Logic High or Logic Low on the required pin.
 * Register(s) : GPIODATA
 * If the input port number or pin number are not correct, The function will return an error value (0xFF).
 */

uint8_t GPIO_readPin(uint8_t PortNum, uint8_t PinNum);

/*
 * Description :
 * Write the value Logic High or Logic Low on the required pin.
 * Register(s) : GPIODATA
 * If the input port number or pin number are not correct, The function will not handle the request.
 */

void GPIO_writePin(uint8_t PortNum, uint8_t PinNum, uint8_t Data);

/*
 * Description :
 * Enables the internal resistance of the specified pin in the specified port.
 * Internal pull up configuration are supported.
 * Register(s) : GPIOPUR .
 * If the input port number or pin number are not correct, The function will not handle the request.
 */
void GPIO_enablePullUp(uint8_t portNum, uint8_t pinNum, GPIO_PinInternalResistance res);

/*
 * Description :
 * Write the given value on the desired port.
 * Register(s) : GPIODATA
 * If the input port number is not correct, The function will not handle the request.
 */
void GPIO_writePort(uint8_t portNum, uint8_t value);

/*
 * Description :
 * Reads the current value of the desired port.
 * Register(s) : GPIODATA
 * If the input port number is not correct, The function will return an error value (0xFF).
 */
uint8_t GPIO_readPort(uint8_t portNum);



#endif /* GPIO_H_ */



