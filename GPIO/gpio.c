/******************************************************************************
 *
 * Module: GPIO
 *
 * File Name: gpio.c
 *
 * Description: Source file for The TivaC GPIO Driver.
 *
 * Author: Mostafa Mahmoud Ali
 *         Mai Esmail Gamal
 *
 *******************************************************************************/

#include "gpio.h"

/*******************************************************************************
 *                           Functions Definitions                             *
 *******************************************************************************/

/*
 * Description :
 * Configures the clock of a given port.
 * Register(s) : SYSCTL_RCGCGPIO_R & SYSCTL_PRGPIO_R.
 * If the input port number is not correct, The function will not handle the request.
 */
void GPIO_configurePortClock(uint8_t portNum)
{

    uint32_t PORT_ClockControl[NUM_OF_PORTS] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20};

    if (portNum >= NUM_OF_PORTS)
    {
        /* Do Nothing */
        return;
    }
    else
    {
        switch (portNum)
        {
        case PORTA_ID:
            SYSCTL_RCGCGPIO_R |= PORT_ClockControl[0];
            while ((SYSCTL_PRGPIO_R & PORT_ClockControl[0]) == 0);
            break;

        case PORTB_ID:
            SYSCTL_RCGCGPIO_R |= PORT_ClockControl[1];
            while ((SYSCTL_PRGPIO_R & PORT_ClockControl[1]) == 0);
            break;

        case PORTC_ID:
            SYSCTL_RCGCGPIO_R |= PORT_ClockControl[2];
            while ((SYSCTL_PRGPIO_R & PORT_ClockControl[2]) == 0);
            break;

        case PORTD_ID:
            SYSCTL_RCGCGPIO_R |= PORT_ClockControl[3];
            while ((SYSCTL_PRGPIO_R & PORT_ClockControl[3]) == 0);
            break;

        case PORTE_ID:
            SYSCTL_RCGCGPIO_R |= PORT_ClockControl[4];
            while ((SYSCTL_PRGPIO_R & PORT_ClockControl[4]) == 0);
            break;

        case PORTF_ID:
            SYSCTL_RCGCGPIO_R |= PORT_ClockControl[5];
            while ((SYSCTL_PRGPIO_R & PORT_ClockControl[5]) == 0);
            break;

        default:
            break;
        }
    }
}

/*
 * Description :
 * Initialization the required Port as Digital I/O.
 * Register(s) : RCGC2, GPIO_LOCK, GPIO_CR, GPIO_DEN, GPIO_AMSEL, GPIO_AFSEL, GPIO_PCTL.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */

void GPIO_Init(uint8_t PortNum)
{

    if (PortNum >= NUM_OF_PORTS)
    {
        return;
    }

    switch (PortNum)
    {
    case PORTA_ID:
    {
        GPIO_configurePortClock(PORTA_ID);
        REG_UNLOCK(GPIO_PORTA_LOCK_R);
        GPIO_PORTA_DEN_R = 0xFF;
        GPIO_PORTA_CR_R = 0xFF;
        GPIO_PORTA_AFSEL_R = 0x00;
        GPIO_PORTA_AMSEL_R = 0x00;
        GPIO_PORTA_PCTL_R = 0x00000000;
        break;
    }
    case PORTB_ID:
    {
        GPIO_configurePortClock(PORTB_ID);
        REG_UNLOCK(GPIO_PORTB_LOCK_R);
        GPIO_PORTB_DEN_R = 0xFF;
        GPIO_PORTB_CR_R = 0xFF;
        GPIO_PORTB_AFSEL_R = 0x00;
        GPIO_PORTB_AMSEL_R = 0x00;
        GPIO_PORTB_PCTL_R = 0x00000000;
        break;
    }
    case PORTC_ID:
    {
        GPIO_configurePortClock(PORTC_ID);
        REG_UNLOCK(GPIO_PORTC_LOCK_R);
        GPIO_PORTC_DEN_R = 0xFF;
        GPIO_PORTC_CR_R = 0xFF;
        GPIO_PORTC_AFSEL_R = 0x00;
        GPIO_PORTC_AMSEL_R = 0x00;
        GPIO_PORTC_PCTL_R = 0x00000000;
        break;
    }
    case PORTD_ID:
    {
        GPIO_configurePortClock(PORTD_ID);
        REG_UNLOCK(GPIO_PORTD_LOCK_R);
        GPIO_PORTD_DEN_R = 0xFF;
        GPIO_PORTD_CR_R = 0xFF;
        GPIO_PORTD_AFSEL_R = 0x00;
        GPIO_PORTD_AMSEL_R = 0x00;
        GPIO_PORTD_PCTL_R = 0x00000000;
        break;
    }
    case PORTE_ID:
    {
        GPIO_configurePortClock(PORTE_ID);
        REG_UNLOCK(GPIO_PORTE_LOCK_R);
        GPIO_PORTE_DEN_R = 0x3F;
        GPIO_PORTE_CR_R = 0x3F;
        GPIO_PORTE_AFSEL_R = 0x00;
        GPIO_PORTE_AMSEL_R = 0x00;
        GPIO_PORTE_PCTL_R = 0x00000000;
        break;
    }
    case PORTF_ID:
    {
        GPIO_configurePortClock(PORTF_ID);
        REG_UNLOCK(GPIO_PORTF_LOCK_R);
        GPIO_PORTF_DEN_R = 0x1F;
        GPIO_PORTF_CR_R = 0x1F;
        GPIO_PORTF_AFSEL_R = 0x00;
        GPIO_PORTF_AMSEL_R = 0x00;
        GPIO_PORTF_PCTL_R = 0x00000000;
        break;
    }
    }
}

/*
 * Description :
 * Setup the direction of the required Port.
 * Register(s) : GPIO_DIR.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */
void GPIO_setPortDirection(uint8_t portNum, uint8_t direction)
{

    if (portNum >= NUM_OF_PORTS)
    {
        /* Do Nothing */
        return;
    }

    switch (portNum)
    {
    case PORTA_ID:
        GPIO_PORTA_DIR_R = direction;
        break;
    case PORTB_ID:
        GPIO_PORTB_DIR_R = direction;
        break;
    case PORTC_ID:
        GPIO_PORTC_DIR_R = direction;
        break;
    case PORTD_ID:
        GPIO_PORTD_DIR_R = direction;
        break;
    case PORTE_ID:
        GPIO_PORTE_DIR_R = direction;
        break;
    case PORTF_ID:
        GPIO_PORTF_DIR_R = direction;
        break;
    }
}

/*
 * Description :
 * Setup the direction of the required pin input/output.
 * Register(s) : GPIO_DIR.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */

void GPIO_setPinDirection(uint8_t portNum, uint8_t pinNum, GPIO_PinDirectionType direction)
{

    if ((pinNum >= NUM_OF_PINS_PER_PORT) || (portNum >= NUM_OF_PORTS))
    {
        /* Do Nothing */
        return;
    }

    if (direction == PIN_OUTPUT)
    {
        switch (portNum)
        {
        case PORTA_ID:
            SET_BIT(GPIO_PORTA_DIR_R, pinNum);
            break;
        case PORTB_ID:
            SET_BIT(GPIO_PORTB_DIR_R, pinNum);
            break;
        case PORTC_ID:
            SET_BIT(GPIO_PORTC_DIR_R, pinNum);
            break;
        case PORTD_ID:
            SET_BIT(GPIO_PORTD_DIR_R, pinNum);
            break;
        case PORTE_ID:
            SET_BIT(GPIO_PORTE_DIR_R, pinNum);
            break;
        case PORTF_ID:
            SET_BIT(GPIO_PORTF_DIR_R, pinNum);
            break;
        }
    }
    else if (direction == PIN_INPUT)
    {
        switch (portNum)
        {
        case PORTA_ID:
            CLEAR_BIT(GPIO_PORTA_DIR_R, pinNum);
            break;
        case PORTB_ID:
            CLEAR_BIT(GPIO_PORTB_DIR_R, pinNum);
            break;
        case PORTC_ID:
            CLEAR_BIT(GPIO_PORTC_DIR_R, pinNum);
            break;
        case PORTD_ID:
            CLEAR_BIT(GPIO_PORTD_DIR_R, pinNum);
            break;
        case PORTE_ID:
            CLEAR_BIT(GPIO_PORTE_DIR_R, pinNum);
            break;
        case PORTF_ID:
            CLEAR_BIT(GPIO_PORTF_DIR_R, pinNum);
            break;
        }
    }
    else
    {
        return;
    }
}

/*
 * Description :
 * Write the value Logic High or Logic Low on the required pin.
 * Register(s) : GPIODATA
 * If the input port number or pin number are not correct, The function will return an error value (0xFF).
 */

uint8_t GPIO_readPin(uint8_t PortNum, uint8_t PinNum)
{

    if ((PinNum >= NUM_OF_PINS_PER_PORT) || (PortNum >= NUM_OF_PORTS))
    {
        return ERROR_VALUE;
    }

    switch (PortNum)
    {
    case PORTA_ID:
        return (GET_BIT(GPIO_PORTA_DATA_R, PinNum));
     
    case PORTB_ID:
        return (GET_BIT(GPIO_PORTB_DATA_R, PinNum));
       
    case PORTC_ID:
        return (GET_BIT(GPIO_PORTC_DATA_R, PinNum));
       
    case PORTD_ID:
        return (GET_BIT(GPIO_PORTD_DATA_R, PinNum));
       
    case PORTE_ID:
        return (GET_BIT(GPIO_PORTE_DATA_R, PinNum));
       
    case PORTF_ID:
        return (GET_BIT(GPIO_PORTF_DATA_R, PinNum));
       
    }
		return 0;
}
/*
 * Description :
 * Write the value Logic High or Logic Low on the required pin.
 * Register(s) : GPIODATA
 * If the input port number or pin number are not correct, The function will not handle the request.
 */

void GPIO_writePin(uint8_t PortNum, uint8_t PinNum, uint8_t Data)
{

    if ((PinNum >= NUM_OF_PINS_PER_PORT) || (PortNum >= NUM_OF_PORTS))
    {
        return;
    }

    switch (PortNum)
    {
    case PORTA_ID:

        WRITE_BIT(GPIO_PORTA_DATA_R, PinNum, Data);
        break;

    case PORTB_ID:

        WRITE_BIT(GPIO_PORTB_DATA_R, PinNum, Data);
        break;

    case PORTC_ID:

        WRITE_BIT(GPIO_PORTC_DATA_R, PinNum, Data);
        break;

    case PORTD_ID:

        WRITE_BIT(GPIO_PORTD_DATA_R, PinNum, Data);
        break;

    case PORTE_ID:

        WRITE_BIT(GPIO_PORTE_DATA_R, PinNum, Data);
        break;

    case PORTF_ID:

        WRITE_BIT(GPIO_PORTF_DATA_R, PinNum, Data);
        break;
    }
}

/*
 * Description :
 * Enables the internal resistance of the specified pin in the specified port.
 * Internal pull up configuration are supported.
 * Register(s) : GPIOPUR .
 * If the input port number or pin number are not correct, The function will not handle the request.
 */
void GPIO_enablePullUp(uint8_t portNum, uint8_t pinNum, GPIO_PinInternalResistance res)
{
    if ((pinNum >= NUM_OF_PINS_PER_PORT) || (portNum >= NUM_OF_PORTS))
    {
        /* Do Nothing */
        return;
    }

    if (res == PULL_UP)
    {
        switch (portNum)
        {
        case PORTA_ID:
            SET_BIT(GPIO_PORTA_PUR_R, pinNum);
            break;
        case PORTB_ID:
            SET_BIT(GPIO_PORTB_PUR_R, pinNum);
            break;
        case PORTC_ID:
            SET_BIT(GPIO_PORTC_PUR_R, pinNum);
            break;
        case PORTD_ID:
            SET_BIT(GPIO_PORTD_PUR_R, pinNum);
            break;
        case PORTE_ID:
            SET_BIT(GPIO_PORTE_PUR_R, pinNum);
            break;
        case PORTF_ID:
            SET_BIT(GPIO_PORTF_PUR_R, pinNum);
            break;
        }
    }

    else
    {
        return;
    }
}

/*
 * Description :
 * Write the given value on the desired port.
 * Register(s) : GPIODATA
 * If the input port number is not correct, The function will not handle the request.
 */
void GPIO_writePort(uint8_t portNum, uint8_t value)
{

    if (portNum >= NUM_OF_PORTS)
    {
        /* Do Nothing */
        return;
    }

    switch (portNum)
    {
    case PORTA_ID:
        GPIO_PORTA_DATA_R = value;
        break;

    case PORTB_ID:
        GPIO_PORTB_DATA_R = value;
        break;

    case PORTC_ID:
        GPIO_PORTC_DATA_R = value;
        break;

    case PORTD_ID:
        GPIO_PORTD_DATA_R = value;
        break;

    case PORTE_ID:
        GPIO_PORTE_DATA_R = value;
        break;

    case PORTF_ID:
        GPIO_PORTF_DATA_R = value;
        break;
    }
}

/*
 * Description :
 * Reads the current value of the desired port.
 * Register(s) : GPIODATA
 * If the input port number is not correct, The function will return an error value (0xFF).
 */
uint8_t GPIO_readPort(uint8_t portNum)
{

    if (portNum >= NUM_OF_PORTS)
    {
        /* Do Nothing */
        return ERROR_VALUE;
    }

    switch (portNum)
    {
    case PORTA_ID:

        return GPIO_PORTA_DATA_R;

    case PORTB_ID:

        return GPIO_PORTB_DATA_R;

    case PORTC_ID:

        return GPIO_PORTC_DATA_R;

    case PORTD_ID:

        return GPIO_PORTD_DATA_R;

    case PORTE_ID:

        return GPIO_PORTE_DATA_R;

    case PORTF_ID:

        return GPIO_PORTF_DATA_R;
    }
		return 0;
}
