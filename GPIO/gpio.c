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
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "Macros.h"

/*******************************************************************************
 *                             GLobal Variables                                *
 *******************************************************************************/

static volatile boolean g_isInit[NUM_OF_PORTS][NUM_OF_PINS_PER_PORT] = {FALSE};

static volatile boolean clockConfigured[NUM_OF_PORTS] = {FALSE};
/*******************************************************************************
 *                           Functions Definitions                             *
 *******************************************************************************/

/*
 * Description :
 * Setup the direction of the required pin input/output.
 * Register(s) : GPIO_DIR.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */
void GPIO_setPinDirection(uint8_t portNum , uint8_t pinNum , GPIO_PinDirectionType direction){

	if((pinNum >= NUM_OF_PINS_PER_PORT) || (portNum >= NUM_OF_PORTS))
	{
		/* Do Nothing */
	}
	else{
		volatile uint32_t * Direction_Registers[] = {&GPIO_PORTA_DIR_R ,
				&GPIO_PORTB_DIR_R ,
				&GPIO_PORTC_DIR_R,
				&GPIO_PORTD_DIR_R,
				&GPIO_PORTE_DIR_R,
				&GPIO_PORTF_DIR_R};

		switch(direction){

		case PIN_OUTPUT:
			SET_BIT(*Direction_Registers[portNum] , pinNum);
			break;

		case PIN_INPUT:
			CLEAR_BIT(*Direction_Registers[portNum] , pinNum);
			break;
		}
	}
}

/*
 * Description :
 * Setup the mode of the required pin analog/digital.
 * Register(s) : GPIO_DEN , GPIO_AMSEL.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */
void GPIO_setPinMode(uint8_t portNum , uint8_t pinNum , GPIO_PinMode mode){

	if((pinNum >= NUM_OF_PINS_PER_PORT) || (portNum >= NUM_OF_PORTS))
	{
		/* Do Nothing */
	}

	else{

		volatile uint32_t * DigitalEnable_Registers[] = {&GPIO_PORTA_DEN_R ,
				&GPIO_PORTB_DEN_R ,
				&GPIO_PORTC_DEN_R,
				&GPIO_PORTD_DEN_R,
				&GPIO_PORTE_DEN_R,
				&GPIO_PORTF_DEN_R};

	    volatile uint32_t * AnalogSelect_Registers[] = {&GPIO_PORTA_AMSEL_R ,
				&GPIO_PORTB_AMSEL_R ,
				&GPIO_PORTC_AMSEL_R,
				&GPIO_PORTD_AMSEL_R,
				&GPIO_PORTE_AMSEL_R,
				&GPIO_PORTF_AMSEL_R};

		switch(mode){

			case DIGITAL:
				SET_BIT(*DigitalEnable_Registers[portNum],pinNum);
				CLEAR_BIT(*AnalogSelect_Registers[portNum],pinNum);
				break;
			

			case ANALOG:
				CLEAR_BIT(*DigitalEnable_Registers[portNum],pinNum);
				SET_BIT(*AnalogSelect_Registers[portNum],pinNum);
				break;
			
		}		
	}
}



/*
 * Description :
 * Configures the clock of a given port.
 * Register(s) : RCGC2.
 * If the input port number is not correct, The function will not handle the request.
 */
void GPIO_configurePortClock(uint8_t portNum){

	uint32_t PORT_ClockControl[NUM_OF_PORTS]= { SYSCTL_RCGC2_GPIOA,SYSCTL_RCGC2_GPIOB
									           ,SYSCTL_RCGC2_GPIOC,SYSCTL_RCGC2_GPIOD
										       ,SYSCTL_RCGC2_GPIOE,SYSCTL_RCGC2_GPIOF
									        };

	if(portNum >= NUM_OF_PORTS || clockConfigured[portNum] == TRUE){
		/* Do Nothing */
	}
	else{

		
		switch (portNum)
		{
		case PORTA_ID:
			SYSCTL_RCGCGPIO_R |= PORT_ClockControl[0];
			while((SYSCTL_PRGPIO_R & PORT_ClockControl[0]) == 0);
			break;
		
		case PORTB_ID:
			SYSCTL_RCGCGPIO_R |= PORT_ClockControl[1];
			while((SYSCTL_PRGPIO_R & PORT_ClockControl[1]) == 0);
			break;
		
		case PORTC_ID:
			SYSCTL_RCGCGPIO_R |= PORT_ClockControl[2];
			while((SYSCTL_PRGPIO_R & PORT_ClockControl[2]) == 0);
			break;
		
		case PORTD_ID:
			SYSCTL_RCGCGPIO_R |= PORT_ClockControl[3];
			while((SYSCTL_PRGPIO_R & PORT_ClockControl[3]) == 0);
			break;
		
		case PORTE_ID:
			SYSCTL_RCGCGPIO_R |= PORT_ClockControl[4];
			while((SYSCTL_PRGPIO_R & PORT_ClockControl[4]) == 0);
			break;
		
		case PORTF_ID:
			SYSCTL_RCGCGPIO_R |= PORT_ClockControl[5];
			while((SYSCTL_PRGPIO_R & PORT_ClockControl[5]) == 0);
			break;
	
		default:
			break;
		}
		clockConfigured[portNum] = TRUE;
	}
}


/*
 * Description :
 * If the required pin is one of the NMI pins (PC3-0 , PD7 , PF0) ,it is unlocked and the corresponding bit in the commit register is set.
 * Otherwise, it does nothing.
 * Register(s) : GPIOLOCK , GPIOCR.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */

void GPIO_unlockPin(uint8_t portNum , uint8_t pinNum){

	if((pinNum >= NUM_OF_PINS_PER_PORT) || (portNum >= NUM_OF_PORTS)){
		/* Do Nothing */
	}

	else{

		switch(portNum){

		case PORTC_ID: /*DO NOT APPORACH THE JTAG PINS*/
			return ;


		case PORTD_ID:

			if(pinNum == PD7){

				REG_UNLOCK(GPIO_PORTD_LOCK_R);

				SET_BIT(GPIO_PORTD_CR_R , pinNum);
			}
			break;

		case PORTF_ID:

			if(pinNum == PF0){

				REG_UNLOCK(GPIO_PORTF_LOCK_R);

				SET_BIT(GPIO_PORTF_CR_R , pinNum);
			}
			break;
		}
	}
}


/*
 * Description :
 * Enables the internal resistance of the specified pin in the specified port.
 * Internal pull up and Internal pull down configurations are supported.
 * Register(s) : GPIOPUR , GPIOPDR.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */
void GPIO_enableInternalResistance(uint8_t portNum , uint8_t pinNum , GPIO_PinInternalResistance res){

	if((pinNum >= NUM_OF_PINS_PER_PORT) || (portNum >= NUM_OF_PORTS))
	{
		/* Do Nothing */
	}

	else{
		/*Write your code here ...*/
	}
}



/*
 * Description :
 * Sets the control of the specified pin in the specified port (Normal mode or Alternate Function).
 * Register(s) : GPIOAFSEL.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */
void GPIO_setPinControl(uint8_t portNum , uint8_t pinNum , GPIO_pinAFSEL select ){

	if((pinNum >= NUM_OF_PINS_PER_PORT) || (portNum >= NUM_OF_PORTS))
	{
		/* Do Nothing */
	}

	else{
		volatile uint32_t * AltFuncSelect_Registers[] = {&GPIO_PORTA_AFSEL_R,
				&GPIO_PORTB_AFSEL_R,
				&GPIO_PORTC_AFSEL_R,
				&GPIO_PORTD_AFSEL_R,
				&GPIO_PORTE_AFSEL_R,
				&GPIO_PORTF_AFSEL_R};

		switch(select){
			case NORMAL_MODE:
				CLEAR_BIT(*AltFuncSelect_Registers[portNum],pinNum);
				break;
			
			case ALTERNATE_FUNCTION:
				SET_BIT(*AltFuncSelect_Registers[portNum],pinNum);
				break;
		}
	}
}

/*
 * Description :
 * it configures the alternate function of a certain pin in a certain port.
 * Register(s) : GPIOPCTL.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */
void GPIO_setAlternateFunction(uint8_t portNum , uint8_t pinNum , uint8_t altFunc){

	if((pinNum >= NUM_OF_PINS_PER_PORT) || (portNum >= NUM_OF_PORTS))
	{
		/* Do Nothing */
	}

	else {
		/*Write your code here ...*/
	}
}

/*
 * Description :
 * Configures a specified pin at the specified port with the given configuration structure.
 *  If the port number or pin number are not correct, The function will not handle the request.
 */
void GPIO_init(GPIO_ConfigurationType *config_ptr){

	if(g_isInit[config_ptr->port_num][config_ptr->pin_num] == TRUE){

		/*Do Nothing*/

	}

	else {
		GPIO_configurePortClock(config_ptr->port_num);

		GPIO_unlockPin(config_ptr->port_num, config_ptr->pin_num);

		GPIO_setPinMode(config_ptr->port_num, config_ptr->pin_num,  config_ptr->mode);

		GPIO_setPinControl(config_ptr->port_num, config_ptr->pin_num, config_ptr->Select);

		if(config_ptr->Select == ALTERNATE_FUNCTION){

			GPIO_setAlternateFunction(config_ptr->port_num, config_ptr->pin_num, config_ptr->AlternateFunctionNo);

		}

		GPIO_setPinDirection(config_ptr->port_num, config_ptr->pin_num, config_ptr->direction);

		if(config_ptr->resistance != NONE)
			GPIO_enableInternalResistance(config_ptr->port_num, config_ptr->pin_num, config_ptr->resistance);

		g_isInit[config_ptr->port_num][config_ptr->pin_num] = TRUE;
	}
}

/*
 * Description :
 * Write the value Logic High or Logic Low on the required pin.
 * Register(s) : GPIODATA
 * If the input port number or pin number are not correct, The function will not handle the request.
 */
void GPIO_writePin(uint8_t portNum , uint8_t pinNum , uint8_t value){

	if((pinNum >= NUM_OF_PINS_PER_PORT) || (portNum >= NUM_OF_PORTS) || g_isInit[portNum][pinNum] == FALSE)
	{
		/* Do Nothing */
	}

	else{

		/*Write your code here ...*/

	}
}

/*
 * Description :
 * Write the value Logic High or Logic Low on the required pin.
 * Register(s) : GPIODATA
 * If the input port number or pin number are not correct, The function will return an error value (0xFF).
 */
uint8_t GPIO_readPin(uint8_t portNum ,uint8_t pinNum){


	if((pinNum >= NUM_OF_PINS_PER_PORT) || (portNum >= NUM_OF_PORTS)|| g_isInit[portNum][pinNum] == FALSE)
	{
		return ERROR_VALUE;
	}


	else{
		/*Write your code here and return the desired value instead of 0 ...*/
		return  0;
	}
}

/*
 * Description :
 * Write the given value on the desired port.
 * Register(s) : GPIODATA
 * If the input port number is not correct, The function will not handle the request.
 */
void GPIO_writePort(uint8_t portNum , uint8_t value){


	if(portNum >= NUM_OF_PORTS)
	{
		/* Do Nothing */
		return ;
	}


	for(uint8_t pin = PIN0_ID ; pin < NUM_OF_PINS_PER_PORT ; pin++){

		if(g_isInit[portNum][pin] == FALSE){
			/* Do Nothing */
			return;

		}
	}
	/*Write your code here  ...*/

}


/*
 * Description :
 * Reads the current value of the desired port.
 * Register(s) : GPIODATA
 * If the input port number is not correct, The function will return an error value (0xFF).
 */
uint8_t GPIO_readPort(uint8_t portNum){

	if(portNum >= NUM_OF_PORTS)
	{
		/* Do Nothing */
		return ERROR_VALUE;
	}


	for(uint8_t pin = PIN0_ID ; pin < NUM_OF_PINS_PER_PORT ; pin++){

		if(g_isInit[portNum][pin] == FALSE){

			return ERROR_VALUE;

		}
	}

	/*Write your code here and return the desired value instead of 0 ...*/

	return 0 ;
}

