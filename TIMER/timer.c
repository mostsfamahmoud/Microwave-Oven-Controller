/***********************
 *
 * Module: systick timer
 *
 * File Name: timer.c
 *
 * Description: Source file for timer Driver.
 * 
 * Authors: Sherin Sameh Abd El-Samad Ali 
 *          Esraa Amr Abdelmoneam Hassan Kandil
 ***************************/
#include "timer.h"
/***************************
 *                       Functions Definitions                             *
 ***************************/

/*
 * Description : 
 * initialising the systick timer and the delay function 
 * Register(s) : NVIC_ST_CTRL_R , NVIC_ST_RELOAD_R ,NVIC_ST_CURRENT_R
 */
void systic_wait(uint32_t delay){
    NVIC_ST_CTRL_R = 0x00;
    NVIC_ST_RELOAD_R = delay -1;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0x05;
    while((NVIC_ST_CTRL_R & 0x00010000)==0){}
}

/*
 * Description : 
 * delay for one millisecond
 */

void Generic_delay_m_sec(uint32_t m_sec){
uint32_t i;

	for(i=0; i< m_sec; i++)
	{
		systic_wait(16000);
	}
}

/*
 * Description : 
 * delay for one second
 */
void Generic_delay_sec(uint32_t sec){
uint32_t i;
	for(i=0; i< sec; i++)
	{
		Generic_delay_m_sec(1000);
	}
}

/*
 * Description : 
 * delay for one minute
 */
void Generic_delay_min(uint32_t mins){
uint32_t i;
	for(i=0; i< mins; i++)
	{
		Generic_delay_sec(60);
	}
}
/*
 * Description :
 * delay for one micro.
 */
void Generic_delay_micro(uint32_t micro)
{
	uint32_t i;

	for (i = 0; i < micro; i++)
	{
		Systic_wait(16);
	}
}
